/**
 * swarm_app.c
 *
 * Crazyflie 2.1+ P2P Swarm Application
 * Decks required: Flow deck v2 (position), AI deck (detection)
 *
 * What this does
 * ──────────────
 *  • Every BROADCAST_INTERVAL_MS each drone broadcasts a SwarmPacket
 *    containing its own position (from the Kalman estimator via Flow deck)
 *    and the latest detection result from the AI deck (none / human / object).
 *
 *  • When a neighbour's packet is received, the drone logs to the debug
 *    console: "Drone N sees a HUMAN / OBJECT at (x, y, z)".
 *
 *  • A simple collision-avoidance step runs every broadcast cycle: if the
 *    nearest active neighbour is closer than MIN_SEPARATION_M the drone
 *    issues a High-Level Commander GoTo that moves it MIN_SEPARATION_M away
 *    in the horizontal plane.
 *
 *  • The network is fully dynamic: drones that have not been heard for
 *    DRONE_TIMEOUT_MS are considered offline and ignored automatically.
 *    Works with 1..MAX_DRONES drones without any reconfiguration.
 *
 * Radio setup  ← READ THIS
 * ──────────────────────────
 *  ALL drones MUST be on the SAME radio channel (e.g. channel 80) for P2P
 *  to work.  Give each drone a unique 5-byte address whose last byte is its
 *  ID (1..16):
 *      Drone 1  → E7:E7:E7:E7:01
 *      Drone 2  → E7:E7:E7:E7:02
 *      …
 *      Drone 16 → E7:E7:E7:E7:10
 *
 *  Connecting the PC to individual drones while P2P is running causes radio
 *  interference.  Use USB for monitoring/debugging during flight.
 *
 * Build (WSL on Windows)
 * ──────────────────────
 *  1. Open a WSL terminal and navigate to crazyflie-firmware/
 *  2. sudo apt install gcc-arm-none-eabi make python3-pip
 *  3. make cf2_defconfig
 *  4. cd examples/swarm_app && make
 *  5. Flash each drone:
 *       make cload CLOAD_CMDS="--addr E7E7E7E701"   # change last byte per drone
 *
 * AI deck payload convention (adapt if your GAP8 firmware differs)
 * ────────────────────────────────────────────────────────────────
 *  The GAP8 sends a CPX packet on CPX_F_APP where data[0] encodes:
 *      0 = no detection
 *      1 = human detected
 *      2 = object detected
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "radiolink.h"
#include "configblock.h"
#include "estimator_kalman.h"
#include "crtp_commander_high_level.h"
#include "cpx.h"
#include "stabilizer_types.h"
#include "supervisor.h"
#include "param.h"

#define DEBUG_MODULE "SWARM"
#include "debug.h"

/* ── Tunable constants ──────────────────────────────────────────────── */
#define MAX_DRONES             17u   /* absolute hard cap; supports IDs 1-16 */
#define P2P_PORT_SWARM         0x02  /* port tag; must differ from DTR's 0x09 */
#define BROADCAST_INTERVAL_MS  100u  /* how often to send our state      */
#define DRONE_TIMEOUT_MS       1000u /* drop a peer after this silence   */
#define MIN_SEPARATION_M       0.167f /* trigger avoidance below this (m) */
#define AVOIDANCE_STEP_M       0.3f  /* how far to move per avoidance step */
#define AVOIDANCE_DURATION_S   0.15f /* HL-commander duration for the step */

/* ── Detection types (must match GAP8 firmware output) ─────────────── */
typedef enum {
    DETECTION_NONE   = 0,
    DETECTION_HUMAN  = 1,
    DETECTION_OBJECT = 2,
} DetectionType;

/* ── P2P wire format ────────────────────────────────────────────────── */
/*  15 bytes – well within the 60-byte P2P_MAX_DATA_SIZE limit.          */
typedef struct __attribute__((packed)) {
    uint8_t sourceId;   /* drone ID derived from radio address last byte */
    float   x;          /* position in metres (Kalman estimator)         */
    float   y;
    float   z;
    uint8_t detection;  /* DetectionType                                 */
    uint8_t seqNum;     /* rolling counter – used to drop duplicates     */
} SwarmPacket;

/* compile-time guard */
_Static_assert(sizeof(SwarmPacket) <= P2P_MAX_DATA_SIZE,
               "SwarmPacket exceeds P2P_MAX_DATA_SIZE");

/* ── Per-peer runtime state ─────────────────────────────────────────── */
typedef struct {
    bool     active;
    float    x, y, z;
    uint8_t  detection;
    uint8_t  lastSeq;
    uint32_t lastSeenMs;
} PeerState;

static PeerState        peers[MAX_DRONES];
static SemaphoreHandle_t peersMutex;
static uint8_t           myId;

/*
 * Home offsets — physical starting position of each drone in the SHARED
 * coordinate frame (drone 1 is the origin at (0, 0)).
 *
 * Layout assumption: drones placed in a straight line along the X axis,
 * 0.5 m apart.  Change the values to match YOUR actual placement.
 *
 *   [D1]  [D2]  [D3]  [D4] ...
 *    0.0   0.5   1.0   1.5  ...
 *
 * Index = drone ID (last byte of the radio address, 0x01 – 0x0F).
 * ID 0 is unused; all others default to a 0.5 m linear spacing.
 */
static const float kHomeX[MAX_DRONES] = {
    0.0f,  /* ID  0 — unused                   */
    0.0f,  /* ID  1 (E7E7E7E701) — ORIGIN       */
    0.5f,  /* ID  2 (E7E7E7E702)                */
    1.0f,  /* ID  3 (E7E7E7E703)                */
    1.5f,  /* ID  4 (E7E7E7E704)                */
    2.0f,  /* ID  5 (E7E7E7E705)                */
    2.5f,  /* ID  6 (E7E7E7E706)                */
    3.0f,  /* ID  7 (E7E7E7E707)                */
    3.5f,  /* ID  8 (E7E7E7E708)                */
    4.0f,  /* ID  9 (E7E7E7E709)                */
    4.5f,  /* ID 10 (E7E7E7E70A)                */
    5.0f,  /* ID 11 (E7E7E7E70B)                */
    5.5f,  /* ID 12 (E7E7E7E70C)                */
    6.0f,  /* ID 13 (E7E7E7E70D)                */
    6.5f,  /* ID 14 (E7E7E7E70E)                */
    7.0f,  /* ID 15 (E7E7E7E70F)                */
    7.5f,  /* ID 16 (E7E7E7E710) — spare        */
};
static const float kHomeY[MAX_DRONES] = {
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
};

static float homeX = 0.0f;
static float homeY = 0.0f;

/* Latest detection from our own AI deck (written by CPX callback) */
static volatile DetectionType currentDetection = DETECTION_NONE;

/* ── Helpers ─────────────────────────────────────────────────────────── */
static inline float dist3(float ax, float ay, float az,
                           float bx, float by, float bz)
{
    float dx = ax - bx, dy = ay - by, dz = az - bz;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ── AI deck – CPX message handler ─────────────────────────────────── */
/*
 * Called in the CPX task context whenever the GAP8 sends a CPX_F_APP
 * packet.  We read data[0] as the detection class.
 *
 * Adapt this function if your GAP8 firmware uses a different encoding.
 */
static void aiDeckHandler(const CPXPacket_t *cpxPkt)
{
    if (cpxPkt->route.function != CPX_F_APP) return;
    if (cpxPkt->dataLength < 1)              return;

    switch (cpxPkt->data[0]) {
        case 1:  currentDetection = DETECTION_HUMAN;  break;
        case 2:  currentDetection = DETECTION_OBJECT; break;
        default: currentDetection = DETECTION_NONE;   break;
    }
}

/* ── P2P receive callback ───────────────────────────────────────────── */
/*
 * Called in the syslink task context (NOT an ISR) whenever a P2P packet
 * arrives.  Keep it short; use the mutex to protect the shared peers[].
 */
static void p2pCallback(P2PPacket *p)
{
    /* filter by port and expected payload size */
    if (p->port != P2P_PORT_SWARM)       return;
    if (p->size != sizeof(SwarmPacket))  return;

    SwarmPacket pkt;
    memcpy(&pkt, p->data, sizeof(SwarmPacket));

    uint8_t id = pkt.sourceId;
    if (id >= MAX_DRONES || id == myId)  return;

    if (xSemaphoreTake(peersMutex, 0) != pdTRUE) return; /* drop on contention */

    PeerState *peer = &peers[id];

    /* duplicate-filter: ignore if we already have this exact seq from an
       active peer (re-broadcasts cause the same packet to be received twice) */
    if (peer->active && pkt.seqNum == peer->lastSeq) {
        xSemaphoreGive(peersMutex);
        return;
    }

    peer->active     = true;
    peer->x          = pkt.x;
    peer->y          = pkt.y;
    peer->z          = pkt.z;
    peer->detection  = pkt.detection;
    peer->lastSeq    = pkt.seqNum;
    peer->lastSeenMs = T2M(xTaskGetTickCount());

    xSemaphoreGive(peersMutex);

    /* log every received P2P packet to the debug console */
    DEBUG_PRINT("P2P rx drone %u: pos=(%.2f,%.2f,%.2f) det=%u\n",
                (unsigned)id,
                (double)pkt.x, (double)pkt.y, (double)pkt.z,
                (unsigned)pkt.detection);

    if (pkt.detection == DETECTION_HUMAN) {
        DEBUG_PRINT("Drone %u sees a HUMAN  at (%.2f, %.2f, %.2f)\n",
                    (unsigned)id,
                    (double)pkt.x, (double)pkt.y, (double)pkt.z);
    } else if (pkt.detection == DETECTION_OBJECT) {
        DEBUG_PRINT("Drone %u sees an OBJECT at (%.2f, %.2f, %.2f)\n",
                    (unsigned)id,
                    (double)pkt.x, (double)pkt.y, (double)pkt.z);
    }
}

/* ── App entry point ────────────────────────────────────────────────── */
void appMain(void)
{
    /*
     * Derive drone ID from the last byte of the 5-byte radio address.
     * Address E7:E7:E7:E7:NN  →  id = NN   (must be unique, 0..9)
     */
    uint64_t addr = configblockGetRadioAddress();
    myId = (uint8_t)(addr & 0xFFu);

    /* Look up this drone's home offset from the table (ID 1 = origin). */
    if (myId < MAX_DRONES) { homeX = kHomeX[myId]; homeY = kHomeY[myId]; }

    peersMutex = xSemaphoreCreateMutex();
    memset(peers, 0, sizeof(peers));

    DEBUG_PRINT("Swarm app starting | id=%u | home=(%.2f,%.2f) | max_peers=%u\n",
                (unsigned)myId, (double)homeX, (double)homeY, MAX_DRONES);

    /* Enable the High-Level Commander so CFClient Takeoff/Land/GoTo work.
     * commander.enHighLevel defaults to 0; setting it to 1 makes the
     * CFClient Takeoff/Land/GoTo buttons active. */
    paramSetInt(paramGetVarId("commander", "enHighLevel"), 1);

    /* Register AI deck CPX handler */
    cpxRegisterAppMessageHandler(aiDeckHandler);

    /* Register P2P receive callback */
    p2pRegisterCB(p2pCallback);

    /* Let the Kalman estimator converge after boot */
    vTaskDelay(M2T(3000));

    /* Prepare a reusable TX packet (port and size are constant) */
    static P2PPacket txPkt;
    txPkt.port = P2P_PORT_SWARM;
    txPkt.size = (uint8_t)sizeof(SwarmPacket);

    uint8_t seqNum    = 0u;
    uint32_t printTick = 0u;   /* throttle status prints to once per second */
    float    lastZ     = 0.0f; /* track vertical movement */

    for (;;) {
        /* ── 1. Read own position from Kalman estimator ── */
        point_t pos;
        estimatorKalmanGetEstimatedPos(&pos);

        /* ── 1a. Status print every 10 cycles (~1 second) ── */
        if (++printTick >= 10u) {
            printTick = 0u;
            bool flying = supervisorIsFlying();
            float dz    = pos.z - lastZ;

            DEBUG_PRINT("--- Drone %u | %s | z=%.2fm",
                        (unsigned)myId,
                        flying ? "AIRBORNE" : "GROUNDED",
                        (double)pos.z);

            if (dz > 0.05f) {
                DEBUG_PRINT(" [GOING UP   +%.2fm]", (double)dz);
            } else if (dz < -0.05f) {
                DEBUG_PRINT(" [GOING DOWN %.2fm]", (double)dz);
            } else {
                DEBUG_PRINT(" [STABLE]");
            }
            DEBUG_PRINT("\n");
            lastZ = pos.z;
        }

        /* ── 2. Log own AI detection if active ── */
        if (currentDetection == DETECTION_HUMAN) {
            DEBUG_PRINT("I (drone %u) see a HUMAN  at (%.2f, %.2f, %.2f)\n",
                        (unsigned)myId,
                        (double)pos.x, (double)pos.y, (double)pos.z);
        } else if (currentDetection == DETECTION_OBJECT) {
            DEBUG_PRINT("I (drone %u) see an OBJECT at (%.2f, %.2f, %.2f)\n",
                        (unsigned)myId,
                        (double)pos.x, (double)pos.y, (double)pos.z);
        }

        /* ── 3. Build and broadcast SwarmPacket ── */
        /* Apply home offset so all drones share drone-0's global frame */
        float gx = pos.x + homeX;
        float gy = pos.y + homeY;

        SwarmPacket out;
        out.sourceId  = myId;
        out.x         = gx;
        out.y         = gy;
        out.z         = pos.z;
        out.detection = (uint8_t)currentDetection;
        out.seqNum    = seqNum++;

        memcpy(txPkt.data, &out, sizeof(SwarmPacket));
        radiolinkSendP2PPacketBroadcast(&txPkt);

        /* ── 4. Age-out stale peers; find closest active neighbour ── */
        uint32_t now         = T2M(xTaskGetTickCount());
        float    closestDist = 1.0e9f;
        float    avoidX      = 0.0f;
        float    avoidY      = 0.0f;

        xSemaphoreTake(peersMutex, portMAX_DELAY);

        for (unsigned i = 0u; i < MAX_DRONES; i++) {
            if (!peers[i].active) continue;

            /* remove peers that have gone silent */
            if ((now - peers[i].lastSeenMs) > DRONE_TIMEOUT_MS) {
                peers[i].active = false;
                DEBUG_PRINT("Drone %u timed out\n", i);
                continue;
            }

            /* Use global coordinates for distance (both sides have home offset) */
            float d = dist3(gx, gy, pos.z,
                            peers[i].x, peers[i].y, peers[i].z);
            if (d < closestDist) {
                closestDist = d;
                /* vector pointing AWAY from this neighbour (global frame) */
                avoidX = gx - peers[i].x;
                avoidY = gy - peers[i].y;
            }
        }

        xSemaphoreGive(peersMutex);

        /* ── 5. Collision avoidance ── */
        /*
         * If the nearest neighbour is closer than MIN_SEPARATION_M (3-D sphere
         * trigger), move AVOIDANCE_STEP_M horizontally away from it.
         * Z is kept constant — altitude changes cause instability.
         * The drone must already be flying under High-Level Commander control
         * (e.g. taken off with crtpCommanderHighLevelTakeoff) for this to
         * have any effect.
         */
        /* Only consider avoidance when this drone AND the peer are both
         * above CRUISE_ALT_M — avoids false triggers during takeoff when
         * all positions are near (0,0,0).                                  */
#define CRUISE_ALT_M 0.3f
        bool peerAlsoAirborne = false;
        if (closestDist < 1.0e8f) { /* at least one active peer */
            xSemaphoreTake(peersMutex, portMAX_DELAY);
            for (unsigned i = 0u; i < MAX_DRONES; i++) {
                if (peers[i].active && peers[i].z > CRUISE_ALT_M) {
                    peerAlsoAirborne = true;
                    break;
                }
            }
            xSemaphoreGive(peersMutex);
        }

        if (closestDist < MIN_SEPARATION_M && closestDist > 0.01f
                && pos.z > CRUISE_ALT_M && peerAlsoAirborne) {
            float mag = sqrtf(avoidX * avoidX + avoidY * avoidY);
            if (mag > 0.01f) {
                float nx = pos.x + (avoidX / mag) * AVOIDANCE_STEP_M;
                float ny = pos.y + (avoidY / mag) * AVOIDANCE_STEP_M;

                if (supervisorIsFlying()) {
                    /* Only move when already airborne */
                    /* Convert global avoidance target back to Kalman-local frame */
                    crtpCommanderHighLevelGoTo(nx - homeX, ny - homeY, pos.z,
                                               0.0f,
                                               AVOIDANCE_DURATION_S,
                                               false /* absolute coords */);
                    DEBUG_PRINT("AVOIDANCE ACTIVE: drone %u too close (%.2fm) -> moving to (%.2f, %.2f)\n",
                                (unsigned)myId, (double)closestDist,
                                (double)nx, (double)ny);
                } else {
                    DEBUG_PRINT("AVOIDANCE SUPPRESSED: drone %u too close (%.2fm) but GROUNDED — not moving\n",
                                (unsigned)myId, (double)closestDist);
                }
            }
        }

        vTaskDelay(M2T(BROADCAST_INTERVAL_MS));
    }
}

/* No runtime parameters needed — home offsets are compiled in based on
 * drone ID. See HOME_X_DRONE_1 / HOME_Y_DRONE_1 near the top of this file. */
