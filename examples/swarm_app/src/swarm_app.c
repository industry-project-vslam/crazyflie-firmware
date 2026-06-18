/**
 * swarm_app.c
 *
 * Leader/follower swarm application for Crazyflie 2.1.
 *
 * Default roles:
 *   ID 19, address E7E7E7E713: leader/scout with Flow + Multi-ranger
 *   ID 2,  address E7E7E7E702: follower/mapper with Flow + AI deck
 *
 * The leader reads the Multi-ranger deck, publishes its position and safety
 * state over P2P, and scouts in small steps. The follower does not assume a
 * shared XY coordinate frame: it holds its own XY position and mirrors the
 * leader's altitude/hold/land state after it is manually put in the air.
 *
 * The previous peer-to-peer distance avoidance code is still present near the
 * bottom of the control loop, but it is disabled by SWARM_ENABLE_PEER_AVOIDANCE.
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
#include "range.h"

#define DEBUG_MODULE "SWARM"
#include "debug.h"

#define MAX_DRONES                      32u
#define P2P_PORT_SWARM                  0x02
#define BROADCAST_INTERVAL_MS           100u
#define DRONE_TIMEOUT_MS                1000u

#define SWARM_LEADER_ID                 1u
#define SWARM_FOLLOWER_ID               11u
#define SWARM_ENABLE_LEADER_FOLLOW      0
#define SWARM_ENABLE_PEER_AVOIDANCE     0
#define SWARM_ENABLE_SCOUT_EXPLORATION  0
#define SWARM_ENABLE_LEADER_SELF_COMMANDS 0

#define FOLLOWER_OFFSET_X_M             -0.50f
#define FOLLOWER_OFFSET_Y_M             0.00f
#define FOLLOWER_COMMAND_INTERVAL_MS    300u
#define FOLLOWER_GOTO_DURATION_S        0.45f
#define FOLLOWER_AUTO_TAKEOFF           0
#define FOLLOWER_XY_FORMATION_FOLLOW    0
#define FOLLOWER_READY_DISTANCE_M       0.35f
#define FOLLOWER_READY_ALT_M            0.20f
#define FOLLOWER_LOST_HOLD_MS           1500u
#define LEADER_YAW_RAD                  0.0f
#define FOLLOWER_YAW_RAD                3.1415926f

#define CRUISE_ALT_M                    0.30f
#define LEADER_SAFETY_ACTIVE_ALT_M      0.35f
#define LEADER_FRONT_STOP_M             0.40f
#define LEADER_UP_STOP_M                0.30f
#define LEADER_SIDE_MIN_M               0.25f
#define LEADER_CORRIDOR_MIN_M           0.80f
#define LEADER_STOP_INTERVAL_MS         300u

#define SCOUT_START_DELAY_MS            3000u
#define SCOUT_COMMAND_INTERVAL_MS       1800u
#define SCOUT_STEP_M                    0.18f
#define SCOUT_STEP_DURATION_S           1.20f
#define SCOUT_FRONT_CLEAR_M             0.55f
#define SCOUT_SIDE_CLEAR_M              0.40f
#define SCOUT_BACK_CLEAR_M              0.35f
#define SCOUT_REACHED_DISTANCE_M        0.12f
#define SCOUT_TARGET_TIMEOUT_MS         4000u
#define VISITED_MAX_CELLS               80u
#define VISITED_CELL_M                  0.25f

#define MIN_SEPARATION_M                0.167f
#define AVOIDANCE_STEP_M                0.30f
#define AVOIDANCE_DURATION_S            0.15f

typedef enum {
    ROLE_GENERIC  = 0,
    ROLE_LEADER   = 1,
    ROLE_FOLLOWER = 2,
} SwarmRole;

typedef enum {
    DETECTION_NONE   = 0,
    DETECTION_HUMAN  = 1,
    DETECTION_OBJECT = 2,
} DetectionType;

typedef enum {
    CMD_FOLLOW = 0,
    CMD_HOLD   = 1,
} SwarmCommand;

typedef enum {
    FLAG_FRONT_BLOCKED     = 1u << 0,
    FLAG_UP_BLOCKED        = 1u << 1,
    FLAG_CORRIDOR_TOO_TIGHT= 1u << 2,
    FLAG_SIDE_TOO_CLOSE    = 1u << 3,
    FLAG_LEADER_AIRBORNE   = 1u << 4,
} SwarmFlags;

typedef struct __attribute__((packed)) {
    uint8_t sourceId;
    uint8_t role;
    uint8_t command;
    uint8_t flags;
    float   x;
    float   y;
    float   z;
    float   targetX;
    float   targetY;
    float   targetZ;
    uint8_t detection;
    uint8_t seqNum;
} SwarmPacket;

_Static_assert(sizeof(SwarmPacket) <= P2P_MAX_DATA_SIZE,
               "SwarmPacket exceeds P2P_MAX_DATA_SIZE");

typedef struct {
    bool     active;
    float    x, y, z;
    float    targetX, targetY, targetZ;
    uint8_t  role;
    uint8_t  command;
    uint8_t  flags;
    uint8_t  detection;
    uint8_t  lastSeq;
    uint32_t lastSeenMs;
} PeerState;

static PeerState peers[MAX_DRONES];
static SemaphoreHandle_t peersMutex;
static uint8_t myId;
static SwarmRole myRole = ROLE_GENERIC;

static const float kHomeX[MAX_DRONES] = {
    0.0f,
    0.0f, -0.5f, 1.0f, 1.5f,
    2.0f, 2.5f, 3.0f, 3.5f,
    4.0f, 0.0f, 5.0f, 5.5f,
    6.0f, 6.5f, 7.0f, 7.5f,
    8.0f, 8.5f, 0.0f, 9.5f,
    10.0f, 10.5f, 11.0f, 11.5f,
    12.0f, 12.5f, 13.0f, 13.5f,
    14.0f, 14.5f, 15.0f,
};

static const float kHomeY[MAX_DRONES] = {
    0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f,
};

static float homeX = 0.0f;
static float homeY = 0.0f;
static volatile DetectionType currentDetection = DETECTION_NONE;

static inline float dist3(float ax, float ay, float az,
                          float bx, float by, float bz)
{
    float dx = ax - bx;
    float dy = ay - by;
    float dz = az - bz;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static inline bool rangeIsValid(float r)
{
    return isfinite(r) && r > 0.01f && r < 5.0f;
}

static float rangeGetM(rangeDirection_t direction)
{
    float mm = rangeGet(direction);
    if (mm >= 32760.0f) {
        return 99.0f;
    }
    return mm * 0.001f;
}

static const char *roleName(SwarmRole role)
{
    switch (role) {
        case ROLE_LEADER: return "LEADER";
        case ROLE_FOLLOWER: return "FOLLOWER";
        default: return "GENERIC";
    }
}

static __attribute__((unused)) float desiredYawForRole(SwarmRole role)
{
    return (role == ROLE_FOLLOWER) ? FOLLOWER_YAW_RAD : LEADER_YAW_RAD;
}

static __attribute__((unused)) float yawForHeading(uint8_t heading)
{
    static const float yaws[4] = {
        0.0f,
        1.5707963f,
        3.1415926f,
        -1.5707963f,
    };
    return yaws[heading & 3u];
}

static __attribute__((unused)) void stepForHeading(uint8_t heading, float *dx, float *dy)
{
    switch (heading & 3u) {
        case 0u: *dx = SCOUT_STEP_M;  *dy = 0.0f; break;
        case 1u: *dx = 0.0f;         *dy = SCOUT_STEP_M; break;
        case 2u: *dx = -SCOUT_STEP_M; *dy = 0.0f; break;
        default: *dx = 0.0f;         *dy = -SCOUT_STEP_M; break;
    }
}

static int16_t visitedCellCoord(float value)
{
    float scaled = value / VISITED_CELL_M;
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static __attribute__((unused)) bool visitedHasCell(const int16_t *visitedX, const int16_t *visitedY,
                           uint8_t visitedCount, float x, float y)
{
    int16_t cx = visitedCellCoord(x);
    int16_t cy = visitedCellCoord(y);

    for (uint8_t i = 0u; i < visitedCount; i++) {
        if (visitedX[i] == cx && visitedY[i] == cy) {
            return true;
        }
    }

    return false;
}

static __attribute__((unused)) void visitedMarkCell(int16_t *visitedX, int16_t *visitedY,
                            uint8_t *visitedCount, float x, float y)
{
    if (visitedHasCell(visitedX, visitedY, *visitedCount, x, y)) {
        return;
    }

    if (*visitedCount >= VISITED_MAX_CELLS) {
        memmove(&visitedX[0], &visitedX[1], (VISITED_MAX_CELLS - 1u) * sizeof(visitedX[0]));
        memmove(&visitedY[0], &visitedY[1], (VISITED_MAX_CELLS - 1u) * sizeof(visitedY[0]));
        *visitedCount = VISITED_MAX_CELLS - 1u;
    }

    visitedX[*visitedCount] = visitedCellCoord(x);
    visitedY[*visitedCount] = visitedCellCoord(y);
    *visitedCount = *visitedCount + 1u;
}

static __attribute__((unused)) bool scoutDirectionClear(uint8_t currentHeading, uint8_t candidateHeading,
                                float front, float left, float right, float back)
{
    uint8_t rel = (candidateHeading - currentHeading) & 3u;

    switch (rel) {
        case 0u: return isfinite(front) && front > SCOUT_FRONT_CLEAR_M;
        case 1u: return isfinite(left) && left > SCOUT_SIDE_CLEAR_M;
        case 2u: return isfinite(back) && back > SCOUT_BACK_CLEAR_M;
        default: return isfinite(right) && right > SCOUT_SIDE_CLEAR_M;
    }
}

static uint8_t leaderSafetyFlags(void)
{
    uint8_t flags = 0u;
    float front = rangeGetM(rangeFront);
    float up = rangeGetM(rangeUp);
    float left = rangeGetM(rangeLeft);
    float right = rangeGetM(rangeRight);

    if (rangeIsValid(front) && front < LEADER_FRONT_STOP_M) {
        flags |= FLAG_FRONT_BLOCKED;
    }

    if (rangeIsValid(up) && up < LEADER_UP_STOP_M) {
        flags |= FLAG_UP_BLOCKED;
    }

    if ((rangeIsValid(left) && left < LEADER_SIDE_MIN_M) ||
        (rangeIsValid(right) && right < LEADER_SIDE_MIN_M)) {
        flags |= FLAG_SIDE_TOO_CLOSE;
    }

    if (rangeIsValid(left) && rangeIsValid(right) &&
        (left + right) < LEADER_CORRIDOR_MIN_M) {
        flags |= FLAG_CORRIDOR_TOO_TIGHT;
    }

    if (supervisorIsFlying()) {
        flags |= FLAG_LEADER_AIRBORNE;
    }

    return flags;
}

static void aiDeckHandler(const CPXPacket_t *cpxPkt)
{
    if (cpxPkt->route.function != CPX_F_APP) return;
    if (cpxPkt->dataLength < 1) return;

    switch (cpxPkt->data[0]) {
        case 1: currentDetection = DETECTION_HUMAN; break;
        case 2: currentDetection = DETECTION_OBJECT; break;
        default: currentDetection = DETECTION_NONE; break;
    }
}

static void p2pCallback(P2PPacket *p)
{
    if (p->port != P2P_PORT_SWARM) return;
    if (p->size != sizeof(SwarmPacket)) return;

    SwarmPacket pkt;
    memcpy(&pkt, p->data, sizeof(SwarmPacket));

    uint8_t id = pkt.sourceId;
    if (id >= MAX_DRONES || id == myId) return;

    if (xSemaphoreTake(peersMutex, 0) != pdTRUE) return;

    PeerState *peer = &peers[id];
    if (peer->active && pkt.seqNum == peer->lastSeq) {
        xSemaphoreGive(peersMutex);
        return;
    }

    peer->active = true;
    peer->x = pkt.x;
    peer->y = pkt.y;
    peer->z = pkt.z;
    peer->targetX = pkt.targetX;
    peer->targetY = pkt.targetY;
    peer->targetZ = pkt.targetZ;
    peer->role = pkt.role;
    peer->command = pkt.command;
    peer->flags = pkt.flags;
    peer->detection = pkt.detection;
    peer->lastSeq = pkt.seqNum;
    peer->lastSeenMs = T2M(xTaskGetTickCount());

    xSemaphoreGive(peersMutex);

    if (pkt.role == ROLE_LEADER) {
        DEBUG_PRINT("P2P leader %u cmd=%u flags=0x%02x pos=(%.2f,%.2f,%.2f)\n",
                    (unsigned)id, (unsigned)pkt.command, (unsigned)pkt.flags,
                    (double)pkt.x, (double)pkt.y, (double)pkt.z);
    }

    if (pkt.detection == DETECTION_HUMAN) {
        DEBUG_PRINT("Drone %u sees a HUMAN at (%.2f, %.2f, %.2f)\n",
                    (unsigned)id, (double)pkt.x, (double)pkt.y, (double)pkt.z);
    } else if (pkt.detection == DETECTION_OBJECT) {
        DEBUG_PRINT("Drone %u sees an OBJECT at (%.2f, %.2f, %.2f)\n",
                    (unsigned)id, (double)pkt.x, (double)pkt.y, (double)pkt.z);
    }
}

static __attribute__((unused)) bool getLeaderPeer(PeerState *out)
{
    bool found = false;
    uint32_t now = T2M(xTaskGetTickCount());

    xSemaphoreTake(peersMutex, portMAX_DELAY);
    PeerState *leader = &peers[SWARM_LEADER_ID];
    if (leader->active && (now - leader->lastSeenMs) <= DRONE_TIMEOUT_MS) {
        *out = *leader;
        found = true;
    }
    xSemaphoreGive(peersMutex);

    return found;
}

static __attribute__((unused)) bool getPeerById(uint8_t id, PeerState *out)
{
    bool found = false;
    uint32_t now = T2M(xTaskGetTickCount());

    if (id >= MAX_DRONES) {
        return false;
    }

    xSemaphoreTake(peersMutex, portMAX_DELAY);
    PeerState *peer = &peers[id];
    if (peer->active && (now - peer->lastSeenMs) <= DRONE_TIMEOUT_MS) {
        *out = *peer;
        found = true;
    }
    xSemaphoreGive(peersMutex);

    return found;
}

static __attribute__((unused)) bool followerReadyForLeaderStep(float leaderTargetGx,
                                                               float leaderTargetGy,
                                                               float leaderTargetZ,
                                                               uint32_t *ageMs)
{
    PeerState follower;
    uint32_t now = T2M(xTaskGetTickCount());

    if (!getPeerById(SWARM_FOLLOWER_ID, &follower)) {
        if (ageMs != NULL) {
            *ageMs = DRONE_TIMEOUT_MS + 1u;
        }
        return false;
    }

    if (ageMs != NULL) {
        *ageMs = now - follower.lastSeenMs;
    }

    float dz = follower.z - leaderTargetZ;

#if FOLLOWER_XY_FORMATION_FOLLOW
    float expectedX = leaderTargetGx + FOLLOWER_OFFSET_X_M;
    float expectedY = leaderTargetGy + FOLLOWER_OFFSET_Y_M;
    float dxy = sqrtf((follower.x - expectedX) * (follower.x - expectedX) +
                      (follower.y - expectedY) * (follower.y - expectedY));

    return (follower.z > FOLLOWER_READY_ALT_M) &&
           (fabsf(dz) < FOLLOWER_READY_DISTANCE_M) &&
           (dxy < FOLLOWER_READY_DISTANCE_M);
#else
    (void)leaderTargetGx;
    (void)leaderTargetGy;
    return (follower.z > FOLLOWER_READY_ALT_M) &&
           (fabsf(dz) < FOLLOWER_READY_DISTANCE_M);
#endif
}

void appMain(void)
{
    uint64_t addr = configblockGetRadioAddress();
    myId = (uint8_t)(addr & 0xFFu);

    if (myId < MAX_DRONES) {
        homeX = kHomeX[myId];
        homeY = kHomeY[myId];
    }

    if (myId == SWARM_LEADER_ID) {
        myRole = ROLE_LEADER;
    } else if (myId == SWARM_FOLLOWER_ID) {
        myRole = ROLE_FOLLOWER;
    }

    peersMutex = xSemaphoreCreateMutex();
    memset(peers, 0, sizeof(peers));

    DEBUG_PRINT("Swarm app starting | id=%u | role=%s | home=(%.2f,%.2f)\n",
                (unsigned)myId, roleName(myRole),
                (double)homeX, (double)homeY);
    DEBUG_PRINT("Leader=%u follower=%u offset=(%.2f,%.2f) peer_avoidance=%u\n",
                (unsigned)SWARM_LEADER_ID, (unsigned)SWARM_FOLLOWER_ID,
                (double)FOLLOWER_OFFSET_X_M, (double)FOLLOWER_OFFSET_Y_M,
                (unsigned)SWARM_ENABLE_PEER_AVOIDANCE);
    DEBUG_PRINT("Mode: PC_CONTROL_PASSIVE leader_follow=%u scout=%u self_cmd=%u\n",
                (unsigned)SWARM_ENABLE_LEADER_FOLLOW,
                (unsigned)SWARM_ENABLE_SCOUT_EXPLORATION,
                (unsigned)SWARM_ENABLE_LEADER_SELF_COMMANDS);
    DEBUG_PRINT("Firmware motion: disabled; PC sends all flight primitives\n");

#if SWARM_ENABLE_SCOUT_EXPLORATION || SWARM_ENABLE_LEADER_SELF_COMMANDS || SWARM_ENABLE_LEADER_FOLLOW || SWARM_ENABLE_PEER_AVOIDANCE
    paramSetInt(paramGetVarId("commander", "enHighLevel"), 1);
#endif
    cpxRegisterAppMessageHandler(aiDeckHandler);
    p2pRegisterCB(p2pCallback);

    vTaskDelay(M2T(3000));

    static P2PPacket txPkt;
    txPkt.port = P2P_PORT_SWARM;
    txPkt.size = (uint8_t)sizeof(SwarmPacket);

    uint8_t seqNum = 0u;
    uint32_t printTick = 0u;
    uint32_t lastLeaderStopMs = 0u;
    uint32_t lastFollowerCmdMs = 0u;
    uint32_t scoutAirborneSinceMs = 0u;
    uint32_t lastScoutCmdMs = 0u;
    uint8_t scoutHeading = 0u;
    uint8_t visitedCount = 0u;
    int16_t visitedX[VISITED_MAX_CELLS];
    int16_t visitedY[VISITED_MAX_CELLS];
    bool leaderTargetActive = false;
    float leaderTargetGx = 0.0f;
    float leaderTargetGy = 0.0f;
    float leaderTargetZ = CRUISE_ALT_M;
    uint32_t leaderTargetStartedMs = 0u;
    float lastZ = 0.0f;

#if !SWARM_ENABLE_LEADER_SELF_COMMANDS
    (void)lastLeaderStopMs;
#endif
#if !SWARM_ENABLE_LEADER_FOLLOW
    (void)lastFollowerCmdMs;
#endif
#if !SWARM_ENABLE_SCOUT_EXPLORATION
    (void)scoutAirborneSinceMs;
    (void)lastScoutCmdMs;
    (void)scoutHeading;
    (void)visitedCount;
    (void)visitedX;
    (void)visitedY;
    (void)leaderTargetActive;
    (void)leaderTargetGx;
    (void)leaderTargetGy;
    (void)leaderTargetZ;
    (void)leaderTargetStartedMs;
#endif

    for (;;) {
        point_t pos;
        estimatorKalmanGetEstimatedPos(&pos);

        float gx = pos.x + homeX;
        float gy = pos.y + homeY;
        uint32_t now = T2M(xTaskGetTickCount());

        uint8_t flags = 0u;
        uint8_t command = CMD_FOLLOW;
        float targetGx = gx;
        float targetGy = gy;
        float targetZ = pos.z;

        if (myRole == ROLE_LEADER) {
            flags = leaderSafetyFlags();

#if SWARM_ENABLE_SCOUT_EXPLORATION
            if (!supervisorIsFlying() || pos.z <= LEADER_SAFETY_ACTIVE_ALT_M) {
                scoutAirborneSinceMs = 0u;
                leaderTargetActive = false;
            } else if (scoutAirborneSinceMs == 0u) {
                scoutAirborneSinceMs = now;
                lastScoutCmdMs = now;
                leaderTargetActive = false;
                visitedMarkCell(visitedX, visitedY, &visitedCount, gx, gy);
                DEBUG_PRINT("SCOUT: airborne, waiting before exploration\n");
            }
#endif

            if (pos.z > LEADER_SAFETY_ACTIVE_ALT_M &&
                (flags & (FLAG_FRONT_BLOCKED | FLAG_UP_BLOCKED |
                          FLAG_CORRIDOR_TOO_TIGHT | FLAG_SIDE_TOO_CLOSE))) {
                command = CMD_HOLD;
            }

#if SWARM_ENABLE_SCOUT_EXPLORATION
            if (leaderTargetActive) {
                targetGx = leaderTargetGx;
                targetGy = leaderTargetGy;
                targetZ = leaderTargetZ;

                float dTarget = sqrtf((gx - leaderTargetGx) * (gx - leaderTargetGx) +
                                      (gy - leaderTargetGy) * (gy - leaderTargetGy));
                uint32_t followerAgeMs = 0u;
                bool followerReady = followerReadyForLeaderStep(leaderTargetGx,
                                                                leaderTargetGy,
                                                                leaderTargetZ,
                                                                &followerAgeMs);
                bool targetTimedOut = (now - leaderTargetStartedMs) >= SCOUT_TARGET_TIMEOUT_MS;

                if ((dTarget < SCOUT_REACHED_DISTANCE_M || targetTimedOut) && followerReady) {
                    leaderTargetActive = false;
                    lastScoutCmdMs = now;
                    DEBUG_PRINT("SCOUT: step complete d=%.2f follower_age=%ums\n",
                                (double)dTarget, (unsigned)followerAgeMs);
                } else if (followerAgeMs > FOLLOWER_LOST_HOLD_MS) {
                    command = CMD_HOLD;
                    lastScoutCmdMs = now;
                    DEBUG_PRINT("SCOUT: waiting, follower lost/stale age=%ums\n",
                                (unsigned)followerAgeMs);
                } else {
                    command = CMD_FOLLOW;
                }
            }

            if (!leaderTargetActive &&
                command != CMD_HOLD &&
                supervisorIsFlying() &&
                pos.z > LEADER_SAFETY_ACTIVE_ALT_M &&
                scoutAirborneSinceMs != 0u &&
                (now - scoutAirborneSinceMs) >= SCOUT_START_DELAY_MS &&
                (now - lastScoutCmdMs) >= SCOUT_COMMAND_INTERVAL_MS) {
                uint32_t followerAgeMs = 0u;
                bool followerReady = followerReadyForLeaderStep(gx, gy, pos.z, &followerAgeMs);

                if (!followerReady) {
                    command = CMD_HOLD;
                    lastScoutCmdMs = now;
                    DEBUG_PRINT("SCOUT: waiting for follower formation age=%ums\n",
                                (unsigned)followerAgeMs);
                } else {
                    float front = rangeGetM(rangeFront);
                    float left = rangeGetM(rangeLeft);
                    float right = rangeGetM(rangeRight);
                    float back = rangeGetM(rangeBack);

                    uint8_t candidates[4] = {
                        scoutHeading,
                        (uint8_t)((scoutHeading + 1u) & 3u),
                        (uint8_t)((scoutHeading + 3u) & 3u),
                        (uint8_t)((scoutHeading + 2u) & 3u),
                    };

                    int8_t chosen = -1;
                    int8_t fallback = -1;
                    for (uint8_t i = 0u; i < 4u; i++) {
                        uint8_t candidate = candidates[i];
                        if (!scoutDirectionClear(scoutHeading, candidate, front, left, right, back)) {
                            continue;
                        }

                        float dx = 0.0f;
                        float dy = 0.0f;
                        stepForHeading(candidate, &dx, &dy);

                        if (fallback < 0) {
                            fallback = (int8_t)candidate;
                        }

                        if (!visitedHasCell(visitedX, visitedY, visitedCount, gx + dx, gy + dy)) {
                            chosen = (int8_t)candidate;
                            break;
                        }
                    }

                    if (chosen < 0) {
                        chosen = fallback;
                    }

                    if (chosen >= 0) {
                        float dx = 0.0f;
                        float dy = 0.0f;
                        scoutHeading = (uint8_t)chosen;
                        stepForHeading(scoutHeading, &dx, &dy);

                        leaderTargetGx = gx + dx;
                        leaderTargetGy = gy + dy;
                        leaderTargetZ = pos.z;
                        leaderTargetActive = true;
                        leaderTargetStartedMs = now;
                        targetGx = leaderTargetGx;
                        targetGy = leaderTargetGy;
                        targetZ = leaderTargetZ;

                        visitedMarkCell(visitedX, visitedY, &visitedCount,
                                        leaderTargetGx, leaderTargetGy);

                        int rc = crtpCommanderHighLevelGoTo(pos.x + dx, pos.y + dy, pos.z,
                                                            yawForHeading(scoutHeading),
                                                            SCOUT_STEP_DURATION_S,
                                                            false);
                        DEBUG_PRINT("SCOUT: target dir=%u step=(%.2f,%.2f) rc=%d range f/l/r/b=%.2f/%.2f/%.2f/%.2f visited=%u\n",
                                    (unsigned)scoutHeading,
                                    (double)dx, (double)dy, rc,
                                    (double)front, (double)left,
                                    (double)right, (double)back,
                                    (unsigned)visitedCount);
                    } else {
                        command = CMD_HOLD;
                        DEBUG_PRINT("SCOUT: no safe direction, holding range f/l/r/b=%.2f/%.2f/%.2f/%.2f\n",
                                    (double)front, (double)left,
                                    (double)right, (double)back);
                    }
                }
            }
#else
            if (pos.z > LEADER_SAFETY_ACTIVE_ALT_M &&
                (flags & FLAG_FRONT_BLOCKED)) {
                command = CMD_HOLD;
            }
#endif

#if SWARM_ENABLE_LEADER_SELF_COMMANDS
            if (command == CMD_HOLD && supervisorIsFlying() &&
                pos.z > LEADER_SAFETY_ACTIVE_ALT_M &&
                (now - lastLeaderStopMs) >= LEADER_STOP_INTERVAL_MS) {
                crtpCommanderHighLevelGoTo(pos.x, pos.y, pos.z,
                                           desiredYawForRole(myRole),
                                           0.30f, false);
                lastLeaderStopMs = now;
                DEBUG_PRINT("LEADER HOLD: flags=0x%02x front=%.2f left=%.2f right=%.2f up=%.2f\n",
                            (unsigned)flags,
                            (double)rangeGetM(rangeFront),
                            (double)rangeGetM(rangeLeft),
                            (double)rangeGetM(rangeRight),
                            (double)rangeGetM(rangeUp));
            }
#endif
        }

        SwarmPacket out;
        out.sourceId = myId;
        out.role = (uint8_t)myRole;
        out.command = command;
        out.flags = flags;
        out.x = gx;
        out.y = gy;
        out.z = pos.z;
        out.targetX = targetGx;
        out.targetY = targetGy;
        out.targetZ = targetZ;
        out.detection = (uint8_t)currentDetection;
        out.seqNum = seqNum++;

        memcpy(txPkt.data, &out, sizeof(SwarmPacket));
        radiolinkSendP2PPacketBroadcast(&txPkt);

#if SWARM_ENABLE_LEADER_FOLLOW
        if (myRole == ROLE_FOLLOWER) {
            PeerState leader;
            bool leaderOk = getLeaderPeer(&leader);

            if (!leaderOk) {
                if (supervisorIsFlying() &&
                    (now - lastFollowerCmdMs) >= FOLLOWER_COMMAND_INTERVAL_MS) {
                    crtpCommanderHighLevelGoTo(pos.x, pos.y, pos.z,
                                               desiredYawForRole(myRole),
                                               FOLLOWER_GOTO_DURATION_S, false);
                    lastFollowerCmdMs = now;
                }
            } else if ((now - lastFollowerCmdMs) >= FOLLOWER_COMMAND_INTERVAL_MS) {
                bool leaderHold = leader.command == CMD_HOLD;
                bool leaderAirborne = (leader.flags & FLAG_LEADER_AIRBORNE) != 0u;

                if (!leaderAirborne && supervisorIsFlying()) {
                    crtpCommanderHighLevelLandYaw(0.0f, 2.0f,
                                                  desiredYawForRole(myRole));
                    DEBUG_PRINT("FOLLOWER: leader landed, landing\n");
                } else if (leaderAirborne && !supervisorIsFlying() && leader.z > CRUISE_ALT_M) {
#if FOLLOWER_AUTO_TAKEOFF
                    if (!supervisorIsArmed()) {
                        bool armed = supervisorRequestArming(true);
                        DEBUG_PRINT("FOLLOWER: arm request %s canArm=%u locked=%u canFly=%u\n",
                                    armed ? "OK" : "FAILED",
                                    (unsigned)supervisorCanArm(),
                                    (unsigned)supervisorIsLocked(),
                                    (unsigned)supervisorCanFly());
                    }

                    if (supervisorIsArmed() || supervisorCanFly()) {
                        int rc = crtpCommanderHighLevelTakeoffYaw(leader.z, 2.0f,
                                                                  desiredYawForRole(myRole));
                        DEBUG_PRINT("FOLLOWER: leader airborne, takeoff to %.2f rc=%d\n",
                                    (double)leader.z, rc);
                    } else {
                        DEBUG_PRINT("FOLLOWER: takeoff blocked armed=%u canArm=%u locked=%u canFly=%u\n",
                                    (unsigned)supervisorIsArmed(),
                                    (unsigned)supervisorCanArm(),
                                    (unsigned)supervisorIsLocked(),
                                    (unsigned)supervisorCanFly());
                    }
#else
                    DEBUG_PRINT("FOLLOWER: leader airborne, waiting for manual takeoff\n");
#endif
                } else if (supervisorIsFlying()) {
#if FOLLOWER_XY_FORMATION_FOLLOW
                    float targetGx = leader.targetX + FOLLOWER_OFFSET_X_M;
                    float targetGy = leader.targetY + FOLLOWER_OFFSET_Y_M;
                    float targetLocalX = targetGx - homeX;
                    float targetLocalY = targetGy - homeY;
                    float targetZ = fmaxf(CRUISE_ALT_M, leader.targetZ);

                    if (leaderHold) {
                        targetLocalX = pos.x;
                        targetLocalY = pos.y;
                        targetZ = pos.z;
                    }

                    crtpCommanderHighLevelGoTo(targetLocalX, targetLocalY,
                                               targetZ, desiredYawForRole(myRole),
                                               FOLLOWER_GOTO_DURATION_S, false);
                    DEBUG_PRINT("FOLLOWER: leader=%u %s target=(%.2f,%.2f,%.2f)\n",
                                (unsigned)SWARM_LEADER_ID,
                                leaderHold ? "HOLD" : "FOLLOW",
                                (double)targetLocalX,
                                (double)targetLocalY,
                                (double)targetZ);
#else
                    float targetZ = leaderHold ? pos.z : fmaxf(CRUISE_ALT_M, leader.z);
                    crtpCommanderHighLevelGoTo(pos.x, pos.y,
                                               targetZ, desiredYawForRole(myRole),
                                               FOLLOWER_GOTO_DURATION_S, false);
                    DEBUG_PRINT("FOLLOWER: leader=%u %s altitude_mirror z=%.2f local_hold=(%.2f,%.2f)\n",
                                (unsigned)SWARM_LEADER_ID,
                                leaderHold ? "HOLD" : "ALT",
                                (double)targetZ,
                                (double)pos.x,
                                (double)pos.y);
#endif
                }

                lastFollowerCmdMs = now;
            }
        }
#endif

        uint32_t closestPeerId = 0u;
        float closestDist = 1.0e9f;
        float avoidX = 0.0f;
        float avoidY = 0.0f;

        xSemaphoreTake(peersMutex, portMAX_DELAY);
        for (unsigned i = 0u; i < MAX_DRONES; i++) {
            if (!peers[i].active) continue;

            if ((now - peers[i].lastSeenMs) > DRONE_TIMEOUT_MS) {
                peers[i].active = false;
                DEBUG_PRINT("Drone %u timed out\n", i);
                continue;
            }

            float d = dist3(gx, gy, pos.z, peers[i].x, peers[i].y, peers[i].z);
            if (d < closestDist) {
                closestDist = d;
                closestPeerId = i;
                avoidX = gx - peers[i].x;
                avoidY = gy - peers[i].y;
            }
        }
        xSemaphoreGive(peersMutex);

#if SWARM_ENABLE_PEER_AVOIDANCE
        bool peerAlsoAirborne = false;
        if (closestDist < 1.0e8f) {
            xSemaphoreTake(peersMutex, portMAX_DELAY);
            for (unsigned i = 0u; i < MAX_DRONES; i++) {
                if (peers[i].active && peers[i].z > CRUISE_ALT_M) {
                    peerAlsoAirborne = true;
                    break;
                }
            }
            xSemaphoreGive(peersMutex);
        }

        if (closestDist < MIN_SEPARATION_M && closestDist > 0.01f &&
            pos.z > CRUISE_ALT_M && peerAlsoAirborne) {
            float mag = sqrtf(avoidX * avoidX + avoidY * avoidY);
            if (mag > 0.01f && supervisorIsFlying()) {
                float nx = pos.x + (avoidX / mag) * AVOIDANCE_STEP_M;
                float ny = pos.y + (avoidY / mag) * AVOIDANCE_STEP_M;
                crtpCommanderHighLevelGoTo(nx - homeX, ny - homeY, pos.z,
                                           desiredYawForRole(myRole),
                                           AVOIDANCE_DURATION_S, false);
                DEBUG_PRINT("PEER AVOIDANCE: closest=%u dist=%.2f target=(%.2f,%.2f)\n",
                            (unsigned)closestPeerId, (double)closestDist,
                            (double)nx, (double)ny);
            }
        }
#else
        (void)closestPeerId;
        (void)closestDist;
        (void)avoidX;
        (void)avoidY;
#endif

        if (++printTick >= 10u) {
            printTick = 0u;
            bool flying = supervisorIsFlying();
            float dz = pos.z - lastZ;

            DEBUG_PRINT("--- Drone %u | %s | %s | z=%.2fm",
                        (unsigned)myId, roleName(myRole),
                        flying ? "AIRBORNE" : "GROUNDED",
                        (double)pos.z);

            if (dz > 0.05f) {
                DEBUG_PRINT(" [GOING UP +%.2fm]", (double)dz);
            } else if (dz < -0.05f) {
                DEBUG_PRINT(" [GOING DOWN %.2fm]", (double)dz);
            } else {
                DEBUG_PRINT(" [STABLE]");
            }

            if (myRole == ROLE_LEADER) {
                DEBUG_PRINT(" range f/l/r/u=%.2f/%.2f/%.2f/%.2f",
                            (double)rangeGetM(rangeFront),
                            (double)rangeGetM(rangeLeft),
                            (double)rangeGetM(rangeRight),
                            (double)rangeGetM(rangeUp));
            }

            DEBUG_PRINT("\n");
            lastZ = pos.z;
        }

        if (currentDetection == DETECTION_HUMAN) {
            DEBUG_PRINT("I drone %u see HUMAN at (%.2f, %.2f, %.2f)\n",
                        (unsigned)myId, (double)gx, (double)gy, (double)pos.z);
        } else if (currentDetection == DETECTION_OBJECT) {
            DEBUG_PRINT("I drone %u see OBJECT at (%.2f, %.2f, %.2f)\n",
                        (unsigned)myId, (double)gx, (double)gy, (double)pos.z);
        }

        vTaskDelay(M2T(BROADCAST_INTERVAL_MS));
    }
}
