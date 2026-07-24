/* v1 旧版 — 保留供参考。函数签名已更新为使用 RobotState *rs。
 * 主循环使用 task_v2.c。 */

#include "task.h"
#include "robot.h"

extern volatile unsigned long tick_ms;

/* 从 empty.c 拆出的任务内部状态。 */
typedef enum
{
    AVOID_IDLE = 0,
    AVOID_STOP,
    AVOID_TURN_LEFT,
    AVOID_FORWARD,
    AVOID_TURN_RIGHT,
    AVOID_TURN_LINE
} AvoidStage;

typedef enum
{
    TURN_IDLE = 0,
    TURN_STOP,
    TURN_LEFT,
    TURN_FORWARD,
    TURN_RIGHT
} TurnStage;

static AvoidStage g_avoidStage = AVOID_IDLE;
static unsigned long g_avoidStartMs = 0;
static TurnStage g_turnStage = TURN_IDLE;
int count = 0;

void ObstacleAvoidance_Task(RobotState *rs, unsigned long nowMs)
{
    switch (g_avoidStage)
    {
    case AVOID_IDLE:
        if (rs->dist > 0 && rs->dist < 25) {
            g_avoidStage = AVOID_STOP;
            g_avoidStartMs = nowMs;
            Set_PWM(0, 0);
        } else {
            PID_control(rs);
        }
        break;
    case AVOID_STOP:
        if (nowMs - g_avoidStartMs > 500) {
            g_avoidStage = AVOID_TURN_RIGHT;
            g_avoidStartMs = nowMs;
            Set_PWM(30, -30);
        }
        break;
    case AVOID_TURN_RIGHT:
        if (nowMs - g_avoidStartMs > 300) {
            g_avoidStage = AVOID_FORWARD;
            g_avoidStartMs = nowMs;
            Set_PWM(30, 30);
        }
        break;
    case AVOID_FORWARD:
        if (nowMs - g_avoidStartMs > 1000) {
            g_avoidStage = AVOID_TURN_LEFT;
            g_avoidStartMs = nowMs;
            Set_PWM(-30, 30);
        }
        break;
    case AVOID_TURN_LEFT:
        if (nowMs - g_avoidStartMs > 300) {
            g_avoidStage = AVOID_TURN_LINE;
            g_avoidStartMs = nowMs;
            Set_PWM(30, 45);
        }
        break;
    case AVOID_TURN_LINE:
        if (rs->isLost == 0) {
            g_avoidStage = AVOID_IDLE;
            g_avoidStartMs = nowMs;
        }
        break;
    }
}

void turn_Task(RobotState *rs)
{
    int turn_angle = 45;
    switch (g_turnStage)
    {
    case TURN_IDLE:
        if (rs->isLost != 0) {
            g_turnStage = TURN_STOP;
            Set_PWM(0, 0);
        } else {
            PID_control(rs);
        }
        break;
    case TURN_STOP:
        rs->originYaw = rs->yaw;
        g_turnStage = (rs->isLost == 1) ? TURN_LEFT : TURN_RIGHT;
        break;
    case TURN_LEFT: {
        float target = rs->originYaw + turn_angle;
        float diff = Angle_Normalize(target - rs->yaw);
        if (diff < 3.0f && diff > -3.0f) g_turnStage = TURN_FORWARD;
        TurnToAngle(rs, target);
        break;
    }
    case TURN_RIGHT: {
        float target = rs->originYaw - turn_angle;
        float diff = Angle_Normalize(target - rs->yaw);
        if (diff < 3.0f && diff > -3.0f) g_turnStage = TURN_FORWARD;
        TurnToAngle(rs, target);
        break;
    }
    case TURN_FORWARD:
        if (rs->isLost == 0) g_turnStage = TURN_IDLE;
        PID_control_head(rs, 30, rs->yaw);
        break;
    }
}

void turn_Task_Reset(void) { g_turnStage = TURN_IDLE; }

/* ---- 旧版 v2 搜线状态机 ---- */
typedef enum
{
    TURN2_IDLE = 0,
    TURN2_STOP,
    TURN2_TURN,
    TURN2_FORWARD,
    TURN2_NEXT
} Turn2Stage;

static Turn2Stage g_turn2Stage = TURN2_IDLE;
static unsigned long g_turn2StartMs = 0;
static int g_turn2SearchAngle = 20;
static int g_turn2Direction = 1;
static int g_turn2SearchCount = 0;

void turn_Task_v2(RobotState *rs)
{
    switch (g_turn2Stage)
    {
    case TURN2_IDLE:
        if (rs->isLost != 0) {
            g_turn2Stage = TURN2_STOP;
            g_turn2StartMs = tick_ms;
            Set_PWM(0, 0);
            rs->originYaw = rs->yaw;
            g_turn2SearchAngle = 20;
            g_turn2Direction = 1;
            g_turn2SearchCount = 0;
        } else {
            PID_control(rs);
        }
        break;
    case TURN2_STOP:
        if (tick_ms - g_turn2StartMs > 200) {
            g_turn2Stage = TURN2_TURN;
            g_turn2StartMs = tick_ms;
        }
        break;
    case TURN2_TURN: {
        float target = rs->originYaw + g_turn2Direction * g_turn2SearchAngle;
        float diff = Angle_Normalize(target - rs->yaw);
        if (diff < 5.0f && diff > -5.0f) {
            g_turn2Stage = TURN2_FORWARD;
            g_turn2StartMs = tick_ms;
            Set_PWM(30, 30);
        } else {
            TurnToAngle(rs, target);
        }
        break;
    }
    case TURN2_FORWARD:
        if (rs->isLost == 0) {
            g_turn2Stage = TURN2_IDLE;
        } else if (tick_ms - g_turn2StartMs > 400) {
            g_turn2Stage = TURN2_NEXT;
        } else {
            PID_control_head(rs, 30, rs->yaw);
        }
        break;
    case TURN2_NEXT:
        g_turn2SearchCount++;
        g_turn2Direction = -g_turn2Direction;
        if (g_turn2Direction == 1) {
            g_turn2SearchAngle += 20;
            if (g_turn2SearchAngle > 80) g_turn2SearchAngle = 80;
        }
        Set_PWM(0, 0);
        g_turn2StartMs = tick_ms;
        g_turn2Stage = TURN2_STOP;
        break;
    }
}

void turn_Task_v2_Reset(void)
{
    g_turn2Stage = TURN2_IDLE;
    g_turn2StartMs = 0;
    g_turn2SearchAngle = 20;
    g_turn2Direction = 1;
    g_turn2SearchCount = 0;
}

void diagonal_Task(RobotState *rs) { (void)rs; /* TODO */ }
void diagonal_Task_Reset(void) {}
