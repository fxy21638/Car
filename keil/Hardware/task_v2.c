#include "task_v2.h"

/* ---- 避障参数 (实车标定后调整) ----
 *  轨迹: 右转45° → 直行绕过 → 左转90° → 直行寻线
 *  适用于障碍物在循线传感器右侧的场景。
 */
#define AVOID_TURN_DEG         45     /* 右转避开角度 (度) */
#define AVOID_RETURN_DEG       45     /* 左转回线角度 (度，与 TURN_DEG 合起来左转 90°) */
#define AVOID_DIST_PULSES      1350   /* 绕障直行距离 (编码器脉冲) */
#define AVOID_SPEED            60     /* 避障期间直行速度 (0~100) */
#define AVOID_OBSTACLE_CM      25     /* 超声波障碍检测阈值 (厘米) */
#define AVOID_CONVERGE_THRESH  5.0f   /* 转向到位判定阈值 (度) */
#define AVOID_SEEK_MAX_PULSES  2500   /* 寻线阶段最大距离，超时放弃 (编码器脉冲) */

/* ---- 避障 v2 状态机：编码器测距 + MPU6050 转角度 ---- */
typedef enum
{
    AVOID2_IDLE = 0,
    AVOID2_STOP,
    AVOID2_TURN_RIGHT,
    AVOID2_FORWARD,
    AVOID2_TURN_LEFT,
    AVOID2_SEEK_LINE
} AvoidStageV2;

static AvoidStageV2 g_avoid2Stage = AVOID2_IDLE;
static int32_t g_segmentStartPulses = 0;  /* 阶段起点脉冲快照，不动累计值 */

extern int16_t dist;
extern int is_lost;
extern float yaw;
extern float origin_yaw;

extern void PID_control(void);

/* 计算两个角度之间的最短差值，归一化到 [-180, 180] */
static float angle_diff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/*
 * 避障状态机 v2 (右避障) — 编码器测距 + MPU6050 转角，不依赖延时
 *
 * 轨迹: 障碍物在线上右侧，车右转 45° 斜出 → 直行绕过 →
 *       左转 90° 回到与线平行的方向 → 直行寻线归位。
 * 每阶段用编码器脉冲差值计距，TurnToAngle 非阻塞转角度。
 */
void ObstacleAvoidance_Task_v2(void)
{
    switch (g_avoid2Stage)
    {
    case AVOID2_IDLE:
        if (dist > 0 && dist < AVOID_OBSTACLE_CM)
        {
            g_avoid2Stage = AVOID2_STOP;
        }
        else
        {
            PID_control();
        }
        break;

    case AVOID2_STOP:
        Set_PWM(0, 0);
        origin_yaw = yaw;
        g_segmentStartPulses = Encoder_GetDistancePulses();
        g_avoid2Stage = AVOID2_TURN_RIGHT;
        break;

    case AVOID2_TURN_RIGHT:
    {
        float target = origin_yaw + AVOID_TURN_DEG;
        float diff = angle_diff(target, yaw);
        if (diff < AVOID_CONVERGE_THRESH && diff > -AVOID_CONVERGE_THRESH)
        {
            g_segmentStartPulses = Encoder_GetDistancePulses();
            g_avoid2Stage = AVOID2_FORWARD;
        }
        else
        {
            TurnToAngle(target);
        }
        break;
    }

    case AVOID2_FORWARD:
        if (Encoder_GetDistancePulses() - g_segmentStartPulses > AVOID_DIST_PULSES)
        {
            g_avoid2Stage = AVOID2_TURN_LEFT;
        }
        else
        {
            PID_control_head(AVOID_SPEED, AVOID_SPEED);
        }
        break;

    case AVOID2_TURN_LEFT:
    {
        float target = origin_yaw - AVOID_RETURN_DEG;
        float diff = angle_diff(target, yaw);
        if (diff < AVOID_CONVERGE_THRESH && diff > -AVOID_CONVERGE_THRESH)
        {
            g_segmentStartPulses = Encoder_GetDistancePulses();
            g_avoid2Stage = AVOID2_SEEK_LINE;
        }
        else
        {
            TurnToAngle(target);
        }
        break;
    }

    case AVOID2_SEEK_LINE:
        if (is_lost == 0)
        {
            g_avoid2Stage = AVOID2_IDLE;
        }
        else if (Encoder_GetDistancePulses() - g_segmentStartPulses > AVOID_SEEK_MAX_PULSES)
        {
            g_avoid2Stage = AVOID2_IDLE;
        }
        else
        {
            PID_control_head(AVOID_SPEED, AVOID_SPEED);
        }
        break;
    }
}

void ObstacleAvoidance_Task_v2_Reset(void)
{
    g_avoid2Stage = AVOID2_IDLE;
}
