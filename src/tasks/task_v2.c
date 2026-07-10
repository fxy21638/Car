#include "task_v2.h"
#include "Sensor.h"
#include "app_config.h"
#include "app_fault.h"
#include "Delay.h"
#include "control_math.h"

/* ---- 避障参数 (实车标定后调整) ----
 *  轨迹: 右转45° → 直行绕过 → 左转90° → 直行寻线
 *  适用于障碍物在循线传感器右侧的场景。
 */
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
static int32_t g_segmentStartPulses = 0; /* 阶段起点脉冲快照，不动累计值 */
static unsigned long g_avoidStageStartMs = 0;

extern int16_t dist;
extern int is_lost;
extern float yaw;
extern float origin_yaw;
extern PID_t anglePID;

extern void PID_control(void);

/* 计算两个角度之间的最短差值，归一化到 [-180, 180] */
static void avoid_enter(AvoidStageV2 stage)
{
    g_avoid2Stage = stage;
    g_avoidStageStartMs = tick_ms;
}

static int avoid_timed_out(unsigned long limitMs)
{
    return (unsigned long)(tick_ms - g_avoidStageStartMs) >= limitMs;
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
    if ((g_avoid2Stage == AVOID2_TURN_RIGHT || g_avoid2Stage == AVOID2_TURN_LEFT) &&
        avoid_timed_out(APP_TURN_TIMEOUT_MS))
    {
        AppFault_Raise(APP_FAULT_TASK_TIMEOUT);
        return;
    }
    if (g_avoid2Stage == AVOID2_FORWARD && avoid_timed_out(APP_FORWARD_TIMEOUT_MS))
    {
        AppFault_Raise(APP_FAULT_TASK_TIMEOUT);
        return;
    }
    if (g_avoid2Stage == AVOID2_SEEK_LINE && avoid_timed_out(APP_SEEK_TIMEOUT_MS))
    {
        AppFault_Raise(APP_FAULT_TASK_TIMEOUT);
        return;
    }

    switch (g_avoid2Stage)
    {
    case AVOID2_IDLE:
        if (dist > 0 && dist < APP_AVOID_OBSTACLE_CM)
        {
            avoid_enter(AVOID2_STOP);
        }
        else
        {
            PID_control();
        }
        break;

    case AVOID2_STOP:
        Set_PWM(0, 0);           /* 停车快照，一帧后立即进入右转，避免惯性冲过 */
        origin_yaw = yaw;
        g_segmentStartPulses = Encoder_GetDistancePulses();
        avoid_enter(AVOID2_TURN_RIGHT);
        break;

    case AVOID2_TURN_RIGHT:
    {
        float target = origin_yaw + APP_AVOID_TURN_DEG;
        float diff = Control_AngleDifference(target, yaw);
        if (diff < APP_AVOID_CONVERGE_DEG && diff > -APP_AVOID_CONVERGE_DEG)
        {
            g_segmentStartPulses = Encoder_GetDistancePulses();
            avoid_enter(AVOID2_FORWARD);
        }
        else
        {
            TurnToAngle(target);
        }
        break;
    }

    case AVOID2_FORWARD:
        if (Encoder_GetDistancePulses() - g_segmentStartPulses > APP_AVOID_DIST_PULSES)
        {
            Set_PWM(0, 0);
            PID_Reset(&anglePID); /* 清空直行积累的角度积分，避免阻碍后续转向 */
            avoid_enter(AVOID2_TURN_LEFT);
        }
        else
        {
            PID_control_head(APP_AVOID_SPEED, origin_yaw + APP_AVOID_TURN_DEG);
        }
        break;

    case AVOID2_TURN_LEFT:
    {
        float target = origin_yaw - APP_AVOID_RETURN_DEG;
        float diff = Control_AngleDifference(target, yaw);
        if (diff < APP_AVOID_CONVERGE_DEG && diff > -APP_AVOID_CONVERGE_DEG)
        {
            g_segmentStartPulses = Encoder_GetDistancePulses();
            avoid_enter(AVOID2_SEEK_LINE);
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
            avoid_enter(AVOID2_IDLE);
        }
        else if (Encoder_GetDistancePulses() - g_segmentStartPulses > APP_AVOID_SEEK_MAX_PULSES)
        {
            avoid_enter(AVOID2_IDLE);
        }
        else
        {
            PID_control_head(APP_AVOID_SPEED, origin_yaw - APP_AVOID_RETURN_DEG);
        }
        break;
    }
}

void ObstacleAvoidance_Task_v2_Reset(void)
{
    avoid_enter(AVOID2_IDLE);
}

/* ---- 转向 v2 参数 ---- */
/* ---- 转向 v2 状态机：传感器检角 + 编码器+MPU6050 对角导航 ----
 * 轨迹: 检测直角 → 前移对齐 → 转137°(朝对角方向) → 直行过对角 →
 *       见线后前移 → 转回 origin_yaw(两端线段平行，回正即可循线)
 * 地图内启动，正方形赛道: 4次转角 = 1圈 (从A回到A)
 */
typedef enum
{
    CORNER2_IDLE = 0,
    CORNER2_ADVANCE, /* 检角后前移，对齐旋转中心 */
    CORNER2_TURN1,
    CORNER2_STRAIGHT,
    CORNER2_ADVANCE2, /* 见线后前移，对齐旋转中心 */
    CORNER2_TURN2
} CornerStageV2;

static CornerStageV2 g_corner2Stage = CORNER2_IDLE;
static int32_t g_cornerSegStartPulses = 0;
static int g_cornerTurnDir = 0; /* 1=右转, -1=左转 */
static int g_cornerDetectDebounce = 0;
static unsigned long g_cornerStageStartMs = 0;
uint8_t g_cornerTurnsCompleted = 0; /* 已完成转角次数 (TURN1+TURN2)，4次=1圈 */

extern uint8_t g_running;

static void corner_enter(CornerStageV2 stage)
{
    g_corner2Stage = stage;
    g_cornerStageStartMs = tick_ms;
}

static int corner_timed_out(unsigned long limitMs)
{
    return (unsigned long)(tick_ms - g_cornerStageStartMs) >= limitMs;
}

void CornerTurn_Task_v2(void)
{
    /* 直角检测: 一侧4灯全黑 */
    int left_black = (Sensor_GetState(0) == 0 && Sensor_GetState(1) == 0 &&
                      Sensor_GetState(2) == 0 && Sensor_GetState(3) == 0);
    int right_black = (Sensor_GetState(4) == 0 && Sensor_GetState(5) == 0 &&
                       Sensor_GetState(6) == 0 && Sensor_GetState(7) == 0);

    if ((g_corner2Stage == CORNER2_TURN1 || g_corner2Stage == CORNER2_TURN2) &&
        corner_timed_out(APP_TURN_TIMEOUT_MS))
    {
        AppFault_Raise(APP_FAULT_TASK_TIMEOUT);
        return;
    }
    if ((g_corner2Stage == CORNER2_ADVANCE || g_corner2Stage == CORNER2_ADVANCE2) &&
        corner_timed_out(APP_ADVANCE_TIMEOUT_MS))
    {
        AppFault_Raise(APP_FAULT_TASK_TIMEOUT);
        return;
    }
    if (g_corner2Stage == CORNER2_STRAIGHT && corner_timed_out(APP_STRAIGHT_TIMEOUT_MS))
    {
        AppFault_Raise(APP_FAULT_TASK_TIMEOUT);
        return;
    }

    switch (g_corner2Stage)
    {
    case CORNER2_IDLE:
        if (left_black || right_black)
        {
            g_cornerDetectDebounce++;
            if (g_cornerDetectDebounce >= APP_CORNER_DETECT_DEBOUNCE)
            {
                g_cornerDetectDebounce = 0;
                /* 左全黑→左直角→左转+137; 右全黑→右直角→右转-137 */
                if (left_black)
                    g_cornerTurnDir = 1; /* +137° (MPU6050: CCW为正) */
                else
                    g_cornerTurnDir = -1; /* -137° */

                origin_yaw = yaw;
                g_cornerSegStartPulses = Encoder_GetDistancePulses();
                corner_enter(CORNER2_ADVANCE);
            }
        }
        else
        {
            g_cornerDetectDebounce = 0;
            PID_control();
        }
        break;

    case CORNER2_ADVANCE:
        /* 传感器物理位置在旋转中心前方，需前移让旋转中心对准拐角顶点再原地转 */
        if (Encoder_GetDistancePulses() - g_cornerSegStartPulses > APP_CORNER_ADVANCE_PULSES)
        {
            PID_Reset(&anglePID); /* 清空直行角度积分，为原地大角度转向准备干净的控制器 */
            corner_enter(CORNER2_TURN1);
        }
        else
        {
            PID_control_head(APP_CORNER_STRAIGHT_SPEED, origin_yaw);
        }
        break;

    case CORNER2_TURN1:
    {
        float target = origin_yaw + g_cornerTurnDir * APP_CORNER_TURN_DEG;
        float diff = Control_AngleDifference(target, yaw);
        if (diff < APP_CORNER_CONVERGE_DEG && diff > -APP_CORNER_CONVERGE_DEG)
        {
            g_cornerTurnsCompleted++;
            g_cornerSegStartPulses = Encoder_GetDistancePulses();
            corner_enter(CORNER2_STRAIGHT);
        }
        else
        {
            TurnToAngle(target);
        }
        break;
    }

    case CORNER2_STRAIGHT:
    {
        /* S3和S4是中间两个传感器，同时见线说明车已到达对边线段上 */
        int mid_black = (Sensor_GetState(3) == 0 && Sensor_GetState(4) == 0);
        int dist_reached = (Encoder_GetDistancePulses() - g_cornerSegStartPulses > APP_CORNER_STRAIGHT_PULSES);
        if (mid_black || dist_reached) /* 见线即停 或 编码器超距保护 */
        {
            g_cornerSegStartPulses = Encoder_GetDistancePulses();
            corner_enter(CORNER2_ADVANCE2);
        }
        else
        {
            PID_control_head(APP_CORNER_STRAIGHT_SPEED, origin_yaw + g_cornerTurnDir * APP_CORNER_TURN_DEG);
        }
        break;
    }

    case CORNER2_ADVANCE2:
        /* 传感器见线时旋转中心尚未到达线段，前移对齐后再转回原始航向 */
        if (Encoder_GetDistancePulses() - g_cornerSegStartPulses
            > APP_CORNER_RETURN_PULSES)
        {
            PID_Reset(&anglePID); /* 清空直行积分，为转回 origin_yaw 准备 */
            corner_enter(CORNER2_TURN2);
        }
        else
        {
            PID_control_head(APP_CORNER_STRAIGHT_SPEED,
                             origin_yaw + g_cornerTurnDir * APP_CORNER_TURN_DEG);
        }
        break;

    case CORNER2_TURN2:
    {
        /* 转回原始航向：正方形对边互相平行，回到 origin_yaw 即可沿下一段继续循线 */
        float target = origin_yaw;
        float diff = Control_AngleDifference(target, yaw);
        if (diff < APP_CORNER_CONVERGE_DEG && diff > -APP_CORNER_CONVERGE_DEG)
        {
            g_cornerTurnsCompleted++;
            corner_enter(CORNER2_IDLE);
        }
        else
        {
            TurnToAngle(target);
        }
        break;
    }
    }
}

void CornerTurn_Task_v2_Reset(void)
{
    corner_enter(CORNER2_IDLE);
    g_cornerDetectDebounce = 0;
    g_cornerTurnsCompleted = 0;
}
