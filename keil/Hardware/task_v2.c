#include "task_v2.h"
#include "Sensor.h"

/* ---- 避障参数 (实车标定后调整) ----
 *  轨迹: 右转45° → 直行绕过 → 左转90° → 直行寻线
 *  适用于障碍物在循线传感器右侧的场景。
 */
#define AVOID_TURN_DEG 45          /* 右转避开角度 (度) */
#define AVOID_RETURN_DEG 45        /* 左转回线角度 (度，与 TURN_DEG 合起来左转 90°) */
#define AVOID_DIST_PULSES 1350     /* 绕障直行距离 (编码器脉冲) */
#define AVOID_SPEED 60             /* 避障期间直行速度 (0~100) */
#define AVOID_OBSTACLE_CM 25       /* 超声波障碍检测阈值 (厘米) */
#define AVOID_CONVERGE_THRESH 3.0f /* 转向到位判定阈值 (度) */
#define AVOID_SEEK_MAX_PULSES 2500 /* 寻线阶段最大距离，超时放弃 (编码器脉冲) */

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

extern int16_t dist;
extern int is_lost;
extern float yaw;
extern float origin_yaw;
extern PID_t anglePID;

extern void PID_control(void);

/* 计算两个角度之间的最短差值，归一化到 [-180, 180] */
static float angle_diff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
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
            Set_PWM(0, 0);
            PID_Reset(&anglePID);
            g_avoid2Stage = AVOID2_TURN_LEFT;
        }
        else
        {
            PID_control_head(AVOID_SPEED, origin_yaw + AVOID_TURN_DEG);
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
            PID_control_head(AVOID_SPEED, origin_yaw - AVOID_RETURN_DEG);
        }
        break;
    }
}

void ObstacleAvoidance_Task_v2_Reset(void)
{
    g_avoid2Stage = AVOID2_IDLE;
}

/* ---- 转向 v2 参数 ---- */
#define CORNER_TURN_DEG 135         /* 转角角度 (度) */
#define CORNER_STRAIGHT_PULSES 4600 /* 直行最大距离 (编码器脉冲，约94cm) */
#define CORNER_STRAIGHT_SPEED 80    /* 直行速度 (0~100) */
#define CORNER_ADVANCE_PULSES 300   /* 检角后前移距离 (编码器脉冲，约19cm) */
#define CORNER_RETURN_ADVANCE_PULSES 400 /* 见线后前移距离，对齐旋转中心再转回 */
#define CORNER_STARTUP_PULSES 400        /* 地图外启动，直行入场距离(不计入圈数) */
#define CORNER_CONVERGE_THRESH 3.0f /* 转向到位阈值 (度) */
#define CORNER_DETECT_DEBOUNCE 3    /* 直角检测消抖帧数 */

/* ---- 转向 v2 状态机：传感器检角 + 编码器+MPU6050 对角导航 ----
 * 轨迹: 检测直角 → 前移对齐 → 转135°(朝对角方向) → 直行过对角 →
 *       见线后前移 → 转回 origin_yaw(两端线段平行，回正即可循线)
 * 正方形赛道: 2组 = 1圈 (从A回到A)
 */
typedef enum
{
    CORNER2_IDLE = 0,
    CORNER2_ADVANCE, /* 检角后前移，对齐旋转中心 */
    CORNER2_TURN1,
    CORNER2_STRAIGHT,
    CORNER2_ADVANCE2, /* 见线后前移，对齐旋转中心 */
    CORNER2_TURN2,
    CORNER2_STARTUP /* 地图外A点外侧启动，直行入场 */
} CornerStageV2;

static CornerStageV2 g_corner2Stage = CORNER2_IDLE;
static int32_t g_cornerSegStartPulses = 0;
static int g_cornerTurnDir = 0; /* 1=右转, -1=左转 */
static int g_cornerDetectDebounce = 0;
uint8_t g_cornerSetsCompleted = 0; /* 已完成拐角组数 */

extern uint8_t g_running;

void CornerTurn_Task_v2(void)
{
    /* 直角检测: 一侧4灯全黑 */
    int left_black = (Sensor_GetState(0) == 0 && Sensor_GetState(1) == 0 &&
                      Sensor_GetState(2) == 0 && Sensor_GetState(3) == 0);
    int right_black = (Sensor_GetState(4) == 0 && Sensor_GetState(5) == 0 &&
                       Sensor_GetState(6) == 0 && Sensor_GetState(7) == 0);

    switch (g_corner2Stage)
    {
    case CORNER2_STARTUP:
        /* 地图外A点外侧启动，直行400脉冲进入地图，此段不计入圈数 */
        if (Encoder_GetDistancePulses() - g_cornerSegStartPulses
            > CORNER_STARTUP_PULSES)
        {
            Encoder_ResetDistance();
            g_corner2Stage = CORNER2_IDLE;
        }
        else
        {
            PID_control_head(CORNER_STRAIGHT_SPEED, origin_yaw);
        }
        break;

    case CORNER2_IDLE:
        if (left_black || right_black)
        {
            g_cornerDetectDebounce++;
            if (g_cornerDetectDebounce >= CORNER_DETECT_DEBOUNCE)
            {
                g_cornerDetectDebounce = 0;
                /* 左全黑→左直角→左转+135; 右全黑→右直角→右转-135 */
                if (left_black)
                    g_cornerTurnDir = 1; /* +135° */
                else
                    g_cornerTurnDir = -1; /* -135° */

                origin_yaw = yaw;
                g_cornerSegStartPulses = Encoder_GetDistancePulses();
                g_corner2Stage = CORNER2_ADVANCE;
            }
        }
        else
        {
            g_cornerDetectDebounce = 0;
            PID_control();
        }
        break;

    case CORNER2_ADVANCE:
        /* 传感器靠前，前移让车中心对准拐角再转 */
        if (Encoder_GetDistancePulses() - g_cornerSegStartPulses > CORNER_ADVANCE_PULSES)
        {
            Set_PWM(0, 0);
            PID_Reset(&anglePID);
            g_corner2Stage = CORNER2_TURN1;
        }
        else
        {
            PID_control_head(CORNER_STRAIGHT_SPEED, origin_yaw);
        }
        break;

    case CORNER2_TURN1:
    {
        float target = origin_yaw + g_cornerTurnDir * CORNER_TURN_DEG;
        float diff = angle_diff(target, yaw);
        if (diff < CORNER_CONVERGE_THRESH && diff > -CORNER_CONVERGE_THRESH)
        {
            g_cornerSegStartPulses = Encoder_GetDistancePulses();
            g_corner2Stage = CORNER2_STRAIGHT;
        }
        else
        {
            TurnToAngle(target);
        }
        break;
    }

    case CORNER2_STRAIGHT:
    {
        int mid_black = (Sensor_GetState(3) == 0 && Sensor_GetState(4) == 0);
        int dist_reached = (Encoder_GetDistancePulses() - g_cornerSegStartPulses > CORNER_STRAIGHT_PULSES);
        if (mid_black || dist_reached)
        {
            Set_PWM(0, 0);
            g_cornerSegStartPulses = Encoder_GetDistancePulses();
            g_corner2Stage = CORNER2_ADVANCE2;
        }
        else
        {
            PID_control_head(CORNER_STRAIGHT_SPEED, origin_yaw + g_cornerTurnDir * CORNER_TURN_DEG);
        }
        break;
    }

    case CORNER2_ADVANCE2:
        /* 传感器靠前，见线后前移让车中心对准线段再转回 */
        if (Encoder_GetDistancePulses() - g_cornerSegStartPulses
            > CORNER_RETURN_ADVANCE_PULSES)
        {
            Set_PWM(0, 0);
            PID_Reset(&anglePID);
            g_corner2Stage = CORNER2_TURN2;
        }
        else
        {
            PID_control_head(CORNER_STRAIGHT_SPEED,
                             origin_yaw + g_cornerTurnDir * CORNER_TURN_DEG);
        }
        break;

    case CORNER2_TURN2:
    {
        /* 转回原始航向：对角线两端线段平行，回到 origin_yaw 即可沿下一段循线 */
        float target = origin_yaw;
        float diff = angle_diff(target, yaw);
        if (diff < CORNER_CONVERGE_THRESH && diff > -CORNER_CONVERGE_THRESH)
        {
            g_cornerSetsCompleted++;
            g_corner2Stage = CORNER2_IDLE;
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
    g_corner2Stage = CORNER2_STARTUP;
    g_cornerDetectDebounce = 0;
    g_cornerSetsCompleted = 0;
    origin_yaw = yaw;
    g_cornerSegStartPulses = Encoder_GetDistancePulses();
}
