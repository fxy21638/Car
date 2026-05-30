#include "task.h"
#include "Delay.h"
#include "Motor.h"
#include "MPU6050_MSPM0.h"
#include "PID.h"
#include "Ultrasonic.h"
#include <math.h>

void SpeedLoop_Task(void);

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

extern int16_t dist;
extern int is_lost;
extern float yaw;
extern float origin_yaw;
extern MPU6050_Handle gImu;
extern volatile unsigned long tick_ms;

extern void PID_control(void);
extern void motor_agle_control(float relAngleDeg);

/* 简单定时避障状态机：
 * 停车 -> 原地右转 -> 前进绕行 -> 左转回线。 */
void ObstacleAvoidance_Task(unsigned long nowMs)
{
    switch (g_avoidStage)
    {
    case AVOID_IDLE:
        if (dist > 0 && dist < 25)
        {
            g_avoidStage = AVOID_STOP;
            g_avoidStartMs = nowMs;
            Set_PWM(0, 0);
        }
        else
        {
            SpeedLoop_Task();
        }
        break;

    case AVOID_STOP:
        if (nowMs - g_avoidStartMs > 500)
        {
            g_avoidStage = AVOID_TURN_RIGHT;
            g_avoidStartMs = nowMs;
            Set_PWM(30, -30);
        }
        break;

    case AVOID_TURN_RIGHT:
        if (nowMs - g_avoidStartMs > 300)
        {
            g_avoidStage = AVOID_FORWARD;
            g_avoidStartMs = nowMs;
            Set_PWM(30, 30);
        }
        break;

    case AVOID_FORWARD:
        if (nowMs - g_avoidStartMs > 1000)
        {
            g_avoidStage = AVOID_TURN_LEFT;
            g_avoidStartMs = nowMs;
            Set_PWM(-30, 30);
        }
        break;

    case AVOID_TURN_LEFT:
        if (nowMs - g_avoidStartMs > 300)
        {
            g_avoidStage = AVOID_TURN_LINE;
            g_avoidStartMs = nowMs;
            Set_PWM(30, 45);
        }
        break;

    case AVOID_TURN_LINE:
        if (is_lost == 0)
        {
            g_avoidStage = AVOID_IDLE;
            g_avoidStartMs = nowMs;
        }
        break;
    }
}

/* 计算两个角度之间的最短差值，归一化到 [-180, 180] */
static float angle_diff(float a, float b)
{
    float d = a - b;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

/* 丢线恢复状态机：
 * 先记录当前航向，再按固定相对角度转向，直到重新找到线。 */
void turn_Task(void)
{
    int turn_angle = 45;

    switch (g_turnStage)
    {
    case TURN_IDLE:
        if (is_lost != 0)
        {
            g_turnStage = TURN_STOP;
            Set_PWM(0, 0);
        }
        else
        {
            SpeedLoop_Task();
        }
        break;

    case TURN_STOP:
        origin_yaw = yaw;
        if (is_lost == 1)
        {
            g_turnStage = TURN_LEFT;
        }
        else if (is_lost == -1)
        {
            g_turnStage = TURN_RIGHT;
        }
        break;

    case TURN_LEFT:
    {
        float target = origin_yaw + turn_angle;
        float diff = angle_diff(target, yaw);
        if (diff < 3.0f && diff > -3.0f)
        {
            g_turnStage = TURN_FORWARD;
        }
        TurnToAngle(target);
        break;
    }

    case TURN_RIGHT:
    {
        float target = origin_yaw - turn_angle;
        float diff = angle_diff(target, yaw);
        if (diff < 3.0f && diff > -3.0f)
        {
            g_turnStage = TURN_FORWARD;
        }
        TurnToAngle(target);
        break;
    }

    case TURN_FORWARD:
		if (is_lost == 0)
        {
            g_turnStage = TURN_IDLE;
        }
        PID_control_head(30, 30);
        break;
    }
}

void turn_Task_Reset(void)
{
    g_turnStage = TURN_IDLE;
}

void SpeedLoop_Task(void)
{
    PID_control();
}

/* turn_Task_v2 扩展搜索状态机：
 * 丢线后以递进角度 (±20°, ±40°, ±60°, ±80°) 交替左右搜索，
 * 每个角度短暂前进一段后检查是否回线。 */
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
static int g_turn2Direction = 1;   /* 1=左, -1=右 */
static int g_turn2SearchCount = 0;

void turn_Task_v2(void)
{
    switch (g_turn2Stage)
    {
    case TURN2_IDLE:
        if (is_lost != 0)
        {
            g_turn2Stage = TURN2_STOP;
            g_turn2StartMs = tick_ms;
            Set_PWM(0, 0);
            origin_yaw = yaw;
            g_turn2SearchAngle = 20;
            g_turn2Direction = 1;
            g_turn2SearchCount = 0;
        }
        else
        {
            PID_control();
        }
        break;

    case TURN2_STOP:
        if (tick_ms - g_turn2StartMs > 200)
        {
            g_turn2Stage = TURN2_TURN;
            g_turn2StartMs = tick_ms;
        }
        break;

    case TURN2_TURN:
    {
        float target = origin_yaw + g_turn2Direction * g_turn2SearchAngle;
        float diff = angle_diff(target, yaw);
        if (diff < 5.0f && diff > -5.0f)
        {
            g_turn2Stage = TURN2_FORWARD;
            g_turn2StartMs = tick_ms;
            Set_PWM(30, 30);
        }
        else
        {
            TurnToAngle(target);
        }
        break;
    }

    case TURN2_FORWARD:
        if (is_lost == 0)
        {
            g_turn2Stage = TURN2_IDLE;
        }
        else if (tick_ms - g_turn2StartMs > 400)
        {
            g_turn2Stage = TURN2_NEXT;
        }
        else
        {
            PID_control_head(30, 30);
        }
        break;

    case TURN2_NEXT:
        g_turn2SearchCount++;
        g_turn2Direction = -g_turn2Direction;
        if (g_turn2Direction == 1)
        {
            g_turn2SearchAngle += 20;
            if (g_turn2SearchAngle > 80)
                g_turn2SearchAngle = 80;
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
