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
        if (dist > 0 && dist < 300)
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
            Set_PWM(30, 37);
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

void SpeedLoop_Task(void)
{
    PID_control();
}
