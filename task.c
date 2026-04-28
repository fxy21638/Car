#include "task.h"
#include "PID.h"
#include "Motor.h"
#include "Ultrasonic.h"
#include "MPU6050_MSPM0.h"
#include "Delay.h"
#include <math.h>

/* 状态枚举与模块内部状态变量 (从 empty.c 移出) */
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
} turnStage;

static AvoidStage g_avoidStage = AVOID_IDLE;
static unsigned long g_avoidStartMs = 0;

static turnStage g_turnStage = TURN_IDLE;

/* 外部变量/函数引用（定义在 empty.c / 其它模块） */
extern int16_t dist;
extern int is_lost;
extern float yaw;
extern float origin_yaw;
extern MPU6050_Handle gImu;
extern volatile unsigned long tick_ms;

extern void PID_control(void);
extern void motor_agle_control(float relAngleDeg);

/* 简单避障任务状态机 */
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
            PID_control();
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

/* 转向任务状态机 */
void turn_Task(void)
{
    unsigned long now = tick_ms; /* tick_ms 在工程全局可用 */
    int turn_angle = 30;

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
            PID_control();
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
        // 目标为相对 +30 度
        if (fabsf(yaw - (origin_yaw + turn_angle)) <= 5.0f)
        {
            g_turnStage = TURN_FORWARD;
            Set_PWM(0, 0);
        }
        TurnToAngle(origin_yaw + turn_angle);
        break;

    case TURN_RIGHT:
        if (fabsf(yaw - (origin_yaw - turn_angle)) <= 5.0f)
        {
            g_turnStage = TURN_FORWARD;
            Set_PWM(0, 0);
        }
        TurnToAngle(origin_yaw - turn_angle);
        break;

    case TURN_FORWARD:
        Set_PWM(30, 30);
        if (is_lost == 0)
        {
            g_turnStage = TURN_IDLE;
        }
        break;
    }
}
