#include "avoid_task.h"
#include "app_state.h"
#include "Motor.h"
#include "speed_loop.h"

typedef enum
{
    AVOID_IDLE = 0,
    AVOID_STOP,
    AVOID_TURN_LEFT,
    AVOID_FORWARD,
    AVOID_TURN_RIGHT,
    AVOID_TURN_LINE
} AvoidStage;

static AvoidStage g_avoidStage = AVOID_IDLE;
static unsigned long g_avoidStartMs = 0;

void ObstacleAvoidance_Reset(void)
{
    g_avoidStage = AVOID_IDLE;
    g_avoidStartMs = 0;
}

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
