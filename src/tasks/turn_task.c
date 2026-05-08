#include "app_state.h"
#include "turn_task.h"
#include "Motor.h"
#include "MPU6050_MSPM0.h"
#include "speed_loop.h"
#include <math.h>

typedef enum
{
    TURN_IDLE = 0,
    TURN_STOP,
    TURN_LEFT,
    TURN_FORWARD,
    TURN_RIGHT
} TurnStage;

static TurnStage g_turnStage = TURN_IDLE;

void TurnTask_Reset(void)
{
    g_turnStage = TURN_IDLE;
}

void turn_Task(void)
{
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
