#include "app_state.h"
#include "h24_task.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "Sensor.h"

typedef enum
{
    H24_STAGE_BLANK_A_TO_B = 0,
    H24_STAGE_ARC_B_TO_C,
    H24_STAGE_BLANK_C_TO_D,
    H24_STAGE_ARC_D_TO_A
} H24Stage;

typedef enum
{
    H24_MENU_MAIN = 0,
    H24_MENU_RUNNING
} H24MenuState;

static H24Stage g_h24Stage = H24_STAGE_BLANK_A_TO_B;
static unsigned long g_h24StageMs = 0;
static float g_h24StraightYaw = 0.0f;
static uint8_t g_h24Task = 0;
static uint8_t g_h24Running = 0;
static H24MenuState g_h24MenuState = H24_MENU_MAIN;
extern volatile unsigned long tick_ms;

#define H24_LINE_SPEED (50)
#define H24_STRAIGHT_SPEED (48)
#define H24_SEEK_SPEED (35)
#define H24_LINE_DEBOUNCE_MS (60UL)
#define H24_RIGHT_CORRECT_DEG (15.0f)

static uint8_t H24_TaskImplemented(uint8_t taskIndex)
{
    return (taskIndex == 0U) ? 1U : 0U;
}

static int H24_ClampInt(int value, int minVal, int maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }
    if (value > maxVal)
    {
        return maxVal;
    }
    return value;
}

static float H24_WrapAngle180(float angleDeg)
{
    while (angleDeg > 180.0f)
    {
        angleDeg -= 360.0f;
    }
    while (angleDeg < -180.0f)
    {
        angleDeg += 360.0f;
    }
    return angleDeg;
}

static int H24_CountActiveLineSensors(void)
{
    int activeCount = 0;

    for (int i = 0; i < 8; ++i)
    {
        if (Sensor_GetState(i) == 1)
        {
            activeCount++;
        }
    }

    return activeCount;
}

static void H24_ApplySpeedTargets(int leftTarget, int rightTarget)
{
    targetLeftSpeed = H24_ClampInt(leftTarget, -90, 90);
    targetRightSpeed = H24_ClampInt(rightTarget, -90, 90);

    leftPID.target = targetLeftSpeed;
    rightPID.target = targetRightSpeed;
    leftPID.actual = leftEncSpeed;
    rightPID.actual = rightEncSpeed;

    PID_Update(&leftPID);
    PID_Update(&rightPID);

    PWMleft = (int) leftPID.output;
    PWMright = (int) rightPID.output;
    Set_PWM(PWMleft, PWMright);
}

static void H24_FollowLineWithSpeed(int baseSpeed)
{
    linePos = Sensor_GetQuantizedPos();

    steerPID.target = 0.0f;
    steerPID.actual = (float) linePos;
    PID_Update(&steerPID);

    H24_ApplySpeedTargets(baseSpeed + (int) steerPID.output,
                          baseSpeed - (int) steerPID.output);
}

static void H24_DriveStraightWithYaw(float targetYawDeg, int baseSpeed)
{
    float yawErr = H24_WrapAngle180(targetYawDeg - yaw);
    float steer = Angle_Control(0.0f, yawErr);

    H24_ApplySpeedTargets(baseSpeed + (int) steer, baseSpeed - (int) steer);
}

static void H24_K0_Task(unsigned long nowMs)
{
    int activeSensors = H24_CountActiveLineSensors();

    switch (g_h24Stage)
    {
    case H24_STAGE_BLANK_A_TO_B:
        H24_DriveStraightWithYaw(g_h24StraightYaw, H24_SEEK_SPEED);
        if (activeSensors > 0)
        {
            g_h24Stage = H24_STAGE_ARC_B_TO_C;
            g_h24StageMs = nowMs;
        }
        break;

    case H24_STAGE_ARC_B_TO_C:
        H24_FollowLineWithSpeed(H24_LINE_SPEED);
        if (activeSensors == 0)
        {
            if (nowMs - g_h24StageMs >= H24_LINE_DEBOUNCE_MS)
            {
                g_h24StraightYaw = H24_WrapAngle180(yaw - H24_RIGHT_CORRECT_DEG);
                g_h24Stage = H24_STAGE_BLANK_C_TO_D;
                g_h24StageMs = nowMs;
            }
        }
        else
        {
            g_h24StageMs = nowMs;
        }
        break;

    case H24_STAGE_BLANK_C_TO_D:
        H24_DriveStraightWithYaw(g_h24StraightYaw, H24_STRAIGHT_SPEED);
        if (activeSensors > 0)
        {
            if (nowMs - g_h24StageMs >= H24_LINE_DEBOUNCE_MS)
            {
                g_h24Stage = H24_STAGE_ARC_D_TO_A;
                g_h24StageMs = nowMs;
            }
        }
        else
        {
            g_h24StageMs = nowMs;
        }
        break;

    case H24_STAGE_ARC_D_TO_A:
        H24_FollowLineWithSpeed(H24_LINE_SPEED);
        if (activeSensors == 0)
        {
            if (nowMs - g_h24StageMs >= H24_LINE_DEBOUNCE_MS)
            {
                g_h24StraightYaw = H24_WrapAngle180(yaw - H24_RIGHT_CORRECT_DEG);
                g_h24Stage = H24_STAGE_BLANK_A_TO_B;
                g_h24StageMs = nowMs;
            }
        }
        else
        {
            g_h24StageMs = nowMs;
        }
        break;
    }
}

static void H24_Reset(void)
{
    g_h24Stage = (H24_CountActiveLineSensors() > 0)
                     ? H24_STAGE_ARC_B_TO_C
                     : H24_STAGE_BLANK_A_TO_B;
    g_h24StageMs = tick_ms;
    g_h24StraightYaw = yaw;
}

static void H24_StartSelectedTask(void)
{
    if (!H24_TaskImplemented(g_h24Task))
    {
        g_h24Running = 0;
        g_h24MenuState = H24_MENU_MAIN;
        Set_PWM(0, 0);
        return;
    }

    g_h24MenuState = H24_MENU_RUNNING;
    g_h24Running = 1;
    H24_Reset();
}

static void H24_RunSelectedTask(unsigned long nowMs)
{
    switch (g_h24Task)
    {
    case 0:
        H24_K0_Task(nowMs);
        break;

    default:
        Set_PWM(0, 0);
        break;
    }
}

static void H24_StartTaskByIndex(uint8_t taskIndex)
{
    g_h24Task = taskIndex;
    H24_StartSelectedTask();
}

static void H24_ShowMainOled(void)
{
    OLED_ShowString(0, 0, "=== H24 TASK ===", OLED_8X16);
    OLED_ShowString(0, 16, "K1:K1 K2:K2", OLED_8X16);
    OLED_ShowString(0, 32, "K3:K3 K4:K4", OLED_8X16);
    OLED_ShowString(0, 48, "Last:K", OLED_8X16);
    OLED_ShowSignedNum(48, 48, (int16_t)(g_h24Task + 1U), 1, OLED_8X16);
    OLED_ShowString(64, 48,
                    H24_TaskImplemented(g_h24Task) ? "READY" : "TODO",
                    OLED_8X16);
}

static void H24_ShowRunningOled(void)
{
    OLED_ShowString(0, 0, "=== RUNNING ===", OLED_8X16);
    OLED_ShowString(0, 16, "Mode:H24", OLED_8X16);
    OLED_ShowString(72, 16, "K", OLED_8X16);
    OLED_ShowSignedNum(88, 16, (int16_t)(g_h24Task + 1U), 1, OLED_8X16);
    OLED_ShowString(0, 32, "Yaw:", OLED_8X16);
    OLED_ShowSignedNum(40, 32, (int16_t) yaw, 4, OLED_8X16);
    OLED_ShowString(0, 48, "K1:Stop", OLED_8X16);
}

void H24_Task(unsigned long nowMs)
{
    uint8_t k1 = Key_GetPressed(0);
    uint8_t k2 = Key_GetPressed(1);
    uint8_t k3 = Key_GetPressed(2);
    uint8_t k4 = Key_GetPressed(3);

    if (g_h24MenuState == H24_MENU_RUNNING && k1)
    {
        g_h24Running = 0;
        g_h24MenuState = H24_MENU_MAIN;
        Set_PWM(0, 0);
    }
    else if (g_h24MenuState == H24_MENU_RUNNING)
    {
        /* Only K1 is active while running. */
    }
    else if (g_h24MenuState == H24_MENU_MAIN)
    {
        if (k1)
        {
            H24_StartTaskByIndex(0U);
        }
        else if (k2)
        {
            H24_StartTaskByIndex(1U);
        }
        else if (k3)
        {
            H24_StartTaskByIndex(2U);
        }
        else if (k4)
        {
            H24_StartTaskByIndex(3U);
        }
    }

    if (g_h24Running)
    {
        H24_RunSelectedTask(nowMs);
    }
    else
    {
        Set_PWM(0, 0);
    }

    OLED_Clear();
    if (g_h24MenuState == H24_MENU_MAIN)
    {
        H24_ShowMainOled();
    }
    else if (g_h24MenuState == H24_MENU_RUNNING)
    {
        H24_ShowRunningOled();
    }
    OLED_Update();
}
