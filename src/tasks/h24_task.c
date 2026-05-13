#include "app_state.h"
#include "h24_task.h"
#include "Key.h"
#include "Motor.h"
#include "OLED.h"
#include "Sensor.h"
#include <math.h>

typedef enum
{
    H24_STAGE_BLANK_A_TO_B = 0,
    H24_STAGE_ARC_B_TO_C,
    H24_STAGE_BLANK_C_TO_D,
    H24_STAGE_ARC_D_TO_A
} H24Stage;

typedef enum
{
    H24_FIG8_TURN_RIGHT = 0,
    H24_FIG8_STRAIGHT_TO_LINE_1,
    H24_FIG8_TRACK_1,
    H24_FIG8_TURN_LEFT,
    H24_FIG8_STRAIGHT_TO_LINE_2,
    H24_FIG8_TRACK_2
} H24Fig8Mode;

typedef enum
{
    H24_SEGMENT_BLACK = 0,
    H24_SEGMENT_WHITE
} H24SegmentType;

typedef enum
{
    H24_MENU_MAIN = 0,
    H24_MENU_RUNNING
} H24MenuState;

static H24Stage g_h24Stage = H24_STAGE_BLANK_A_TO_B;
static float g_h24StraightYaw = 0.0f;
static uint8_t g_h24Task = 0;
static uint8_t g_h24Running = 0;
static H24MenuState g_h24MenuState = H24_MENU_MAIN;
static H24SegmentType g_h24StartSegment = H24_SEGMENT_BLACK;
static H24SegmentType g_h24CurrentSegment = H24_SEGMENT_BLACK;
static uint8_t g_h24AllWhiteFiltered = 0;
static uint8_t g_h24AllWhiteLastRaw = 0;
static unsigned long g_h24AllWhiteLastChangeMs = 0;
static uint8_t g_h24CompletedTransitions = 0;
static H24Fig8Mode g_h24Fig8Mode = H24_FIG8_TURN_RIGHT;
static float g_h24TurnTargetYaw = 0.0f;
extern volatile unsigned long tick_ms;

#define H24_LINE_SPEED (30)
#define H24_STRAIGHT_SPEED (48)
#define H24_SEEK_SPEED (35)
#define H24_IR_FILTER_MS (20UL)
#define H24_RIGHT_CORRECT_DEG (8.0f)
#define H24_TRANSITIONS_PER_LAP (4U)
#define H24_TRACK_STEER_SCALE (1.70f)
#define H24_FIG8_ENTRY_TURN_DEG (35.0f)

static uint8_t H24_TaskImplemented(uint8_t taskIndex)
{
    return (taskIndex <= 1U) ? 1U : 0U;
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
        if (Sensor_GetState(i) != 1)
        {
            activeCount++;
        }
    }

    return activeCount;
}

static uint8_t H24_IsAllWhite(void)
{
    return (H24_CountActiveLineSensors() == 0) ? 1U : 0U;
}

static uint8_t H24_GetAllWhiteFiltered(unsigned long nowMs)
{
    uint8_t raw = H24_IsAllWhite();

    if (raw != g_h24AllWhiteLastRaw)
    {
        g_h24AllWhiteLastRaw = raw;
        g_h24AllWhiteLastChangeMs = nowMs;
    }

    if ((nowMs - g_h24AllWhiteLastChangeMs) >= H24_IR_FILTER_MS)
    {
        g_h24AllWhiteFiltered = raw;
    }

    return g_h24AllWhiteFiltered;
}

static uint8_t H24_OnSegmentTransition(H24SegmentType nextSegment)
{
    if (nextSegment == g_h24CurrentSegment)
    {
        return 0U;
    }

    g_h24CurrentSegment = nextSegment;
    g_h24CompletedTransitions++;

    if ((g_h24CurrentSegment == g_h24StartSegment) &&
        ((g_h24CompletedTransitions % H24_TRANSITIONS_PER_LAP) == 0U))
    {
        return 1U;
    }

    return 0U;
}

static void H24_StopRunning(void)
{
    g_h24Running = 0;
    g_h24MenuState = H24_MENU_MAIN;
    Set_PWM(0, 0);
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
    float steerOutput;

    linePos = Sensor_GetQuantizedPos();

    steerPID.target = 0.0f;
    steerPID.actual = (float) linePos;
    PID_Update(&steerPID);
    steerOutput = steerPID.output * H24_TRACK_STEER_SCALE;

    H24_ApplySpeedTargets(baseSpeed + (int) steerOutput,
                          baseSpeed - (int) steerOutput);
}

static void H24_DriveStraightWithYaw(float targetYawDeg, int baseSpeed)
{
    float yawErr = H24_WrapAngle180(targetYawDeg - yaw);
    float steer = Angle_Control(0.0f, yawErr);

    H24_ApplySpeedTargets(baseSpeed + (int) steer, baseSpeed - (int) steer);
}

static void H24_K0_Task(unsigned long nowMs)
{
    uint8_t allWhite = H24_GetAllWhiteFiltered(nowMs);

    switch (g_h24Stage)
    {
    case H24_STAGE_BLANK_A_TO_B:
        H24_DriveStraightWithYaw(g_h24StraightYaw, H24_SEEK_SPEED);
        if (!allWhite)
        {
            if (H24_OnSegmentTransition(H24_SEGMENT_BLACK))
            {
                H24_StopRunning();
                break;
            }
            g_h24Stage = H24_STAGE_ARC_B_TO_C;
        }
        break;

    case H24_STAGE_ARC_B_TO_C:
        H24_FollowLineWithSpeed(H24_LINE_SPEED);
        if (allWhite)
        {
            if (H24_OnSegmentTransition(H24_SEGMENT_WHITE))
            {
                H24_StopRunning();
                break;
            }
            g_h24StraightYaw = H24_WrapAngle180(yaw - H24_RIGHT_CORRECT_DEG);
            g_h24Stage = H24_STAGE_BLANK_C_TO_D;
        }
        break;

    case H24_STAGE_BLANK_C_TO_D:
        H24_DriveStraightWithYaw(g_h24StraightYaw, H24_STRAIGHT_SPEED);
        if (!allWhite)
        {
            if (H24_OnSegmentTransition(H24_SEGMENT_BLACK))
            {
                H24_StopRunning();
                break;
            }
            g_h24Stage = H24_STAGE_ARC_D_TO_A;
        }
        break;

    case H24_STAGE_ARC_D_TO_A:
        H24_FollowLineWithSpeed(H24_LINE_SPEED);
        if (allWhite)
        {
            if (H24_OnSegmentTransition(H24_SEGMENT_WHITE))
            {
                H24_StopRunning();
                break;
            }
            g_h24StraightYaw = H24_WrapAngle180(yaw - H24_RIGHT_CORRECT_DEG);
            g_h24Stage = H24_STAGE_BLANK_A_TO_B;
        }
        break;
    }
}

static void H24_K1_Fig8_Task(unsigned long nowMs)
{
    uint8_t allWhite = H24_GetAllWhiteFiltered(nowMs);

    (void)nowMs;

    switch (g_h24Fig8Mode)
    {
    case H24_FIG8_TURN_RIGHT:
        TurnToAngle(g_h24TurnTargetYaw);
        if (fabsf(H24_WrapAngle180(yaw - g_h24TurnTargetYaw)) <= 5.0f)
        {
            g_h24StraightYaw = g_h24TurnTargetYaw;
            g_h24Fig8Mode = H24_FIG8_STRAIGHT_TO_LINE_1;
            g_h24Stage = H24_STAGE_BLANK_A_TO_B;
        }
        break;

    case H24_FIG8_STRAIGHT_TO_LINE_1:
        H24_DriveStraightWithYaw(g_h24StraightYaw, H24_SEEK_SPEED);
        if (!allWhite)
        {
            g_h24Fig8Mode = H24_FIG8_TRACK_1;
            g_h24Stage = H24_STAGE_ARC_B_TO_C;
        }
        break;

    case H24_FIG8_TRACK_1:
        H24_FollowLineWithSpeed(H24_LINE_SPEED);
        if (allWhite)
        {
            /* 保留原有的出线修正，再从修正后的朝向左转 40 度进入第二段。 */
            g_h24StraightYaw = H24_WrapAngle180(yaw - H24_RIGHT_CORRECT_DEG);
            g_h24TurnTargetYaw = H24_WrapAngle180(
                g_h24StraightYaw + H24_FIG8_ENTRY_TURN_DEG);
            g_h24Fig8Mode = H24_FIG8_TURN_LEFT;
            g_h24Stage = H24_STAGE_BLANK_C_TO_D;
        }
        break;

    case H24_FIG8_TURN_LEFT:
        TurnToAngle(g_h24TurnTargetYaw);
        if (fabsf(H24_WrapAngle180(yaw - g_h24TurnTargetYaw)) <= 5.0f)
        {
            g_h24StraightYaw = g_h24TurnTargetYaw;
            g_h24Fig8Mode = H24_FIG8_STRAIGHT_TO_LINE_2;
            g_h24Stage = H24_STAGE_BLANK_C_TO_D;
        }
        break;

    case H24_FIG8_STRAIGHT_TO_LINE_2:
        H24_DriveStraightWithYaw(g_h24StraightYaw, H24_SEEK_SPEED);
        if (!allWhite)
        {
            g_h24Fig8Mode = H24_FIG8_TRACK_2;
            g_h24Stage = H24_STAGE_ARC_D_TO_A;
        }
        break;

    case H24_FIG8_TRACK_2:
        H24_FollowLineWithSpeed(H24_LINE_SPEED);
        if (allWhite)
        {
            H24_StopRunning();
        }
        break;
    }
}

static void H24_Reset(uint8_t taskIndex)
{
    uint8_t allWhite = H24_IsAllWhite();

    g_h24AllWhiteFiltered = allWhite;
    g_h24AllWhiteLastRaw = allWhite;
    g_h24AllWhiteLastChangeMs = tick_ms;
    g_h24CompletedTransitions = 0;
    g_h24StartSegment = allWhite ? H24_SEGMENT_WHITE : H24_SEGMENT_BLACK;
    g_h24CurrentSegment = g_h24StartSegment;
    if (taskIndex == 0U)
    {
        g_h24Stage = allWhite ? H24_STAGE_BLANK_A_TO_B : H24_STAGE_ARC_B_TO_C;
        g_h24StraightYaw = yaw;
    }
    else
    {
        g_h24TurnTargetYaw = H24_WrapAngle180(yaw - H24_FIG8_ENTRY_TURN_DEG);
        g_h24Fig8Mode = allWhite ? H24_FIG8_TURN_RIGHT : H24_FIG8_TRACK_1;
        g_h24Stage = allWhite ? H24_STAGE_BLANK_A_TO_B : H24_STAGE_ARC_B_TO_C;
        g_h24StraightYaw = yaw;
    }
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
    H24_Reset(g_h24Task);
}

static void H24_RunSelectedTask(unsigned long nowMs)
{
    switch (g_h24Task)
    {
    case 0:
        H24_K0_Task(nowMs);
        break;

    case 1:
        H24_K1_Fig8_Task(nowMs);
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
    OLED_ShowString(0, 0, "H24", OLED_8X16);
    OLED_ShowString(0, 16, "K1: LOOP", OLED_8X16);
    OLED_ShowString(0, 32, "K2: FIG8", OLED_8X16);
    OLED_ShowString(0, 48, "WAIT KEY", OLED_8X16);
}

static char *H24_GetStageText(void)
{
    static char kTrackText[] = "TRACK";
    static char kStraightText[] = "STRAIGHT";

    switch (g_h24Stage)
    {
    case H24_STAGE_ARC_B_TO_C:
    case H24_STAGE_ARC_D_TO_A:
        return kTrackText;

    case H24_STAGE_BLANK_A_TO_B:
    case H24_STAGE_BLANK_C_TO_D:
    default:
        return kStraightText;
    }
}

static void H24_ShowRunningOled(void)
{
    OLED_ShowString(0, 0, "State:", OLED_8X16);
    OLED_ShowString(48, 0, H24_GetStageText(), OLED_8X16);
    OLED_ShowString(0, 16, "Yaw:", OLED_8X16);
    OLED_ShowSignedNum(40, 16, (int16_t) yaw, 4, OLED_8X16);
    OLED_ShowString(0, 32, "L:", OLED_8X16);
    OLED_ShowSignedNum(24, 32, leftEncSpeed, 4, OLED_8X16);
    OLED_ShowString(64, 32, "R:", OLED_8X16);
    OLED_ShowSignedNum(88, 32, rightEncSpeed, 4, OLED_8X16);
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
