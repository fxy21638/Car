#include "task.h"
#include "PID.h"
#include "Motor.h"
#include "Ultrasonic.h"
#include "MPU6050_MSPM0.h"
#include "Sensor.h"
#include "Encoder.h"
#include "Key.h"
#include "Delay.h"
#include <math.h>

/* K0 任务的赛道运行状态 */
typedef enum
{
    TRACK_IDLE = 0,
    TRACK_VISIBLE_CURVE,
    TRACK_HIDDEN_STRAIGHT
} TrackState;

typedef enum
{
    SEGMENT_BLACK = 0,
    SEGMENT_WHITE
} SegmentType;

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

static const uint8_t kStartKey = 0;
static const int kCurveBaseSpeed = 52;
static const int kHiddenBaseSpeed = 48;
static const unsigned long kHiddenStraightMinMs = 120UL;
static const unsigned long kHiddenEntryFollowMs = 80UL;
static const unsigned long kIrFilterMs = 20UL;
static const float kHiddenYawKp = 0.6f;
static const int kHiddenYawMaxComp = 5;
static const float kHiddenExitTurnDeg = -15.0f;
static const float kHiddenYawDeadbandDeg = 2.0f;
static const uint8_t kTransitionsPerLap = 4U;

static uint8_t g_running = 0;
static uint8_t g_targetLaps = 1U;
static uint8_t g_completedLaps = 0;
static uint8_t g_completedTransitions = 0;
static unsigned long g_hiddenStartMs = 0;
static int g_hiddenEntryLinePos = 0;
static float g_hiddenTargetYaw = 0.0f;
static TrackState g_trackState = TRACK_IDLE;
static SegmentType g_startSegment = SEGMENT_BLACK;
static SegmentType g_currentSegment = SEGMENT_BLACK;
static uint8_t g_allWhiteFiltered = 0;
static uint8_t g_allWhiteLastRaw = 0;
static unsigned long g_allWhiteLastChangeMs = 0;

static AvoidStage g_avoidStage = AVOID_IDLE;
static unsigned long g_avoidStartMs = 0;

static turnStage g_turnStage = TURN_IDLE;

static void Reset_RunState(void);
static void Start_Run(void);
static void Stop_Run(void);
static uint8_t Sensor_IsAllWhiteLocal(void);
static uint8_t Sensor_GetAllWhiteFiltered(void);
static int ClampIntLocal(int value, int minValue, int maxValue);
static uint8_t OnSegmentTransition(SegmentType nextSegment);

extern int BASE_SPEED;
extern int linePos;
extern int lastSumPos;
extern int PWMleft, PWMright;
extern int targetLeftSpeed, targetRightSpeed;
extern int leftEncSpeed, rightEncSpeed;
extern int16_t dist;
extern int is_lost;
extern float yaw;
extern float origin_yaw;
extern MPU6050_Handle gImu;
extern uint8_t g_mpuOk;
extern PID_t leftPID;
extern PID_t rightPID;
extern PID_t steerPID;
extern PID_t anglePID;
extern volatile unsigned long tick_ms;

/* 在系统初始化后调用一次，重置 K0 任务内部状态 */
void K0_RunTask_Init(uint8_t lapCount)
{
    g_targetLaps = (lapCount == 0U) ? 1U : lapCount;
    Reset_RunState();
}

/* 主循环周期调用：处理 K0 启停、循迹和空白区通过 */
void K0_RunTask(void)
{
    uint8_t allWhite = Sensor_GetAllWhiteFiltered();

    /* 运行中持续更新偏航角，隐藏直道会用到 yaw 做修正 */
    if (g_mpuOk && g_running)
    {
        if (MPU6050_UpdateYaw(&gImu, tick_ms))
        {
            yaw = MPU6050_GetYawDeg(&gImu);
        }
    }

    if (Key_GetPressed(kStartKey))
    {
        if (g_running)
            Stop_Run();
        else
            Start_Run();
    }

    if (!g_running)
    {
        Set_PWM(0, 0);
        return;
    }

    /* 正常看到赛道时，直接按红外位置做循迹 */
    if (g_trackState == TRACK_VISIBLE_CURVE)
    {
        if (!allWhite)
        {
            BASE_SPEED = kCurveBaseSpeed;
            linePos = Sensor_GetQuantizedPos();
            PID_control();
        }
        else
        {
            /* 第一次进入全白区，切到隐藏直道状态并记录进入姿态 */
            if (OnSegmentTransition(SEGMENT_WHITE))
            {
                Stop_Run();
                return;
            }
            g_hiddenStartMs = tick_ms;
            g_hiddenEntryLinePos = linePos;
            origin_yaw = yaw;
            g_hiddenTargetYaw = yaw + kHiddenExitTurnDeg;
            g_trackState = TRACK_HIDDEN_STRAIGHT;

            BASE_SPEED = kHiddenBaseSpeed;
            is_lost = 0;
            PID_control();
        }
    }
    else if (g_trackState == TRACK_HIDDEN_STRAIGHT)
    {
        /* 全白区内主要靠进入时的线位置和 yaw 维持方向 */
        unsigned long hiddenElapsedMs = tick_ms - g_hiddenStartMs;
        int yawComp = 0;

        BASE_SPEED = kHiddenBaseSpeed;
        is_lost = 0;

        if (g_mpuOk)
        {
            float yawErr = g_hiddenTargetYaw - yaw;
            if ((yawErr >= -kHiddenYawDeadbandDeg) && (yawErr <= kHiddenYawDeadbandDeg))
            {
                yawErr = 0.0f;
            }
            yawComp += ClampIntLocal((int)(yawErr * kHiddenYawKp), -kHiddenYawMaxComp, kHiddenYawMaxComp);
        }

        if (hiddenElapsedMs < kHiddenEntryFollowMs)
        {
            linePos = (int)((long)g_hiddenEntryLinePos * (long)(kHiddenEntryFollowMs - hiddenElapsedMs) / (long)kHiddenEntryFollowMs) + yawComp;
        }
        else
        {
            linePos = yawComp;
        }

        PID_control();

        if (!allWhite && hiddenElapsedMs >= kHiddenStraightMinMs)
        {
            if (OnSegmentTransition(SEGMENT_BLACK))
            {
                /* 跑完设定空白段数量后自动停车 */
                Stop_Run();
            }
            else
            {
                /* 重新看到线后切回常规循迹 */
                g_trackState = TRACK_VISIBLE_CURVE;
                BASE_SPEED = kCurveBaseSpeed;
                linePos = Sensor_GetQuantizedPos();
                PID_control();
            }
        }
    }
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

static void Reset_RunState(void)
{
    /* 同时重置运行状态、控制器状态和里程/姿态基准 */
    BASE_SPEED = kCurveBaseSpeed;
    linePos = 0;
    lastSumPos = 0;
    is_lost = 0;
    PWMleft = 0;
    PWMright = 0;
    targetLeftSpeed = 0;
    targetRightSpeed = 0;
    leftEncSpeed = 0;
    rightEncSpeed = 0;
    dist = 0;
    yaw = 0.0f;
    origin_yaw = 0.0f;

    g_running = 0;
    g_completedLaps = 0;
    g_completedTransitions = 0;
    g_hiddenStartMs = 0;
    g_hiddenEntryLinePos = 0;
    g_hiddenTargetYaw = 0.0f;
    g_trackState = TRACK_IDLE;
    g_startSegment = SEGMENT_BLACK;
    g_currentSegment = SEGMENT_BLACK;
    g_allWhiteFiltered = 0;
    g_allWhiteLastRaw = 0;
    g_allWhiteLastChangeMs = tick_ms;

    PID_Reset(&leftPID);
    PID_Reset(&rightPID);
    PID_Reset(&steerPID);
    PID_Reset(&anglePID);
    Encoder_ResetDistance();
    if (g_mpuOk)
    {
        MPU6050_ResetYaw(&gImu);
    }
}

static void Start_Run(void)
{
    Reset_RunState();
    if (g_mpuOk)
    {
        /* 起跑前重新校准并清零 yaw，避免上一轮漂移带入 */
        (void)MPU6050_CalibrateGyroZ(&gImu, 150u, 3u);
        MPU6050_ResetYaw(&gImu);
        yaw = 0.0f;
        origin_yaw = 0.0f;
        g_hiddenTargetYaw = 0.0f;
    }
    g_running = 1;
    if (Sensor_GetAllWhiteFiltered())
    {
        g_startSegment = SEGMENT_WHITE;
        g_currentSegment = SEGMENT_WHITE;
        g_hiddenStartMs = tick_ms;
        g_hiddenEntryLinePos = 0;
        origin_yaw = yaw;
        g_hiddenTargetYaw = yaw;
        g_trackState = TRACK_HIDDEN_STRAIGHT;
    }
    else
    {
        g_startSegment = SEGMENT_BLACK;
        g_currentSegment = SEGMENT_BLACK;
        linePos = Sensor_GetQuantizedPos();
        g_trackState = TRACK_VISIBLE_CURVE;
    }
}

static void Stop_Run(void)
{
    Reset_RunState();
    Set_PWM(0, 0);
}

static uint8_t Sensor_IsAllWhiteLocal(void)
{
    for (int i = 0; i < 8; i++)
    {
        if (Sensor_GetState(i) != 1)
            return 0;
    }

    return 1;
}

static uint8_t Sensor_GetAllWhiteFiltered(void)
{
    /* 对“全白”判定做简单消抖，避免状态机在边界来回切换 */
    uint8_t raw = Sensor_IsAllWhiteLocal();

    if (raw != g_allWhiteLastRaw)
    {
        g_allWhiteLastRaw = raw;
        g_allWhiteLastChangeMs = tick_ms;
    }

    if ((tick_ms - g_allWhiteLastChangeMs) >= kIrFilterMs)
    {
        g_allWhiteFiltered = raw;
    }

    return g_allWhiteFiltered;
}

static int ClampIntLocal(int value, int minValue, int maxValue)
{
    if (value < minValue)
        return minValue;
    if (value > maxValue)
        return maxValue;
    return value;
}

static uint8_t OnSegmentTransition(SegmentType nextSegment)
{
    if (nextSegment == g_currentSegment)
    {
        return 0U;
    }

    g_currentSegment = nextSegment;
    g_completedTransitions++;

    if ((g_currentSegment == g_startSegment) &&
        (g_completedTransitions % kTransitionsPerLap == 0U))
    {
        g_completedLaps++;
        if (g_completedLaps >= g_targetLaps)
        {
            return 1U;
        }
    }

    return 0U;
}
