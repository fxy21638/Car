/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "Motor.h"
#include "Sensor.h"
#include "Encoder.h"
#include "PID.h"
#include "Delay.h"
#include "Key.h"
#include "OLED.h"
#include "MPU6050_MSPM0.h"

PID_t leftPID;
PID_t rightPID;
PID_t steerPID;
PID_t anglePID;

int BASE_SPEED = 60;
int linePos = 0;
int lastSumPos = 0;
int is_lost = 0;
int PWMleft = 0;
int PWMright = 0;
int targetLeftSpeed = 0;
int targetRightSpeed = 0;
int leftEncSpeed = 0;
int rightEncSpeed = 0;
int16_t dist = 0;
float yaw = 0.0f;
float origin_yaw = 0.0f;
MPU6050_Handle gImu;

typedef enum
{
    TRACK_IDLE = 0,
    TRACK_VISIBLE_CURVE,
    TRACK_HIDDEN_STRAIGHT
} TrackState;

static const uint8_t kStartKey = 0;
static const int kCurveBaseSpeed = 52;
static const int kHiddenBaseSpeed = 48;
static const int kHiddenRightTrend = -3;
static const uint8_t kHiddenSegmentsPerLap = 2;
static const unsigned long kOledRefreshMs = 300UL;
static const unsigned long kHiddenStraightMinMs = 120UL;
static const unsigned long kHiddenEntryFollowMs = 80UL;
static const unsigned long kHiddenTrendMs = 140UL;
static const unsigned long kIrFilterMs = 20UL;

static uint8_t g_running = 0;
static uint8_t g_hiddenSegments = 0;
static unsigned long g_hiddenStartMs = 0;
static int g_hiddenEntryLinePos = 0;
static TrackState g_trackState = TRACK_IDLE;
static uint8_t g_allWhiteRaw = 0;
static uint8_t g_allWhiteFiltered = 0;
static uint8_t g_allWhiteLastRaw = 0;
static unsigned long g_allWhiteLastChangeMs = 0;

static void System_Init(void);
static void Reset_RunState(void);
static void Start_Run(void);
static void Stop_Run(void);
static uint8_t Sensor_IsAllWhiteLocal(void);
static uint8_t Sensor_GetAllWhiteFiltered(void);
static void OLED_ShowDebug(uint8_t allWhite);

int main(void)
{
    SYSCFG_DL_init();
    System_Init();

    while (1)
    {
        static unsigned long lastOledMs = 0;
        uint8_t allWhite = Sensor_GetAllWhiteFiltered();

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
            if ((tick_ms - lastOledMs) >= kOledRefreshMs || lastOledMs == 0UL)
            {
                lastOledMs = tick_ms;
                OLED_ShowDebug(allWhite);
            }
            continue;
        }

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
                g_hiddenSegments++;
                g_hiddenStartMs = tick_ms;
                g_hiddenEntryLinePos = linePos;
                g_trackState = TRACK_HIDDEN_STRAIGHT;

                BASE_SPEED = kHiddenBaseSpeed;
                is_lost = 0;
                PID_control();
            }
        }
        else if (g_trackState == TRACK_HIDDEN_STRAIGHT)
        {
            unsigned long hiddenElapsedMs = tick_ms - g_hiddenStartMs;
            int hiddenTrend = (hiddenElapsedMs < kHiddenTrendMs) ? kHiddenRightTrend : 0;

            BASE_SPEED = kHiddenBaseSpeed;
            is_lost = 0;

            if (hiddenElapsedMs < kHiddenEntryFollowMs)
            {
                linePos = (int)((long)g_hiddenEntryLinePos * (long)(kHiddenEntryFollowMs - hiddenElapsedMs) / (long)kHiddenEntryFollowMs) + hiddenTrend;
            }
            else
            {
                linePos = hiddenTrend;
            }

            PID_control();

            if (!allWhite && (tick_ms - g_hiddenStartMs) >= kHiddenStraightMinMs)
            {
                if (g_hiddenSegments >= kHiddenSegmentsPerLap)
                {
                    Stop_Run();
                }
                else
                {
                    g_trackState = TRACK_VISIBLE_CURVE;
                    BASE_SPEED = kCurveBaseSpeed;
                    linePos = Sensor_GetQuantizedPos();
                    PID_control();
                }
            }
        }

        if ((tick_ms - lastOledMs) >= kOledRefreshMs || lastOledMs == 0UL)
        {
            lastOledMs = tick_ms;
            OLED_ShowDebug(allWhite);
        }
    }
}

static void System_Init(void)
{
    SysTick_Init();
    Motor_Init();
    Sensor_Init();

    NVIC_SetPriority(GPIOA_INT_IRQn, 0);
    NVIC_SetPriority(TIMA0_INT_IRQn, 1);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(TIMA0_INT_IRQn);

    Encoder_Init();
    Key_Init();
    __enable_irq();
    OLED_Init();

    PID_Init(&leftPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);
    PID_Init(&rightPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);
    PID_Init(&steerPID, 1.10f, 0.03f, 0.9f, 80, -80, 60, 0.7f);
    PID_Init(&anglePID, 1.2f, 0.1f, 1.2f, 70, -70, 80, 0.7f);
    Reset_RunState();
}

static void Reset_RunState(void)
{
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
    g_hiddenSegments = 0;
    g_hiddenStartMs = 0;
    g_hiddenEntryLinePos = 0;
    g_trackState = TRACK_IDLE;

    PID_Reset(&leftPID);
    PID_Reset(&rightPID);
    PID_Reset(&steerPID);
    PID_Reset(&anglePID);
    Encoder_ResetDistance();
}

static void Start_Run(void)
{
    Reset_RunState();
    g_running = 1;
    g_trackState = TRACK_VISIBLE_CURVE;
    OLED_ShowDebug(Sensor_IsAllWhiteLocal());
}

static void Stop_Run(void)
{
    Reset_RunState();
    Set_PWM(0, 0);
    OLED_ShowDebug(Sensor_IsAllWhiteLocal());
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
    uint8_t raw = Sensor_IsAllWhiteLocal();

    g_allWhiteRaw = raw;
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

static void OLED_ShowDebug(uint8_t allWhite)
{
    OLED_ClearArea(0, 0, 128, 64);
    OLED_ShowString(0, 0, "S:", OLED_6X8);
    OLED_ShowSignedNum(12, 0, g_trackState, 1, OLED_6X8);
    OLED_ShowString(24, 0, "R:", OLED_6X8);
    OLED_ShowSignedNum(36, 0, g_running, 1, OLED_6X8);
    OLED_ShowString(48, 0, "W:", OLED_6X8);
    OLED_ShowSignedNum(60, 0, allWhite, 1, OLED_6X8);
    OLED_ShowString(72, 0, "Rw:", OLED_6X8);
    OLED_ShowSignedNum(90, 0, g_allWhiteRaw, 1, OLED_6X8);

    OLED_ShowString(0, 16, "G:", OLED_6X8);
    OLED_ShowSignedNum(12, 16, g_hiddenSegments, 1, OLED_6X8);
    OLED_ShowString(24, 16, "Fms:", OLED_6X8);
    OLED_ShowSignedNum(48, 16, (int)(tick_ms - g_allWhiteLastChangeMs), 4, OLED_6X8);

    OLED_ShowString(0, 32, "Tms:", OLED_6X8);
    OLED_ShowSignedNum(24, 32, (int)(tick_ms - g_hiddenStartMs), 4, OLED_6X8);
    OLED_ShowString(60, 32, "B:", OLED_6X8);
    OLED_ShowSignedNum(72, 32, BASE_SPEED, 3, OLED_6X8);

    OLED_ShowString(0, 48, "L:", OLED_6X8);
    OLED_ShowSignedNum(12, 48, leftEncSpeed, 4, OLED_6X8);
    OLED_ShowString(48, 48, "Tr:", OLED_6X8);
    OLED_ShowSignedNum(66, 48, (tick_ms - g_hiddenStartMs) < kHiddenTrendMs ? kHiddenRightTrend : 0, 2, OLED_6X8);

    OLED_Update();
}
