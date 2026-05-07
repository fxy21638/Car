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
#include "task.h"

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
uint8_t g_mpuOk = 0;

static const unsigned long kOledRefreshMs = 300UL;

static void System_Init(void);
static void OLED_ShowDebug(void);

int main(void)
{
    SYSCFG_DL_init();
    System_Init();

    while (1)
    {
        static unsigned long lastOledMs = 0;

        /* 当前 K0 的整套运行逻辑都在 task.c 的这个入口里 */
        K0_RunTask();

        /* OLED 仅做低频调试显示，避免影响主控制频率 */
        if ((tick_ms - lastOledMs) >= kOledRefreshMs || lastOledMs == 0UL)
        {
            lastOledMs = tick_ms;
            OLED_ShowDebug();
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

    if (MPU6050_Init(&gImu))
    {
        gImu.samplePeriodMs = 20u;
        g_mpuOk = MPU6050_CalibrateGyroZ(&gImu, 200u, 3u) ? 1u : 0u;
        yaw = 0.0f;
        origin_yaw = 0.0f;
    }
    else
    {
        g_mpuOk = 0;
    }

    PID_Init(&leftPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);
    PID_Init(&rightPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);
    PID_Init(&steerPID, 1.35f, 0.03f, 0.9f, 80, -80, 60, 0.7f);
    PID_Init(&anglePID, 1.2f, 0.1f, 1.2f, 70, -70, 80, 0.7f);

    /* 让 task.c 接管 K0 对应的运行状态机 */
    K0_RunTask_Init();
}

static void OLED_ShowDebug(void)
{
    OLED_ClearArea(0, 0, 128, 64);

    OLED_ShowString(6, 0, "M:", OLED_6X8);
    OLED_ShowSignedNum(18, 0, g_mpuOk, 1, OLED_6X8);

    OLED_ShowString(6, 16, "Y:", OLED_6X8);
    OLED_ShowSignedNum(18, 16, (int)yaw, 4, OLED_6X8);

    OLED_ShowString(6, 32, "Oy:", OLED_6X8);
    OLED_ShowSignedNum(24, 32, (int)origin_yaw, 4, OLED_6X8);

    OLED_ShowString(6, 48, "Tg:", OLED_6X8);
    OLED_ShowSignedNum(24, 48, targetLeftSpeed, 4, OLED_6X8);

    OLED_Update();
}
