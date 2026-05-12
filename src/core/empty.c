/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include "Delay.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "MPU6050_MSPM0.h"
#include "OLED.h"
#include "app_state.h"
#include "PID.h"
#include "Sensor.h"
#include "h24_task.h"
#include "pid_test.h"
#include "Uart.h"
#include "Ultrasonic.h"

PID_t leftPID;
PID_t rightPID;
PID_t steerPID;
PID_t anglePID;

int BASE_SPEED = 60;
int linePos = 0;
int lastSumPos = 0;
int is_lost = 0;
int PWMleft, PWMright;
int targetLeftSpeed, targetRightSpeed;
int leftEncSpeed, rightEncSpeed;
int16_t dist;
float yaw;
float origin_yaw;

MPU6050_Handle gImu;
uint8_t gMPU6050_OK = 0;

void System_Init(void);
static void InitPidControllers(void);

static void InitPidControllers(void)
{
    /*
    PID_Init(&leftPID, 2.0f, 0.0f, 0.0f, 80, -80, 60, 0.2f);
    PID_Init(&rightPID, 2.0f, 0.0f, 0.0f, 80, -80, 60, 0.2f);
    */

    /*
    PID_Init(&leftPID, 1.5f, 0.02f, 0.0f, 80, -80, 60, 0.2f);
    PID_Init(&rightPID, 1.5f, 0.02f, 0.0f, 80, -80, 60, 0.2f);
    */

    PID_Init(&leftPID, 2.8f, 0.16f, 0.0f, 100, -100, 350, 0.2f);
    PID_Init(&rightPID, 2.8f, 0.16f, 0.0f, 100, -100, 350, 0.2f);
}

int main(void)
{
    SYSCFG_DL_init();
    System_Init();

    while (1)
    {
        unsigned long nowMs = tick_ms;

        if (gMPU6050_OK && MPU6050_UpdateYaw(&gImu, nowMs))
        {
            yaw = MPU6050_GetYawDeg(&gImu);
        }

        H24_Task(nowMs);
        /* PIDTest_Task(nowMs); */

        VOFA_SendSpeedLoop();
        Delay_ms(5);
    }
}

void System_Init(void)
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
    Ultrasonic_Init();
    __enable_irq();
    OLED_Init();

    if (MPU6050_Init(&gImu))
    {
        gMPU6050_OK = 1;
        gImu.samplePeriodMs = 50;
        if (!MPU6050_CalibrateGyroZ(&gImu, 200u, 5u))
        {
            gMPU6050_OK = 0;
        }
    }
    else
    {
        gMPU6050_OK = 0;
    }

    InitPidControllers();
    PID_Init(&steerPID, 0.54f, 0.00f, 0.00f, 68, -68, 45, 0.95f);
    PID_Init(&anglePID, 0.55f, 0.00f, 0.08f, 24, -24, 12, 0.7f);
}
