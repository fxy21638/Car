/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include "Motor.h"
#include "Sensor.h"
#include "Encoder.h"
#include "PID.h"
#include "Delay.h"
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

static void System_Init(void);

int main(void)
{
    SYSCFG_DL_init();
    System_Init();

    while (1)
    {
        linePos = Sensor_GetQuantizedPos();
        PID_control();
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
    __enable_irq();

    PID_Init(&leftPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);
    PID_Init(&rightPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);
    PID_Init(&steerPID, 0.75f, 0.03f, 0.9f, 80, -80, 60, 0.7f);
    PID_Init(&anglePID, 1.2f, 0.1f, 1.2f, 70, -70, 80, 0.7f);
}
