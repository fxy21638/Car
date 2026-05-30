/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Delay.h"
#include "Encoder.h"
#include "Key.h"
#include "Motor.h"
#include "MPU6050_MSPM0.h"
#include "OLED.h"
#include "PID.h"
#include "Sensor.h"
#include "task.h"
#include "Uart.h"
#include "Ultrasonic.h"

/* 全局控制状态 */
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
float angle_err;
char sensorStr[9];

MPU6050_Handle gImu;
uint8_t gMPU6050_OK = 0;

typedef enum
{
    MENU_MAIN = 0,
    MENU_SET_LAPS,
    MENU_RUNNING,
    MENU_AVOID,
    MENU_MPU_DEBUG
} MenuState;

static MenuState g_menuState = MENU_MAIN;
static uint8_t g_avoidEnable = 0;
static uint8_t g_turnEnable = 0;
static uint8_t g_running = 0;
static uint8_t g_targetCircles = 1;

void OlED_show(void);
void System_Init(void);
static void Handle_Keys(void);

int main(void)
{
    SYSCFG_DL_init();
    System_Init();

    while (1)
    {
        /* 读取循迹传感器位置 */
         linePos = Sensor_GetQuantizedPos();

        // PID_control();

        /* 更新超声波距离数据（如果避障功能开启） */
        if (g_avoidEnable)
            dist = Read_Ultrasonic();

        /* 更新 MPU6050 Yaw（如果转向功能开启） */
        if (g_turnEnable)
        {
            MPU6050_UpdateYaw(&gImu, tick_ms);
            yaw = MPU6050_GetYawDeg(&gImu);
        }
		if(!g_running)
		{
			Handle_Keys();
		}

        if (g_running)
        {
            if (Get_Current_Circles() >= g_targetCircles)
            {
                g_running = 0;
                Set_PWM(0, 0); /* 达到目标圈数，停止 */
            }
            else
            {
                if (g_avoidEnable)
                {
                    ObstacleAvoidance_Task(tick_ms);
                }
                else if (g_turnEnable)
                {
                    turn_Task();
                }
                else
                {
                    PID_control();
                }
            }
        }
        else
        {
            Set_PWM(0, 0); /* 停止电机 */
        }
		
        if (!OLED_IsBusy())
        {
		    OlED_show();
        }
        OLED_Task();

        //VOFA_SendSpeedLoop();

        //Delay_ms(10); /* 固定循环周期，保证 MPU6050 与 OLED I2C 总线不冲突 */
    }
}

void OlED_show(void)
{
    /* 根据菜单状态刷新 OLED 页面。 */
    OLED_Clear();

    if (g_menuState == MENU_MAIN)
    {
        OLED_ShowString(0, 0, "=== MAIN MENU ===", OLED_8X16);
        OLED_ShowString(0, 16, "K2:Set Laps", OLED_8X16);
        OLED_ShowString(0, 32, "K3:MPU Debug", OLED_8X16);
        OLED_ShowString(0, 48, "K4:Start Run", OLED_8X16);
    }
    else if (g_menuState == MENU_SET_LAPS)
    {
        OLED_ShowString(0, 0, "=== SET LAPS ===", OLED_8X16);
        OLED_ShowString(0, 16, "Laps:", OLED_8X16);
        OLED_ShowSignedNum(48, 16, g_targetCircles, 2, OLED_8X16);
        OLED_ShowString(0, 32, "K2:+ K3:-", OLED_8X16);
        OLED_ShowString(0, 48, "K1/K4:Back", OLED_8X16);
    }
    else if (g_menuState == MENU_RUNNING)
    {
        OLED_ShowString(0, 0, "=== RUNNING ===", OLED_8X16);
        OLED_ShowString(0, 16, "yaw:", OLED_8X16);
        OLED_ShowSignedNum(40, 16, yaw, 6, OLED_8X16);
        OLED_ShowString(0, 32, "yaw_o:", OLED_8X16);
        OLED_ShowSignedNum(48, 32, origin_yaw, 2, OLED_8X16);
        OLED_ShowString(0, 48, "K1:Stop", OLED_8X16);
    }
    else if (g_menuState == MENU_AVOID)
    {
        OLED_ShowString(0, 0, "=== AVOID ===", OLED_8X16);
        OLED_ShowString(0, 16, g_avoidEnable ? "Status: ON " : "Status: OFF",
                        OLED_8X16);
        OLED_ShowString(0, 32, "Dis:", OLED_8X16);
        OLED_ShowSignedNum(32, 32, dist, 4, OLED_8X16);
        OLED_ShowString(0, 48, "K2/3:Tog K4:Back", OLED_8X16);
    }
    else if (g_menuState == MENU_MPU_DEBUG)
    {
        OLED_ShowString(0, 0, "=== MPU6050 ===", OLED_8X16);
        if (!gMPU6050_OK)
        {
            OLED_ShowString(0, 16, "Status: FAIL", OLED_8X16);
            OLED_ShowString(0, 32, "Check I2C", OLED_8X16);
        }
        else
        {
            int16_t yaw_int = (int16_t) yaw;

            OLED_ShowString(0, 16, "Yaw(deg):", OLED_8X16);
            OLED_ShowSignedNum(90, 16, yaw_int, 4, OLED_8X16);
            OLED_ShowString(0, 32,
                            g_turnEnable ? "Status: ON " : "Status: OFF",
                            OLED_8X16);
            OLED_ShowString(0, 48, "K2/3:Tog K4:Back", OLED_8X16);
        }
        OLED_ShowString(0, 48, "K4:Back", OLED_8X16);
    }

    OLED_Update();
}

static void Handle_Keys(void)
{
    uint8_t k1 = Key_GetPressed(0);
    uint8_t k2 = Key_GetPressed(1);
    uint8_t k3 = Key_GetPressed(2);
    uint8_t k4 = Key_GetPressed(3);

    /* K1 统一作为急停 / 返回主菜单。 */
    if (k1)
    {
        g_menuState = MENU_MAIN;
        g_running = 0;
        Set_PWM(0, 0);
    }
    else if (g_menuState == MENU_MAIN)
    {
        if (k2)
        {
            g_menuState = MENU_SET_LAPS;
        }
        else if (k3)
        {
            g_menuState = MENU_MPU_DEBUG;
        }
        else if (k4)
        {
            g_menuState = MENU_RUNNING;
            g_running = 1;
        }
    }
    else if (g_menuState == MENU_MPU_DEBUG)
    {
        if (k2 || k3)
        {
            g_turnEnable = !g_turnEnable;
        }
        else if (k4)
        {
            g_menuState = MENU_MAIN;
        }
    }
    else if (g_menuState == MENU_SET_LAPS)
    {
        if (k2)
        {
            g_targetCircles++;
            if (g_targetCircles > 99)
            {
                g_targetCircles = 99;
            }
        }
        else if (k3)
        {
            if (g_targetCircles > 1)
            {
                g_targetCircles--;
            }
        }
        else if (k4)
        {
            g_menuState = MENU_MAIN;
        }
    }
    else if (g_menuState == MENU_AVOID)
    {
        if (k2 || k3)
        {
            g_avoidEnable = !g_avoidEnable;
        }
        else if (k4)
        {
            g_menuState = MENU_MAIN;
        }
    }
}

void System_Init(void)
{
    /* 基础时基与底层模块初始化 */
    SysTick_Init();
    Motor_Init();
    Sensor_Init();

    /* 中断优先级与使能 */
    NVIC_SetPriority(GPIOA_INT_IRQn, 0);
    NVIC_SetPriority(TIMA0_INT_IRQn, 1);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(TIMA0_INT_IRQn);

    /* 应用层实际使用到的外设初始化 */
    Encoder_Init();
    Key_Init();
    Ultrasonic_Init();
    __enable_irq();
    OLED_Init();

    /* IMU 初始化与零偏标定 */
    if (MPU6050_Init(&gImu))
    {
        gMPU6050_OK = 1;
        gImu.samplePeriodMs = 10;
        if (!MPU6050_CalibrateGyroZ(&gImu, 200u, 5u))
        {
            gMPU6050_OK = 0;
        }
    }
    else
    {
        gMPU6050_OK = 0;
    }

    /* 控制器默认参数 */
    PID_Init(&leftPID, 2.5f, 0.16f, 0.0f, 100, -100, 35, 0.2f);
    PID_Init(&rightPID, 2.8f, 0.16f, 0.0f, 100, -100, 35, 0.2f);
    PID_Init(&steerPID, 1.1f, 0.0f, 0.1f, 80, -80, 20, 0.7f);
    PID_Init(&anglePID, 0.8f, 0.3f, 0.5f, 24, -24, 30, 0.7f);
}
