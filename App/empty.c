/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Motor.h"
#include "Sensor.h"
#include "Encoder.h"
#include "OLED.h"
#include "PID.h"
#include "Delay.h"
#include "MPU6050_MSPM0.h"
#include "Key.h"
#include "Ultrasonic.h"
#include "Uart.h"
#include "task.h"

// 全局 PID
PID_t leftPID;
PID_t rightPID;
PID_t steerPID;
PID_t anglePID;

int BASE_SPEED = 60; // 基础速度
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
uint8_t gMPU6050_OK = 0; /* 1 = MPU6050 init success, 0 = failed */

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
    // 初始化 SysConfig
    SYSCFG_DL_init();

    // 系统硬件初始化
    System_Init();

    while (1)
    {

        // 读取循迹传感器位置
        // linePos = Sensor_GetQuantizedPos();

        // PID_control();

        /*
        // 更新超声波距离数据（如果避障功能开启）
        if (g_avoidEnable)
            dist = Read_Ultrasonic();

        // 更新 MPU6050 Yaw（如果转向功能开启）
        if (g_turnEnable)
        {
            MPU6050_UpdateYaw(&gImu, tick_ms);
            yaw = MPU6050_GetYawDeg(&gImu);
        }

        Handle_Keys();

        // Example: test turn to absolute 90 degrees (uncomment to enable)
        // TurnToAngle(90.0f);

        if (g_running)
        {
            if (Get_Current_Circles() >= g_targetCircles)
            {
                g_running = 0;
                Set_PWM(0, 0); // 达到目标圈数，停止
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
                    // TurnToAngle(30);
                }
                else
                {
                    PID_control();
                }
            }
        }
        else
        {
            Set_PWM(0, 0); // 停止电机
        }

        OlED_show();
        */

        // Uart_SendString("car is running\r\n");

        VOFA_SendSpeedLoop();

        Set_PWM(50, 50);
    }
}

void OlED_show(void)
{
    /* OLED 屏幕刷新与菜单显示 */
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
        /*
        OLED_ShowString(0, 16, "Dis:", OLED_8X16);
        OLED_ShowSignedNum(40, 16, dist, 6, OLED_8X16);
        */
        OLED_ShowString(0, 16, "yaw:", OLED_8X16);
        OLED_ShowSignedNum(40, 16, yaw, 6, OLED_8X16);
        /*
        OLED_ShowString(0, 32, "Laps:", OLED_8X16);
        OLED_ShowSignedNum(48, 32, g_targetCircles, 2, OLED_8X16);
        */
        OLED_ShowString(0, 32, "yaw_o:", OLED_8X16);
        OLED_ShowSignedNum(48, 32, origin_yaw, 2, OLED_8X16);
        OLED_ShowString(0, 48, "K1:Stop", OLED_8X16);
    }
    else if (g_menuState == MENU_AVOID)
    {
        OLED_ShowString(0, 0, "=== AVOID ===", OLED_8X16);
        OLED_ShowString(0, 16, g_avoidEnable ? "Status: ON " : "Status: OFF", OLED_8X16);
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
            OLED_ShowString(0, 16, "Yaw(deg):", OLED_8X16);
            int16_t yaw_int = (int16_t)yaw;
            OLED_ShowSignedNum(90, 16, yaw_int, 4, OLED_8X16);
            OLED_ShowString(0, 32, g_turnEnable ? "Status: ON " : "Status: OFF", OLED_8X16);
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

    // K1: Emergency Stop / Back to Main Menu
    if (k1)
    {
        g_menuState = MENU_MAIN;
        g_running = 0;
        Set_PWM(0, 0); // Stop motors
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
                g_targetCircles = 99;
        }
        else if (k3)
        {
            if (g_targetCircles > 1)
                g_targetCircles--;
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

// 硬件初始化
void System_Init(void)
{
    /* 1. 系统时钟初始化 */
    SysTick_Init();

    /* 2. 电机初始化 */
    Motor_Init();

    /* 3. 传感器初始化 */
    Sensor_Init();

    /* 4. NVIC 中断优先级配置 */
    NVIC_SetPriority(GPIOA_INT_IRQn, 0); // GPIOA 中断优先级
    NVIC_SetPriority(TIMA0_INT_IRQn, 1); // 定时器中断优先级

    /* 5. 清除 pending 中断 */
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);

    /* 6. 使能中断 */
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(TIMA0_INT_IRQn);

    /* 7. 编码器初始化 */
    Encoder_Init();

    /* 8. 按键初始化 */
    Key_Init();

    /* 9. 超声波初始化 */
    Ultrasonic_Init();

    /* 10. 全局中断使能 */
    __enable_irq();

    /* 11. OLED 显示初始化 */
    OLED_Init();

    /* 12. MPU6050 IMU 初始化 + 状态校准 */
    if (MPU6050_Init(&gImu))
    {
        gMPU6050_OK = 1;
        gImu.samplePeriodMs = 50; // 采样周期 50ms
        if (!MPU6050_CalibrateGyroZ(&gImu, 200u, 5u))
        {
            gMPU6050_OK = 0; // 校准失败
        }
    }
    else
    {
        gMPU6050_OK = 0; // 初始化失败
    }

    /* 13. PID 控制器初始化 (handle, Kp, Ki, Kd, maxOut, minOut, iMax, deadzone) */
    PID_Init(&leftPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);   // 左轮速度 PID
    PID_Init(&rightPID, 1.5f, 0.01f, 3.0f, 80, -80, 60, 0.2f);  // 右轮速度 PID
    PID_Init(&steerPID, 0.75f, 0.03f, 0.9f, 80, -80, 60, 0.7f); // 转向 PID
    PID_Init(&anglePID, 1.2f, 0.1f, 1.2f, 70, -70, 80, 0.7f);   // 转向 PID
}
