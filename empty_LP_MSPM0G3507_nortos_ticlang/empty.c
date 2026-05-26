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

// �ٶ�PID
PID_t leftPID;
PID_t rightPID;

// ѭ��ת��PID
PID_t steerPID;

int BASE_SPEED = 60; // �����ٶ�
int linePos = 0;
int lastSumPos = 0;
int is_lost = 0;
int PWMleft, PWMright;
int targetLeftSpeed, targetRightSpeed;
int leftEncSpeed, rightEncSpeed;
char sensorStr[9];

static MPU6050_Handle gImu;

typedef enum
{
    MENU_MAIN = 0,
    MENU_SET_LAPS,
    MENU_RUNNING,
    MENU_AVOID
} MenuState;

static MenuState g_menuState = MENU_MAIN;
static uint8_t g_avoidEnable = 0;
static uint8_t g_running = 0;
static uint8_t g_targetCircles = 1;

typedef enum
{
    AVOID_IDLE = 0,
    AVOID_STOP,
    AVOID_TURN_LEFT,
    AVOID_FORWARD,
    AVOID_TURN_RIGHT,
} AvoidStage;

static AvoidStage g_avoidStage = AVOID_IDLE;
static unsigned long g_avoidStartMs = 0;

void OlED_show(void);
void System_Init(void);
static void Handle_Keys(void);
static void ObstacleAvoidance_Task(unsigned long nowMs);

int main(void)
{
    // �����ȵ���SysConfig��ʼ��
    SYSCFG_DL_init();

    // ϵͳӲ����ʼ��
    System_Init();

    DL_GPIO_setPins(LEDB_PORT, LEDB_PIN_22_PIN);
    while (1)
    {

        //        // ��ȡѭ��λ��
        linePos = Sensor_GetQuantizedPos();

        /* 每帧更新一次测距数据 */
        Ultrasonic_Task(tick_ms);

        Handle_Keys();

        if (g_running)
        {
            if (Get_Current_Circles() >= g_targetCircles)
            {
                g_running = 0;
                Set_PWM(0, 0); // 跑完目标圈数，停止
            }
            else
            {
                PID_control();
                // if (g_avoidEnable) {
                //     ObstacleAvoidance_Task(tick_ms);
                // }
            }
        }
        else
        {
            Set_PWM(0, 0); // 停止状态
        }

        OlED_show();
        // show_dis();

        // PID_control();
    }
}

void OlED_show(void)
{
    /* 全屏刷新：每次清空显示缓冲区后整屏更新 */
    OLED_Clear();

    if (g_menuState == MENU_MAIN)
    {
        OLED_ShowString(0, 0, "=== MAIN MENU ===", OLED_8X16);
        OLED_ShowString(0, 16, "K2:Set Laps", OLED_8X16);
        OLED_ShowString(0, 32, g_avoidEnable ? "K3:Avoid [ON]" : "K3:Avoid [OFF]", OLED_8X16);
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
        int16_t distanceCm = Ultrasonic_GetDistanceCm();
        OLED_ShowString(0, 0, "=== RUNNING ===", OLED_8X16);
        OLED_ShowString(0, 16, "Dis:", OLED_8X16);
        OLED_ShowSignedNum(40, 16, distanceCm, 4, OLED_8X16);
        OLED_ShowString(0, 32, "Laps:", OLED_8X16);
        OLED_ShowSignedNum(48, 32, g_targetCircles, 2, OLED_8X16);
        OLED_ShowString(0, 48, "K1:Stop", OLED_8X16);
    }
    else if (g_menuState == MENU_AVOID)
    {
        int16_t distanceCm = Ultrasonic_GetDistanceCm();
        OLED_ShowString(0, 0, "=== AVOID ===", OLED_8X16);
        OLED_ShowString(0, 16, g_avoidEnable ? "Status: ON " : "Status: OFF", OLED_8X16);
        OLED_ShowString(0, 32, "Dis:", OLED_8X16);
        OLED_ShowSignedNum(32, 32, distanceCm, 4, OLED_8X16);
        OLED_ShowString(0, 48, "K2/3:Tog K4:Back", OLED_8X16);
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
            g_menuState = MENU_AVOID;
        }
        else if (k4)
        {
            g_menuState = MENU_RUNNING;
            g_running = 1;
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

// Ӳʼ
void System_Init(void)
{
    SysTick_Init();

    // �����ʼ��
    Motor_Init();

    // ѭ����ʼ��
    Sensor_Init();

    // �������ȼ�������ԽС���ȼ�Խ�ߣ�Cortex-M0+ ֧�� 0~3 �� 4 ����
    NVIC_SetPriority(GPIOA_INT_IRQn, 0); // GPIOA ������ȼ�
    NVIC_SetPriority(TIMA0_INT_IRQn, 1); // ��ʱ���θ�

    // ������ܲ����Ĺ����־
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_ClearPendingIRQ(TIMA0_INT_IRQn);

    // ʹ���ж�
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(TIMA0_INT_IRQn);

    // ��������ʼ��
    Encoder_Init();

    Key_Init();
    Ultrasonic_Init();

    // �����ж�
    __enable_irq();

    OLED_Init();

    /* MPU6050: 初始化 + 静止校准（上电保持车不动约 1s） */
    //    (void)MPU6050_Init(&gImu);
    //    (void)MPU6050_CalibrateGyroZ(&gImu, 200u, 5u);

    PID_Init(&leftPID, 1.0f, 0.01f, 3.0f, 80, -80, 60, 0.2f); // 速度环
    PID_Init(&rightPID, 1.0f, 0.01f, 3.0f, 80, -80, 60, 0.2f);
    PID_Init(&steerPID, 0.5f, 0.05f, 0.5f, 70, -70, 80, 0.7f); // 转向环
}

/*
// 避障任务示例，可根据需要取消注释并在主循环调用
static void ObstacleAvoidance_Task(unsigned long nowMs)
{
    int16_t dist = Ultrasonic_GetDistanceCm();

    switch (g_avoidStage)
    {
        case AVOID_IDLE:
            if (dist > 0 && dist < 20)
            {
                g_avoidStage = AVOID_STOP;
                g_avoidStartMs = nowMs;
                Set_PWM(0, 0);
            }
            else
            {
                // PID_control(); // 寻迹直线行驶等
                Set_PWM(BASE_SPEED, BASE_SPEED);
            }
            break;

        case AVOID_STOP:
            if (nowMs - g_avoidStartMs > 500)
            {
                g_avoidStage = AVOID_TURN_LEFT;
                g_avoidStartMs = nowMs;
                Set_PWM(-BASE_SPEED, BASE_SPEED);
            }
            break;

        case AVOID_TURN_LEFT:
            if (nowMs - g_avoidStartMs > 500)
            {
                g_avoidStage = AVOID_FORWARD;
                g_avoidStartMs = nowMs;
                Set_PWM(BASE_SPEED, BASE_SPEED);
            }
            break;

        case AVOID_FORWARD:
            if (nowMs - g_avoidStartMs > 1000)
            {
                g_avoidStage = AVOID_TURN_RIGHT;
                g_avoidStartMs = nowMs;
                Set_PWM(BASE_SPEED, -BASE_SPEED);
            }
            break;

        case AVOID_TURN_RIGHT:
            if (nowMs - g_avoidStartMs > 500)
            {
                g_avoidStage = AVOID_IDLE; // 返回空闲/寻迹模式
                g_avoidStartMs = nowMs;
            }
            break;
    }
}
*/
