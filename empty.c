#include "main.h"

/* 所有模块间共享的状态收敛到单一结构体，通过指针传递，消除 extern */
static RobotState g_rs;

static const char *g_taskNames[4] = {
    "Trace",
    "Avoid",
    "Diag 1",
    "Diag 4"
};

void OlED_show(RobotState *rs);
void System_Init(RobotState *rs);
static void Handle_Keys(RobotState *rs);
static void TaskSwitchConfig(RobotState *rs, TaskType task);

int main(void)
{
    SYSCFG_DL_init();
    System_Init(&g_rs);

    while (1)
    {
        /* 读取循迹传感器位置 */
        g_rs.linePos = Sensor_GetQuantizedPos(&g_rs);

        /* 超声波非阻塞测距（始终运行） */
        Ultrasonic_Task(tick_ms);
        g_rs.dist = Ultrasonic_GetDistanceCm();

        /* 更新 MPU6050 Yaw（TIMA0 10ms定时触发） */
        if (g_mpu6050_flag)
        {
            MPU6050_UpdateYaw(&g_rs.imu, tick_ms);
            g_rs.yaw = MPU6050_GetYawDeg(&g_rs.imu);
            g_mpu6050_flag = 0;
        }

        if (!g_rs.running)
        {
            Handle_Keys(&g_rs);
        }

        if (g_rs.running)
        {
            if (g_rs.taskType == TASK_TRACE)
                PID_control(&g_rs);
            else if (g_rs.taskType == TASK_AVOID)
                ObstacleAvoidance_Task_v2(&g_rs);
            else
                CornerTurn_Task_v2(&g_rs);

            if (g_rs.taskType >= TASK_DIAG_1)
            {
                if (g_cornerTurnsCompleted >= g_rs.targetCircles * 4)
                {
                    g_rs.running = 0;
                    Set_PWM(0, 0);
                }
            }
            else if (Get_Current_Circles() >= g_rs.targetCircles)
            {
                g_rs.running = 0;
                Set_PWM(0, 0);
            }
        }
        else
        {
            Set_PWM(0, 0);
        }

        if (!OLED_IsBusy())
        {
            OlED_show(&g_rs);
        }
        OLED_Task();

        Uart_PollTx();

        //Delay_ms(10);
    }
}

void OlED_show(RobotState *rs)
{
    OLED_Clear();

    if (rs->menuState == MENU_MAIN)
    {
        OLED_ShowString(0, 0, "=== MAIN ===", OLED_8X16);
        OLED_ShowString(0, 16, (char *)g_taskNames[rs->taskType], OLED_8X16);
        OLED_ShowString(72, 16, "L:", OLED_8X16);
        OLED_ShowSignedNum(88, 16, rs->targetCircles, 1, OLED_8X16);
        OLED_ShowString(0, 32, "K2:Sel K3:Lap", OLED_8X16);
        OLED_ShowString(0, 48, "K4:Run", OLED_8X16);
    }
    else if (rs->menuState == MENU_TASK_SEL)
    {
        OLED_ShowString(0, 0, "== SEL TASK ==", OLED_8X16);
        OLED_ShowString(0, 16, ">", OLED_8X16);
        OLED_ShowString(16, 16, (char *)g_taskNames[rs->taskType], OLED_8X16);
        OLED_ShowString(0, 48, "K2/3:Chg K4:OK", OLED_8X16);
    }
    else if (rs->menuState == MENU_SET_LAPS)
    {
        OLED_ShowString(0, 0, "== SET LAPS ==", OLED_8X16);
        OLED_ShowString(0, 16, "Laps:", OLED_8X16);
        OLED_ShowSignedNum(48, 16, rs->targetCircles, 2, OLED_8X16);
        OLED_ShowString(0, 32, "K2:+ K3:-", OLED_8X16);
        OLED_ShowString(0, 48, "K4:OK", OLED_8X16);
    }
    else if (rs->menuState == MENU_RUNNING)
    {
        OLED_ShowString(0, 0, (char *)g_taskNames[rs->taskType], OLED_8X16);

        if (rs->taskType <= TASK_AVOID)
        {
            OLED_ShowString(0, 16, "Lap:", OLED_8X16);
            OLED_ShowNum(32, 16, (uint32_t)Get_Current_Circles(), 1, OLED_8X16);
            OLED_ShowString(40, 16, "/", OLED_8X16);
            OLED_ShowNum(48, 16, rs->targetCircles, 1, OLED_8X16);
        }

        OLED_ShowString(0, 32, "Spd:", OLED_8X16);
        OLED_ShowNum(32, 32, rs->baseSpeed, 2, OLED_8X16);
        OLED_ShowString(56, 32, "cm:", OLED_8X16);
        OLED_ShowSignedNum(80, 32, rs->dist, 4, OLED_8X16);

        OLED_ShowString(0, 48, "L:", OLED_8X16);
        OLED_ShowSignedNum(16, 48, Encoder_GetLeftSpeed(), 4, OLED_8X16);
        OLED_ShowString(56, 48, "R:", OLED_8X16);
        OLED_ShowSignedNum(72, 48, Encoder_GetRightSpeed(), 4, OLED_8X16);
    }
    else if (rs->menuState == MENU_MPU_DEBUG)
    {
        OLED_ShowString(0, 0, "== MPU6050 ==", OLED_8X16);
        if (!rs->imuOk)
        {
            OLED_ShowString(0, 16, "Status: FAIL", OLED_8X16);
            OLED_ShowString(0, 32, "Check I2C", OLED_8X16);
        }
        else
        {
            int16_t yaw_int = (int16_t)rs->yaw;
            OLED_ShowString(0, 16, "Yaw(deg):", OLED_8X16);
            OLED_ShowSignedNum(90, 16, yaw_int, 4, OLED_8X16);
        }
        OLED_ShowString(0, 48, "K4:Back", OLED_8X16);
    }

    OLED_Update();
}

/* 任务切换时自动配置圈距和圈数 */
static void TaskSwitchConfig(RobotState *rs, TaskType task)
{
    rs->taskType = task;
    switch (task)
    {
    case TASK_TRACE:
        Encoder_SetPulsesPerCircle(17500);
        rs->targetCircles = rs->lastUserLaps;
        rs->baseSpeed = 80;
        rs->baseSpeedTarget = 80;
        break;
    case TASK_AVOID:
        Encoder_SetPulsesPerCircle(19000);
        rs->targetCircles = rs->lastUserLaps;
        rs->baseSpeed = 80;
        rs->baseSpeedTarget = 80;
        break;
    case TASK_DIAG_1:
        Encoder_SetPulsesPerCircle(21400);
        rs->targetCircles = 1;
        rs->baseSpeed = 80;
        rs->baseSpeedTarget = 80;
        break;
    case TASK_DIAG_4:
        Encoder_SetPulsesPerCircle(21400);
        rs->targetCircles = 4;
        rs->baseSpeed = 80;
        rs->baseSpeedTarget = 80;
        break;
    }
}

static void Handle_Keys(RobotState *rs)
{
    uint8_t k1 = Key_GetPressed(0);
    uint8_t k2 = Key_GetPressed(1);
    uint8_t k3 = Key_GetPressed(2);
    uint8_t k4 = Key_GetPressed(3);

    /* K1 急停/回主菜单 */
    if (k1)
    {
        rs->menuState = MENU_MAIN;
        rs->running = 0;
        Set_PWM(0, 0);
        return;
    }

    if (rs->menuState == MENU_MAIN)
    {
        if (k2)
        {
            rs->menuState = MENU_TASK_SEL;
        }
        else if (k3)
        {
            if (rs->taskType <= TASK_AVOID)
                rs->menuState = MENU_SET_LAPS;
        }
        else if (k4)
        {
            Encoder_ResetDistance();
            if (rs->taskType == TASK_AVOID)
                ObstacleAvoidance_Task_v2_Reset();
            else if (rs->taskType >= TASK_DIAG_1)
                CornerTurn_Task_v2_Reset();
            rs->menuState = MENU_RUNNING;
            rs->running = 1;
        }
    }
    else if (rs->menuState == MENU_TASK_SEL)
    {
        if (k2)
        {
            if (rs->taskType > 0)
                TaskSwitchConfig(rs, (TaskType)(rs->taskType - 1));
            else
                TaskSwitchConfig(rs, TASK_DIAG_4);
        }
        else if (k3)
        {
            if (rs->taskType < TASK_DIAG_4)
                TaskSwitchConfig(rs, (TaskType)(rs->taskType + 1));
            else
                TaskSwitchConfig(rs, TASK_TRACE);
        }
        else if (k4)
        {
            rs->menuState = MENU_MAIN;
        }
    }
    else if (rs->menuState == MENU_SET_LAPS)
    {
        if (k2)
        {
            rs->targetCircles++;
            if (rs->targetCircles > 5)
                rs->targetCircles = 5;
            rs->lastUserLaps = rs->targetCircles;
        }
        else if (k3)
        {
            if (rs->targetCircles > 1)
            {
                rs->targetCircles--;
                rs->lastUserLaps = rs->targetCircles;
            }
        }
        else if (k4)
        {
            rs->menuState = MENU_MAIN;
        }
    }
    else if (rs->menuState == MENU_MPU_DEBUG)
    {
        if (k4)
        {
            rs->menuState = MENU_MAIN;
        }
    }
}

void System_Init(RobotState *rs)
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
    Uart_TX_Init();
    OLED_Init();

    /* 初始化 RobotState */
    rs->baseSpeed = 80;
    rs->baseSpeedTarget = 80;
    rs->linePos = 0;
    rs->lastSumPos = 0;
    rs->isLost = 0;
    rs->yaw = 0.0f;
    rs->originYaw = 0.0f;
    rs->dist = -1;
    rs->running = 0;
    rs->menuState = MENU_MAIN;
    rs->taskType = TASK_TRACE;
    rs->targetCircles = 1;
    rs->lastUserLaps = 1;

    /* IMU 初始化与零偏标定 */
    if (MPU6050_Init(&rs->imu))
    {
        rs->imuOk = 1;
        rs->imu.samplePeriodMs = 10;
        if (!MPU6050_CalibrateGyroZ(&rs->imu, 200u, 5u))
        {
            rs->imuOk = 0;
        }
    }
    else
    {
        rs->imuOk = 0;
    }

    /* 控制器默认参数 */
    PID_Init(&rs->leftPID, 3.7f, 0.15f, 0.05f, 100, -100, 80, 0.35f);
    PID_Init(&rs->rightPID, 3.3f, 0.15f, 0.05f, 100, -100, 80, 0.35f);
    PID_Init(&rs->steerPID, 0.7f, 0.03f, 0.30f, 80, -80, 40, 0.7f);
    PID_Init(&rs->anglePID, 1.3f, 0.15f, 0.5f, 40, -40, 30, 0.7f);

    /* 默认任务配置，确保Encoder圈距同步 */
    TaskSwitchConfig(rs, TASK_TRACE);
}
