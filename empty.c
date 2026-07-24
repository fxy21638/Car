#include "main.h"

/* 全局控制状态 */
PID_t leftPID;
PID_t rightPID;
PID_t steerPID;
PID_t anglePID;

int BASE_SPEED = 80;
int g_baseSpeedTarget = 80;
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

/* 四项比赛任务 */
typedef enum
{
    TASK_TRACE = 0,   /* 任务1: 纯循线 */
    TASK_AVOID,       /* 任务2: 循线避障 */
    TASK_DIAG_1,      /* 任务3: 对角线 1圈 */
    TASK_DIAG_4       /* 任务4: 对角线 4圈 */
} TaskType;

static const char *g_taskNames[4] = {
    "Trace",
    "Avoid",
    "Diag 1",
    "Diag 4"
};

typedef enum
{
    MENU_MAIN = 0,
    MENU_TASK_SEL,
    MENU_SET_LAPS,
    MENU_RUNNING,
    MENU_MPU_DEBUG
} MenuState;

static MenuState g_menuState = MENU_MAIN;
static TaskType g_taskType = TASK_TRACE;
uint8_t g_running = 0;
static uint8_t g_targetCircles = 1;
static uint8_t g_lastUserLaps = 1;   /* 任务1/2 的用户设定圈数，切换任务时恢复 */

void OlED_show(void);
void System_Init(void);
static void Handle_Keys(void);
static void TaskSwitchConfig(TaskType task);

int main(void)
{
    SYSCFG_DL_init();
    System_Init();

    while (1)
    {
        /* 读取循迹传感器位置 */
        linePos = Sensor_GetQuantizedPos();

        /* 超声波非阻塞测距（始终运行） */
        Ultrasonic_Task(tick_ms);
        dist = Ultrasonic_GetDistanceCm();

        /* 更新 MPU6050 Yaw（TIMA0 10ms定时触发） */
        if (g_mpu6050_flag)
        {
            MPU6050_UpdateYaw(&gImu, tick_ms);
            yaw = MPU6050_GetYawDeg(&gImu);
            g_mpu6050_flag = 0;
        }

        if (!g_running)
        {
            Handle_Keys();
        }

        if (g_running)
        {
            if (g_taskType == TASK_TRACE)
                PID_control();
            else if (g_taskType == TASK_AVOID)
                ObstacleAvoidance_Task_v2();
            else
                CornerTurn_Task_v2();

            if (g_taskType >= TASK_DIAG_1)
            {
                if (g_cornerTurnsCompleted >= g_targetCircles * 4)
                {
                    g_running = 0;
                    Set_PWM(0, 0);
                }
            }
            else if (Get_Current_Circles() >= g_targetCircles)
            {
                g_running = 0;
                Set_PWM(0, 0);
            }
        }
        else
        {
            Set_PWM(0, 0);
        }

        if (!OLED_IsBusy())
        {
            OlED_show();
        }
        OLED_Task();

        Uart_PollTx();

        //Delay_ms(10);
    }
}

void OlED_show(void)
{
    OLED_Clear();

    if (g_menuState == MENU_MAIN)
    {
        OLED_ShowString(0, 0, "=== MAIN ===", OLED_8X16);
        OLED_ShowString(0, 16, (char *)g_taskNames[g_taskType], OLED_8X16);
        OLED_ShowString(72, 16, "L:", OLED_8X16);
        OLED_ShowSignedNum(88, 16, g_targetCircles, 1, OLED_8X16);
        OLED_ShowString(0, 32, "K2:Sel K3:Lap", OLED_8X16);
        OLED_ShowString(0, 48, "K4:Run", OLED_8X16);
    }
    else if (g_menuState == MENU_TASK_SEL)
    {
        OLED_ShowString(0, 0, "== SEL TASK ==", OLED_8X16);
        OLED_ShowString(0, 16, ">", OLED_8X16);
        OLED_ShowString(16, 16, (char *)g_taskNames[g_taskType], OLED_8X16);
        OLED_ShowString(0, 48, "K2/3:Chg K4:OK", OLED_8X16);
    }
    else if (g_menuState == MENU_SET_LAPS)
    {
        OLED_ShowString(0, 0, "== SET LAPS ==", OLED_8X16);
        OLED_ShowString(0, 16, "Laps:", OLED_8X16);
        OLED_ShowSignedNum(48, 16, g_targetCircles, 2, OLED_8X16);
        OLED_ShowString(0, 32, "K2:+ K3:-", OLED_8X16);
        OLED_ShowString(0, 48, "K4:OK", OLED_8X16);
    }
    else if (g_menuState == MENU_RUNNING)
    {
        OLED_ShowString(0, 0, (char *)g_taskNames[g_taskType], OLED_8X16);

        if (g_taskType <= TASK_AVOID)
        {
            OLED_ShowString(0, 16, "Lap:", OLED_8X16);
            OLED_ShowNum(32, 16, (uint32_t)Get_Current_Circles(), 1, OLED_8X16);
            OLED_ShowString(40, 16, "/", OLED_8X16);
            OLED_ShowNum(48, 16, g_targetCircles, 1, OLED_8X16);
        }

        OLED_ShowString(0, 32, "Spd:", OLED_8X16);
        OLED_ShowNum(32, 32, BASE_SPEED, 2, OLED_8X16);
        OLED_ShowString(56, 32, "dist:", OLED_8X16);
        OLED_ShowSignedNum(88, 32, dist, 4, OLED_8X16);

        OLED_ShowString(0, 48, "K1:Stop", OLED_8X16);
    }
    else if (g_menuState == MENU_MPU_DEBUG)
    {
        OLED_ShowString(0, 0, "== MPU6050 ==", OLED_8X16);
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
        }
        OLED_ShowString(0, 48, "K4:Back", OLED_8X16);
    }

    OLED_Update();
}

/* 任务切换时自动配置圈距和圈数 */
static void TaskSwitchConfig(TaskType task)
{
    g_taskType = task;
    switch (task)
    {
    case TASK_TRACE:
        Encoder_SetPulsesPerCircle(17500);
        g_targetCircles = g_lastUserLaps;
        BASE_SPEED = 80;
        g_baseSpeedTarget = 80;
        break;
    case TASK_AVOID:
        Encoder_SetPulsesPerCircle(19000);
        g_targetCircles = g_lastUserLaps;
        BASE_SPEED = 80;
        g_baseSpeedTarget = 80;
        break;
    case TASK_DIAG_1:
        Encoder_SetPulsesPerCircle(21400);
        g_targetCircles = 1;
        BASE_SPEED = 80;
        g_baseSpeedTarget = 80;
        break;
    case TASK_DIAG_4:
        Encoder_SetPulsesPerCircle(21400);
        g_targetCircles = 4;
        BASE_SPEED = 80;
        g_baseSpeedTarget = 80;
        break;
    }
}

static void Handle_Keys(void)
{
    uint8_t k1 = Key_GetPressed(0);
    uint8_t k2 = Key_GetPressed(1);
    uint8_t k3 = Key_GetPressed(2);
    uint8_t k4 = Key_GetPressed(3);

    /* K1 急停/回主菜单 */
    if (k1)
    {
        g_menuState = MENU_MAIN;
        g_running = 0;
        Set_PWM(0, 0);
        return;
    }

    if (g_menuState == MENU_MAIN)
    {
        if (k2)
        {
            g_menuState = MENU_TASK_SEL;
        }
        else if (k3)
        {
            /* 任务3/4 固定圈数，不进入设置页 */
            if (g_taskType <= TASK_AVOID)
                g_menuState = MENU_SET_LAPS;
        }
        else if (k4)
        {
            Encoder_ResetDistance();
            if (g_taskType == TASK_AVOID)
                ObstacleAvoidance_Task_v2_Reset();
            else if (g_taskType >= TASK_DIAG_1)
                CornerTurn_Task_v2_Reset();
            g_menuState = MENU_RUNNING;
            g_running = 1;
        }
    }
    else if (g_menuState == MENU_TASK_SEL)
    {
        if (k2)
        {
            if (g_taskType > 0)
                TaskSwitchConfig((TaskType)(g_taskType - 1));
            else
                TaskSwitchConfig(TASK_DIAG_4);   /* 循环到头 */
        }
        else if (k3)
        {
            if (g_taskType < TASK_DIAG_4)
                TaskSwitchConfig((TaskType)(g_taskType + 1));
            else
                TaskSwitchConfig(TASK_TRACE);     /* 循环到头 */
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
            if (g_targetCircles > 5)
                g_targetCircles = 5;
            g_lastUserLaps = g_targetCircles;
        }
        else if (k3)
        {
            if (g_targetCircles > 1)
            {
                g_targetCircles--;
                g_lastUserLaps = g_targetCircles;
            }
        }
        else if (k4)
        {
            g_menuState = MENU_MAIN;
        }
    }
    else if (g_menuState == MENU_MPU_DEBUG)
    {
        if (k4)
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
    Uart_TX_Init();
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
    PID_Init(&leftPID, 3.7f, 0.15f, 0.05f, 100, -100, 80, 0.35f);
    PID_Init(&rightPID, 3.3f, 0.15f, 0.05f, 100, -100, 80, 0.35f);
    PID_Init(&steerPID, 0.7f, 0.03f, 0.30f, 80, -80, 40, 0.7f);
    PID_Init(&anglePID, 1.3f, 0.15f, 0.5f, 40, -40, 30, 0.7f);

    /* 默认任务配置，确保Encoder圈距同步 */
    TaskSwitchConfig(TASK_TRACE);
}

