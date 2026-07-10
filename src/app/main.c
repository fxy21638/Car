#include "main.h"
#include "app_config.h"
#include "app_fault.h"
#include "app_scheduler.h"
#include "watchdog.h"

/* 全局控制状态 */
PID_t leftPID;
PID_t rightPID;
PID_t steerPID;
PID_t anglePID;

int BASE_SPEED = APP_BASE_SPEED;
int g_baseSpeedTarget = APP_BASE_SPEED;
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
    MENU_MPU_DEBUG,
    MENU_FAULT
} MenuState;

static MenuState g_menuState = MENU_MAIN;
static TaskType g_taskType = TASK_TRACE;
uint8_t g_running = 0;
static uint8_t g_targetCircles = 1;
static uint8_t g_lastUserLaps = 1;   /* 任务1/2 的用户设定圈数，切换任务时恢复 */
static uint8_t g_imuConsecutiveFailures = 0;
static unsigned long g_stallStartMs = 0;

void OlED_show(void);
void System_Init(void);
static void Handle_Keys(void);
static void TaskSwitchConfig(TaskType task);
static void StopApplication(void);
static void RunControlStep(void);
static uint8_t TaskRequiresImu(TaskType task);

int main(void)
{
    SYSCFG_DL_init();
    System_Init();

    while (1)
    {
        /* 超声波非阻塞测距（始终运行） */
        Ultrasonic_Task(tick_ms);
        dist = Ultrasonic_GetDistanceCm();

        /* 所有状态下都扫描按键，确保运行期间 K1 急停有效。 */
        Handle_Keys();

        if (AppScheduler_TakeControlTick())
        {
            RunControlStep();
        }

        if (AppFault_IsActive())
        {
            g_running = 0;
            g_menuState = MENU_FAULT;
            Motor_EmergencyStop();
        }

        if (!g_running && !AppFault_IsActive())
        {
            Motor_Stop();
        }

        {
            static unsigned long s_lastOledMs = 0;
            if (!OLED_IsBusy() &&
                (unsigned long)(tick_ms - s_lastOledMs) >= APP_OLED_REFRESH_MS)
            {
                s_lastOledMs = tick_ms;
                OlED_show();
            }
        }
        OLED_Task();
		
		//Test_MPU6050_TurnToAngle();

        Uart_PollTx();
        Watchdog_Service(tick_ms, AppScheduler_GetHeartbeat());

        //Delay_ms(10);
    }
}

static uint8_t TaskRequiresImu(TaskType task)
{
    return (task != TASK_TRACE) ? 1U : 0U;
}

static void StopApplication(void)
{
    g_running = 0;
    PWMleft = 0;
    PWMright = 0;
    Motor_EmergencyStop();
    PID_Reset(&leftPID);
    PID_Reset(&rightPID);
    PID_Reset(&steerPID);
    PID_Reset(&anglePID);
    ObstacleAvoidance_Task_v2_Reset();
    CornerTurn_Task_v2_Reset();
    g_stallStartMs = 0;
}

static void RunControlStep(void)
{
    if (gMPU6050_OK)
    {
        if (MPU6050_UpdateYaw(&gImu, tick_ms))
        {
            yaw = MPU6050_GetYawDeg(&gImu);
            g_imuConsecutiveFailures = 0;
        }
        else if (g_imuConsecutiveFailures < 0xFFU)
        {
            g_imuConsecutiveFailures++;
        }
    }

    if (!g_running || AppFault_IsActive())
        return;

    linePos = Sensor_GetQuantizedPos();

    if (TaskRequiresImu(g_taskType) &&
        (!gMPU6050_OK ||
         g_imuConsecutiveFailures >= APP_IMU_MAX_CONSECUTIVE_FAILURES))
    {
        AppFault_Raise(gMPU6050_OK ? APP_FAULT_IMU_RUNTIME : APP_FAULT_IMU_INIT);
        return;
    }

    if (g_taskType == TASK_TRACE)
        PID_control();
    else if (g_taskType == TASK_AVOID)
        ObstacleAvoidance_Task_v2();
    else
        CornerTurn_Task_v2();

    if (AppFault_IsActive())
        return;

    if (g_taskType >= TASK_DIAG_1)
    {
        if (g_cornerTurnsCompleted >= g_targetCircles * 4U)
            StopApplication();
    }
    else if (Get_Current_Circles() >= g_targetCircles)
    {
        StopApplication();
    }

    if (g_running &&
        ((PWMleft >= APP_ENCODER_STALL_MIN_COMMAND || PWMleft <= -APP_ENCODER_STALL_MIN_COMMAND) ||
         (PWMright >= APP_ENCODER_STALL_MIN_COMMAND || PWMright <= -APP_ENCODER_STALL_MIN_COMMAND)) &&
        leftEncSpeed == 0 && rightEncSpeed == 0)
    {
        if (g_stallStartMs == 0)
            g_stallStartMs = tick_ms;
        else if ((unsigned long)(tick_ms - g_stallStartMs) >= APP_ENCODER_STALL_TIMEOUT_MS)
            AppFault_Raise(APP_FAULT_ENCODER_STALL);
    }
    else
    {
        g_stallStartMs = 0;
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
        OLED_ShowString(56, 32, "Dis:", OLED_8X16);
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
    else if (g_menuState == MENU_FAULT)
    {
        OLED_ShowString(0, 0, "=== FAULT ===", OLED_8X16);
        OLED_ShowString(0, 16, (char *)AppFault_GetText(AppFault_Get()), OLED_8X16);
        OLED_ShowString(0, 48, "K4:Clear", OLED_8X16);
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
        Encoder_SetPulsesPerCircle(APP_TRACE_PULSES_PER_CIRCLE);
        g_targetCircles = g_lastUserLaps;
        BASE_SPEED = APP_BASE_SPEED;
        g_baseSpeedTarget = APP_BASE_SPEED;
        break;
    case TASK_AVOID:
        Encoder_SetPulsesPerCircle(APP_AVOID_PULSES_PER_CIRCLE);
        g_targetCircles = g_lastUserLaps;
        BASE_SPEED = APP_BASE_SPEED;
        g_baseSpeedTarget = APP_BASE_SPEED;
        break;
    case TASK_DIAG_1:
        Encoder_SetPulsesPerCircle(APP_DIAG_PULSES_PER_CIRCLE);
        g_targetCircles = 1;
        BASE_SPEED = APP_BASE_SPEED;
        g_baseSpeedTarget = APP_BASE_SPEED;
        break;
    case TASK_DIAG_4:
        Encoder_SetPulsesPerCircle(APP_DIAG_PULSES_PER_CIRCLE);
        g_targetCircles = 4;
        BASE_SPEED = APP_BASE_SPEED;
        g_baseSpeedTarget = APP_BASE_SPEED;
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
        StopApplication();
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
            if (TaskRequiresImu(g_taskType) && !gMPU6050_OK)
            {
                AppFault_Raise(APP_FAULT_IMU_INIT);
                g_menuState = MENU_FAULT;
                return;
            }
            AppFault_Clear();
            BASE_SPEED = g_baseSpeedTarget;
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
    else if (g_menuState == MENU_FAULT)
    {
        if (k4)
        {
            AppFault_Clear();
            StopApplication();
            g_menuState = MENU_MAIN;
        }
    }
}

void System_Init(void)
{
    /* 基础时基与底层模块初始化 */
    SysTick_Init();
    AppScheduler_Init();
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
    PID_Init(&leftPID, 3.7f, 0.30f, 0.05f, 100, -100, 80, 0.35f);
    PID_Init(&rightPID, 3.3f, 0.30f, 0.05f, 100, -100, 80, 0.35f);
    PID_Init(&steerPID, 0.9f, 0.03f, 0.30f, 80, -80, 40, 0.7f);
    PID_Init(&anglePID, 1.3f, 0.15f, 0.5f, 40, -40, 30, 0.7f);

    /* 默认任务配置，确保Encoder圈距同步 */
    TaskSwitchConfig(TASK_TRACE);

    /* 所有阻塞式上电标定完成后再启动看门狗。 */
    Watchdog_Init();
    if (Watchdog_PreviousResetWasWatchdog())
        AppFault_Raise(APP_FAULT_WATCHDOG_RESET);
}

