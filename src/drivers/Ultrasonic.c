#include "Ultrasonic.h"

/* ---- SysConfig 生成的宏 ----
 * TIMER_US_INST              → TIMG0
 * TIMER_US_INST_INT_IRQN     → TIMG0_INT_IRQn
 * TIMER_US_INST_IRQHandler   → TIMG0_IRQHandler
 * Ultrasonic_Pins_PORT       → PORTB
 * Ultrasonic_Pins_Trig_Pin_PIN → PB8
 * Ultrasonic_Pins_Echo_Pin_PIN → PB15
 * CPUCLK_FREQ                → 80000000
 */

#define US_PERIOD_MS       (80UL)     /* 测量间隔 */
#define US_TIMEOUT_US      (40000U)   /* 回波超时 (对应 ~7m) */
#define US_OBSTACLE_CM     (20)       /* 障碍判定阈值 */

#define US_TRIG_PORT  (Ultrasonic_Pins_PORT)
#define US_TRIG_PIN   (Ultrasonic_Pins_Trig_Pin_PIN)
#define US_TRIG_IOMUX (Ultrasonic_Pins_Trig_Pin_IOMUX)
#define US_ECHO_PORT  (Ultrasonic_Pins_PORT)
#define US_ECHO_PIN   (Ultrasonic_Pins_Echo_Pin_PIN)
#define US_ECHO_IOMUX (Ultrasonic_Pins_Echo_Pin_IOMUX)

static volatile uint32_t s_echoUs   = 0;   /* 当前回波脉宽 (us) */
static volatile uint32_t s_waitUs   = 0;   /* 等待 Echo 拉高的累积时间 */
static volatile uint8_t  s_busy     = 0;   /* 1=正在测量中 */
static volatile uint8_t  s_echoSeen = 0;   /* 已检测到 Echo 上升 */
volatile int16_t s_distCm = -1;            /* 最近一次有效距离 (cm) */

/* ---- 50us 定时器 ISR --------------------------------------------------
 * TIMER_US 配置为 Periodic 模式，每 50us 触发一次 Zero 中断。
 * ISR 内轮询 Echo 引脚电平，累计高电平时间 (s_echoUs)。
 * 检测到下降沿且 s_echoUs > 0 → 计算距离，清除 s_busy。
 * 超时 (40ms 无有效回波) → s_distCm = -1。 */
void TIMER_US_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMER_US_INST) != DL_TIMER_IIDX_ZERO)
        return;

    if (s_busy)
    {
        s_waitUs += 50U;
        if (DL_GPIO_readPins(US_ECHO_PORT, US_ECHO_PIN) != 0)
        {
            s_echoSeen = 1U;
            s_echoUs += 50;
        }
        else
        {
            if (s_echoSeen != 0U)
            {
                s_distCm = (int16_t)(s_echoUs / 58);  /* us → cm */
                s_busy   = 0;
                DL_TimerG_stopCounter(TIMER_US_INST);
            }
        }
    }

    /* 超时保护：无论 Echo 卡在高还是低，都不会死等 */
    if (s_busy)
    {
        if (s_waitUs >= US_TIMEOUT_US)
        {
            s_distCm = -1;
            s_busy   = 0;
            DL_TimerG_stopCounter(TIMER_US_INST);
        }
    }
}

/* ---- 初始化 ---------------------------------------------------------- */
void Ultrasonic_Init(void)
{
    /* Trig → 推挽输出，初始低电平 */
    DL_GPIO_initDigitalOutputFeatures(US_TRIG_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_clearPins(US_TRIG_PORT, US_TRIG_PIN);
    DL_GPIO_enableOutput(US_TRIG_PORT, US_TRIG_PIN);

    /* Echo → 数字输入 */
    DL_GPIO_initDigitalInputFeatures(US_ECHO_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    s_distCm = -1;
    s_busy   = 0;
    s_echoUs = 0;
    s_waitUs = 0;
    s_echoSeen = 0;

    /* 使能定时器 NVIC 并启动 50us 周期计数。
     * 注：SYSCFG_DL_TIMER_US_init() 已在 System_Init() 之前调用，
     * 完成了时钟、模式、中断使能的配置。这里只需 NVIC + 启动。 */
    DL_TimerG_stopCounter(TIMER_US_INST);
    NVIC_ClearPendingIRQ(TIMER_US_INST_INT_IRQN);
    NVIC_SetPriority(TIMER_US_INST_INT_IRQN, 2);
    NVIC_EnableIRQ(TIMER_US_INST_INT_IRQN);
}

/* ---- 主循环周期性调用 ------------------------------------------------ */
void Ultrasonic_Task(unsigned long nowMs)
{
    static unsigned long s_lastMs = 0;

    if ((unsigned long)(nowMs - s_lastMs) < US_PERIOD_MS)
        return;
    if (s_busy)
        return;

    s_lastMs = nowMs;
    s_echoUs = 0;
    s_waitUs = 0;
    s_echoSeen = 0U;
    s_busy   = 1;

    DL_TimerG_setTimerCount(TIMER_US_INST, TIMER_US_INST_LOAD_VALUE);
    DL_TimerG_startCounter(TIMER_US_INST);

    /* 产生 10us 以上 Trig 高电平脉冲 */
    DL_GPIO_clearPins(US_TRIG_PORT, US_TRIG_PIN);
    DL_GPIO_setPins(US_TRIG_PORT, US_TRIG_PIN);
    delay_cycles((CPUCLK_FREQ / 1000000U) * 12);   /* ~12us */
    DL_GPIO_clearPins(US_TRIG_PORT, US_TRIG_PIN);
}

/* ---- 读取结果 -------------------------------------------------------- */
int16_t Ultrasonic_GetDistanceCm(void)
{
    return s_distCm;
}

bool Ultrasonic_IsObstacle(void)
{
    return (s_distCm > 0 && s_distCm < US_OBSTACLE_CM);
}
