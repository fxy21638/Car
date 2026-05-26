#include "Ultrasonic.h"

#include "ti_msp_dl_config.h"
#include "Delay.h"

#define ULTRASONIC_TRIG_PORT (Ultrasonic_Pins_PORT)
#define ULTRASONIC_TRIG_PIN (Ultrasonic_Pins_Trig_Pin_PIN)
#define ULTRASONIC_TRIG_IOMUX (Ultrasonic_Pins_Trig_Pin_IOMUX)

#define ULTRASONIC_ECHO_PORT (Ultrasonic_Pins_PORT)
#define ULTRASONIC_ECHO_PIN (Ultrasonic_Pins_Echo_Pin_PIN)
#define ULTRASONIC_ECHO_IOMUX (Ultrasonic_Pins_Echo_Pin_IOMUX)

#define ULTRASONIC_PERIOD_MS (80UL)
#define ULTRASONIC_OBSTACLE_THRESHOLD_CM (20)
#define ULTRASONIC_TIMEOUT_US (40000U)
#define ULTRASONIC_SETTLE_MS (0U)

#define ULTRASONIC_CYCLES_PER_US (CPUCLK_FREQ / 1000000U)

static volatile uint32_t s_echoTimeUs = 0;
static volatile uint32_t s_waitTimeoutUs = 0;
static volatile uint8_t s_measuring = 0;
volatile int16_t s_lastDistanceCm = -1;
static volatile uint32_t s_irqCount = 0;

static inline uint32_t Ultrasonic_Micros(void)
{
    uint32_t ms1 = tick_ms;
    uint32_t val = SysTick->VAL;
    uint32_t ms2 = tick_ms;

    if (ms2 != ms1)
    {
        ms1 = ms2;
        val = SysTick->VAL;
    }

    uint32_t reload = (SysTick->LOAD + 1U);
    uint32_t cyclesIntoMs = (reload - val);
    uint32_t usIntoMs = cyclesIntoMs / ULTRASONIC_CYCLES_PER_US;

    return (ms1 * 1000U) + usIntoMs;
}

static void Ultrasonic_TriggerPulse10us(void)
{
    DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    DL_GPIO_setPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    /* HC-SR04 等模块要求 Trig 高电平 >= 10us；用主频计算避免不同主频下脉冲过短 */
    delay_cycles(10U * ULTRASONIC_CYCLES_PER_US);
    DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
}

void Ultrasonic_Init(void)
{
    DL_GPIO_initDigitalOutputFeatures(ULTRASONIC_TRIG_IOMUX,
                                      DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
                                      DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_clearPins(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);
    DL_GPIO_enableOutput(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN);

    DL_GPIO_initDigitalInputFeatures(ULTRASONIC_ECHO_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    s_lastDistanceCm = -1;
    s_measuring = 0;
    s_echoTimeUs = 0;
    s_waitTimeoutUs = 0;
    s_irqCount = 0;

    NVIC_EnableIRQ(TIMER_US_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_US_INST);
}

void TIMER_US_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_US_INST))
    {
    case DL_TIMER_IIDX_ZERO:
        s_irqCount++;
        if (s_measuring == 1)
        {
            if (DL_GPIO_readPins(ULTRASONIC_ECHO_PORT, ULTRASONIC_ECHO_PIN) > 0)
            {
                s_echoTimeUs += 50;
            }
            else
            {
                if (s_echoTimeUs > 0)
                {
                    s_lastDistanceCm = s_echoTimeUs / 58;
                    s_measuring = 0;
                }
                else
                {
                    s_waitTimeoutUs += 50;
                }
            }

            if (s_echoTimeUs > ULTRASONIC_TIMEOUT_US || s_waitTimeoutUs > ULTRASONIC_TIMEOUT_US)
            {
                s_lastDistanceCm = -1;
                s_measuring = 0;
                s_echoTimeUs = 0;
                s_waitTimeoutUs = 0;
            }
        }
        break;
    default:
        break;
    }
}

void Ultrasonic_Task(unsigned long nowMs)
{
    static unsigned long lastStartMs = 0;

    if ((unsigned long)(nowMs - lastStartMs) < ULTRASONIC_PERIOD_MS)
    {
        return;
    }

    if (s_measuring == 0)
    {
        lastStartMs = nowMs;
        s_echoTimeUs = 0;
        s_waitTimeoutUs = 0;
        s_measuring = 1;
        Ultrasonic_TriggerPulse10us();
    }
}

int16_t Ultrasonic_GetDistanceCm(void)
{
    return s_lastDistanceCm;
}

void Ultrasonic_GetDebugState(int16_t *distanceCm, uint8_t *measuring,
                              uint32_t *echoTimeUs, uint32_t *waitTimeoutUs, uint32_t *irqCount)
{
    if (distanceCm != NULL)
    {
        *distanceCm = s_lastDistanceCm;
    }
    if (measuring != NULL)
    {
        *measuring = s_measuring;
    }
    if (echoTimeUs != NULL)
    {
        *echoTimeUs = s_echoTimeUs;
    }
    if (waitTimeoutUs != NULL)
    {
        *waitTimeoutUs = s_waitTimeoutUs;
    }
    if (irqCount != NULL)
    {
        *irqCount = s_irqCount;
    }
}

bool Ultrasonic_IsObstacle(void)
{
    int16_t d = s_lastDistanceCm;
    return (d > 0 && d < ULTRASONIC_OBSTACLE_THRESHOLD_CM);
}
