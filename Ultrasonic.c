#include "Ultrasonic.h"

#include "ti_msp_dl_config.h"
#include "Delay.h"

#define ULTRASONIC_TIMEOUT_MS (100)

void Ultrasonic_Init(void)
{
}

int16_t Read_Ultrasonic(void)
{
    unsigned long start, cur;
    int16_t distVal;

    mspm0_get_clock_ms(&start);
    DL_Timer_setTimerCount(TIMER_US_INST, 0);
    DL_Timer_clearInterruptStatus(TIMER_US_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);

    DL_GPIO_setPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Trig_Pin_PIN);
    DL_Common_delayCycles(CPUCLK_FREQ / 100000);
    DL_GPIO_clearPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Trig_Pin_PIN);

    while (!DL_GPIO_readPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Echo_Pin_PIN))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + ULTRASONIC_TIMEOUT_MS))
        {
            return -1; // 超时未收到回波，返回-1表示无效距离
        }
    }

    DL_Timer_startCounter(TIMER_US_INST);

    while (DL_GPIO_readPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Echo_Pin_PIN))
    {
        if (DL_Timer_getRawInterruptStatus(TIMER_US_INST, DL_TIMER_INTERRUPT_LOAD_EVENT))
        {
            DL_Timer_stopCounter(TIMER_US_INST);
            return -2; // 超时等待回波结束，返回-2表示测距异常
        }
    }

    DL_Timer_stopCounter(TIMER_US_INST);
    distVal = (DL_Timer_getTimerCount(TIMER_US_INST) * 0.17);
    return distVal;
}

int is_obstacle(int16_t dist)
{
    if (dist > 0 && dist < 200)
    {
        return 1;
    }
    return 0;
}