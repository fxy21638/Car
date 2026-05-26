#include "ti_msp_dl_config.h"
#include "Delay.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;

void SysTick_Handler(void)
{
    tick_ms++;
}

int Delay_ms(unsigned long num_ms)
{
    start_time = tick_ms;
    while (tick_ms - start_time < num_ms)
        ;
    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}

void SysTick_Init(void)
{
    tick_ms = 0;
    (void)SysTick_Config(CPUCLK_FREQ / 1000U);
    NVIC_SetPriority(SysTick_IRQn, 0);
}
