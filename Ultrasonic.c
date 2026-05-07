#include "Ultrasonic.h"

#include "ti_msp_dl_config.h"
#include "Delay.h"

/* 最长等待时间，避免测距异常时卡死主循环。 */
#define ULTRASONIC_TIMEOUT_MS (100)

void Ultrasonic_Init(void)
{
    /* GPIO 和定时器底层初始化已由 SysConfig 完成。
     * 这里保留为空函数，便于后续增加模块自检。 */
}

int16_t Read_Ultrasonic(void)
{
    unsigned long start, cur;
    int16_t distVal;

    /* 记录本轮测距开始时间，并清零计时器状态。 */
    mspm0_get_clock_ms(&start);
    DL_Timer_setTimerCount(TIMER_US_INST, 0);
    DL_Timer_clearInterruptStatus(TIMER_US_INST, DL_TIMER_INTERRUPT_LOAD_EVENT);

    /* 发送触发脉冲，启动超声波模块发射。 */
    DL_GPIO_setPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Trig_Pin_PIN);
    DL_Common_delayCycles(CPUCLK_FREQ / 100000);
    DL_GPIO_clearPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Trig_Pin_PIN);

    /* 等待 Echo 拉高，表示开始收到回波窗口。 */
    while (!DL_GPIO_readPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Echo_Pin_PIN))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + ULTRASONIC_TIMEOUT_MS))
        {
            /* -1 表示在限定时间内没有等到回波开始。 */
            return -1;
        }
    }

    /* Echo 拉高后开始计时。 */
    DL_Timer_startCounter(TIMER_US_INST);

    /* 等待 Echo 重新拉低，表示回波结束。 */
    while (DL_GPIO_readPins(Ultrasonic_Pins_PORT, Ultrasonic_Pins_Echo_Pin_PIN))
    {
        if (DL_Timer_getRawInterruptStatus(TIMER_US_INST, DL_TIMER_INTERRUPT_LOAD_EVENT))
        {
            DL_Timer_stopCounter(TIMER_US_INST);
            /* -2 表示回波持续时间过长，通常为超量程或异常波形。 */
            return -2;
        }
    }

    DL_Timer_stopCounter(TIMER_US_INST);

    /* 当前项目使用经验系数 0.17 把计数值换算为距离。 */
    distVal = (DL_Timer_getTimerCount(TIMER_US_INST) * 0.17);
    return distVal;
}

int is_obstacle(int16_t dist)
{
    /* 简单阈值判定：
     * 有效距离在 0~200 范围内时，视为前方有障碍物。 */
    if (dist > 0 && dist < 200)
    {
        return 1;
    }
    return 0;
}
