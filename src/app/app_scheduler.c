#include "app_scheduler.h"
#include "Encoder.h"
#include "ti_msp_dl_config.h"

static volatile uint8_t s_controlPending = 0;
static volatile uint32_t s_heartbeat = 0;

void AppScheduler_Init(void)
{
    s_controlPending = 0;
    s_heartbeat = 0;
}

bool AppScheduler_TakeControlTick(void)
{
    uint32_t primask = __get_PRIMASK();
    bool pending;

    __disable_irq();
    pending = (s_controlPending != 0U);
    s_controlPending = 0U;
    __set_PRIMASK(primask);
    return pending;
}

uint32_t AppScheduler_GetHeartbeat(void)
{
    return s_heartbeat;
}

void TIMA0_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        Encoder_SampleSpeed10msFromISR();
        s_controlPending = 1U;
        s_heartbeat++;
    }

    DL_TimerA_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
}
