#include "Motor.h"
#include "ti_msp_dl_config.h"

static void Fault_StopAndReset(void)
{
    __disable_irq();
    Motor_EmergencyStop();
    NVIC_SystemReset();
    for (;;)
    {
    }
}

void HardFault_Handler(void)
{
    Fault_StopAndReset();
}

void Default_Handler(void)
{
    Fault_StopAndReset();
}
