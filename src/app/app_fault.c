#include "app_fault.h"
#include "Motor.h"

static volatile AppFaultCode s_fault = APP_FAULT_NONE;

void AppFault_Raise(AppFaultCode code)
{
    if (code == APP_FAULT_NONE)
        return;

    Motor_EmergencyStop();
    if (s_fault == APP_FAULT_NONE)
        s_fault = code;
}

void AppFault_Clear(void)
{
    Motor_EmergencyStop();
    s_fault = APP_FAULT_NONE;
}

bool AppFault_IsActive(void)
{
    return s_fault != APP_FAULT_NONE;
}

AppFaultCode AppFault_Get(void)
{
    return s_fault;
}

const char *AppFault_GetText(AppFaultCode code)
{
    switch (code)
    {
    case APP_FAULT_IMU_INIT:       return "IMU INIT";
    case APP_FAULT_IMU_RUNTIME:    return "IMU LOST";
    case APP_FAULT_TASK_TIMEOUT:   return "TASK TIME";
    case APP_FAULT_ENCODER_STALL:  return "ENC STALL";
    case APP_FAULT_WATCHDOG_RESET: return "WDT RESET";
    case APP_FAULT_UNEXPECTED_IRQ: return "BAD IRQ";
    case APP_FAULT_HARDFAULT:      return "HARDFAULT";
    default:                       return "NONE";
    }
}
