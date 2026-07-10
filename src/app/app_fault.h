#ifndef APP_FAULT_H
#define APP_FAULT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_FAULT_NONE = 0,
    APP_FAULT_IMU_INIT,
    APP_FAULT_IMU_RUNTIME,
    APP_FAULT_TASK_TIMEOUT,
    APP_FAULT_ENCODER_STALL,
    APP_FAULT_WATCHDOG_RESET,
    APP_FAULT_UNEXPECTED_IRQ,
    APP_FAULT_HARDFAULT
} AppFaultCode;

void AppFault_Raise(AppFaultCode code);
void AppFault_Clear(void);
bool AppFault_IsActive(void);
AppFaultCode AppFault_Get(void);
const char *AppFault_GetText(AppFaultCode code);

#endif
