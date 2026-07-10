#ifndef APP_WATCHDOG_H
#define APP_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

void Watchdog_Init(void);
void Watchdog_Service(unsigned long nowMs, uint32_t schedulerHeartbeat);
bool Watchdog_PreviousResetWasWatchdog(void);

#endif
