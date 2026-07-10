#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

void AppScheduler_Init(void);
bool AppScheduler_TakeControlTick(void);
uint32_t AppScheduler_GetHeartbeat(void);

#endif
