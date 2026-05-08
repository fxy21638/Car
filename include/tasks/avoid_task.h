#ifndef __AVOID_TASK_H
#define __AVOID_TASK_H

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void ObstacleAvoidance_Reset(void);
    void ObstacleAvoidance_Task(unsigned long nowMs);

#ifdef __cplusplus
}
#endif

#endif // __AVOID_TASK_H
