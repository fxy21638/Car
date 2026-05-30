#ifndef __TASK_H
#define __TASK_H

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void ObstacleAvoidance_Task(unsigned long nowMs);
    void turn_Task(void);
    void turn_Task_Reset(void);
    void turn_Task_v2(void);
    void turn_Task_v2_Reset(void);

#ifdef __cplusplus
}
#endif

#endif // __TASK_H
