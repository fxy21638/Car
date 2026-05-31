#ifndef __TASK_V2_H
#define __TASK_V2_H

#include "ti_msp_dl_config.h"
#include "Encoder.h"
#include "Motor.h"
#include "MPU6050_MSPM0.h"
#include "PID.h"
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void ObstacleAvoidance_Task_v2(void);
    void ObstacleAvoidance_Task_v2_Reset(void);

    void CornerTurn_Task_v2(void);
    void CornerTurn_Task_v2_Reset(void);

#ifdef __cplusplus
}
#endif

#endif // __TASK_V2_H
