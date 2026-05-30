#ifndef __TASK_H
#define __TASK_H

#include "ti_msp_dl_config.h"
#include "Delay.h"
#include "Motor.h"
#include "MPU6050_MSPM0.h"
#include "PID.h"
#include "Ultrasonic.h"
#include <math.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void ObstacleAvoidance_Task(unsigned long nowMs);
    void turn_Task(void);
    void turn_Task_Reset(void);
    void diagonal_Task(void);
    void diagonal_Task_Reset(void);

#ifdef __cplusplus
}
#endif

#endif // __TASK_H
