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

    #include "robot.h"
    void ObstacleAvoidance_Task(RobotState *rs, unsigned long nowMs);
    void turn_Task(RobotState *rs);
    void turn_Task_Reset(void);
    void turn_Task_v2(RobotState *rs);
    void turn_Task_v2_Reset(void);
    void diagonal_Task(RobotState *rs);
    void diagonal_Task_Reset(void);

#ifdef __cplusplus
}
#endif

#endif // __TASK_H
