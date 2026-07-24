#ifndef __TASK_V2_H
#define __TASK_V2_H

#include "ti_msp_dl_config.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include <math.h>

#include "robot.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void ObstacleAvoidance_Task_v2(RobotState *rs);
    void ObstacleAvoidance_Task_v2_Reset(void);

    void CornerTurn_Task_v2(RobotState *rs);
    void CornerTurn_Task_v2_Reset(void);
    extern uint8_t g_cornerTurnsCompleted;

#ifdef __cplusplus
}
#endif

#endif // __TASK_V2_H
