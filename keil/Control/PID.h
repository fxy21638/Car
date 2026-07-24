#ifndef __PID_H
#define __PID_H

#include "ti_msp_dl_config.h"
#include "robot.h"

// 函数声明
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float output_max, float output_min, float sum_max, float filter_alpha);
void PID_Update(PID_t *pid);
void PID_Reset(PID_t *pid);

void PID_control(RobotState *rs);
void PID_control_head(RobotState *rs, int speed, float targetYawDeg);
void PID_SetSpeedTunings(RobotState *rs, float kp, float ki, float kd);
float Angle_Control(RobotState *rs, float targetYawDeg, float actualYawDeg);
void TurnToAngle(RobotState *rs, float targetYawDeg);
float Angle_Normalize(float angle);

#endif
