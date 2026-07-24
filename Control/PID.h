#ifndef __PID_H
#define __PID_H

#include "ti_msp_dl_config.h"
#include "robot.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
             float output_max, float output_min, float sum_max, float filter_alpha);
void PID_Update(PID_t *pid);
void PID_Reset(PID_t *pid);

#endif
