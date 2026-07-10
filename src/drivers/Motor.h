#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>

void Motor_Init(void);
void Set_PWM(int pwm_l, int pwm_r);
void Motor_Stop(void);
void Motor_EmergencyStop(void);

#endif
