#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

void Motor_Init(void);
void Set_PWM(int pwm_l, int pwm_r);
void Motor_Stop(void);

#endif
