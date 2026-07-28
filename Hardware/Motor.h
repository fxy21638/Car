/*
 * ============================================================
 *  motor — TB6612 双路电机驱动硬件抽象
 *
 *  引脚: MOTOR_PORT(PB6=AIN1, PB7=AIN2) — 左电机方向
 *        MOTOR_PORT(PB9=BIN1, PB8=BIN2) — 右电机方向
 *        PA14(PWMA), PB14(PWMB) — TIMG12 CCP0/CCP1
 *  PWM: 80MHz TIMG12, 逻辑范围 ±2000, 物理占空比 = |pwm|/2000
 * ============================================================
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

void Motor_Init(void);
void Motor_Drive(int16_t left_pwm, int16_t right_pwm);
void Motor_ShortBrake(void);

/* 兼容旧接口 */
static inline void Set_PWM(int pwm_l, int pwm_r) {
    Motor_Drive((int16_t)pwm_l * 10, (int16_t)pwm_r * 10);
}
static inline void Motor_Stop(void) { Motor_ShortBrake(); }

#endif
