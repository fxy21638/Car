#include "Motor.h"
#include <stdlib.h>

#define PWM_PERIOD 1000

void Motor_Init(void)
{
}

void Set_PWM(int pwm_l, int pwm_r)
{
    // 将百分比（0~100）映射到占空比（0~PWM_PERIOD）
    uint32_t duty_l = abs(pwm_l) * PWM_PERIOD / 100;
    uint32_t duty_r = abs(pwm_r) * PWM_PERIOD / 100;

    // 限幅（防止计算溢出）
    if (duty_l > PWM_PERIOD)
        duty_l = PWM_PERIOD;
    if (duty_r > PWM_PERIOD)
        duty_r = PWM_PERIOD;

    // 方向控制（与之前相同）
    if (pwm_l > 0)
    {
        DL_GPIO_setPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    }
    else if (pwm_l < 0)
    {
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    }

    if (pwm_r > 0)
    {
        DL_GPIO_setPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    }
    else if (pwm_r < 0)
    {
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    }
    if (pwm_l == 0)
        duty_l = 0;
    else if (duty_l < 25)
        duty_l = 25;

    if (pwm_r == 0)
        duty_r = 0;
    else if (duty_r < 25)
        duty_r = 25;

    // 设置 PWM 比较值
    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_l, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_r, DL_TIMER_CC_1_INDEX);
}

void Motor_Stop(void)
{
    Set_PWM(0, 0);
}
