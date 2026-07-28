#include "Motor.h"

#define PWM_PERIOD 1000

void Motor_Init(void)
{
}

/* ---- 电机驱动输出 ----
 * pwm_l/r: -100~+100, 正=前进, 负=后退, 0=刹车。
 * H桥方向控制：AIN1/AIN2 (左轮), BIN1/BIN2 (右轮)。
 * 占空比下限 25/1000，避免低速堵转。 */
void Set_PWM(int pwm_l, int pwm_r)
{
    uint32_t duty_l = abs(pwm_l) * PWM_PERIOD / 100;
    uint32_t duty_r = abs(pwm_r) * PWM_PERIOD / 100;

    if (duty_l > PWM_PERIOD)
        duty_l = PWM_PERIOD;
    if (duty_r > PWM_PERIOD)	
        duty_r = PWM_PERIOD;

    /* 左轮 H桥方向 */
    if (pwm_l > 0)
    {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN2_PIN);
    }
    else if (pwm_l < 0)
    {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    }

    /* 右轮 H桥方向 */
    if (pwm_r > 0)
    {
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN2_PIN);
    }
    else if (pwm_r < 0)
    {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN);
        DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
    }
    else
    {
        DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    }

    if (duty_l < 100)
        duty_l = 100;       /* 低速死区补偿 */
    if (duty_r < 100)
        duty_r = 100;

    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_l, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_r, DL_TIMER_CC_1_INDEX);
}

void Motor_Stop(void)
{
    Set_PWM(0, 0);
}
