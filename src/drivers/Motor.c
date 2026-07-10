#include "Motor.h"

#define PWM_PERIOD 1000

static int8_t s_leftSign = 0;
static int8_t s_rightSign = 0;

static int8_t Motor_Sign(int value)
{
    return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

void Motor_Init(void)
{
}

/* ---- 电机驱动输出 ----
 * pwm_l/r: -100~+100, 正=前进, 负=后退, 0=刹车。
 * H桥方向控制：AIN1/AIN2 (左轮), BIN1/BIN2 (右轮)。
 * 占空比下限 25/1000，避免低速堵转。 */
void Set_PWM(int pwm_l, int pwm_r)
{
    int8_t leftSign;
    int8_t rightSign;
    uint32_t duty_l;
    uint32_t duty_r;

    if (pwm_l > 100) pwm_l = 100;
    if (pwm_l < -100) pwm_l = -100;
    if (pwm_r > 100) pwm_r = 100;
    if (pwm_r < -100) pwm_r = -100;

    leftSign = Motor_Sign(pwm_l);
    rightSign = Motor_Sign(pwm_r);
    duty_l = (uint32_t)((pwm_l < 0) ? -pwm_l : pwm_l) * PWM_PERIOD / 100U;
    duty_r = (uint32_t)((pwm_r < 0) ? -pwm_r : pwm_r) * PWM_PERIOD / 100U;

    if (leftSign != s_leftSign)
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0U, DL_TIMER_CC_0_INDEX);
    if (rightSign != s_rightSign)
        DL_Timer_setCaptureCompareValue(PWM_0_INST, 0U, DL_TIMER_CC_1_INDEX);
    if ((leftSign != s_leftSign && s_leftSign != 0) ||
        (rightSign != s_rightSign && s_rightSign != 0))
        delay_cycles((CPUCLK_FREQ / 1000000U) * 2U);

    if (duty_l > PWM_PERIOD)
        duty_l = PWM_PERIOD;
    if (duty_r > PWM_PERIOD)
        duty_r = PWM_PERIOD;

    /* 左轮 H桥方向 */
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

    /* 右轮 H桥方向 */
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

    if (duty_l > 0U && duty_l < 100U)
        duty_l = 100;       /* 低速死区补偿 */
    if (duty_r > 0U && duty_r < 100U)
        duty_r = 100;

    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_l, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_r, DL_TIMER_CC_1_INDEX);
    s_leftSign = leftSign;
    s_rightSign = rightSign;
}

void Motor_Stop(void)
{
    Set_PWM(0, 0);
}

void Motor_EmergencyStop(void)
{
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0U, DL_TIMER_CC_1_INDEX);
    DL_GPIO_clearPins(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    s_leftSign = 0;
    s_rightSign = 0;
}
