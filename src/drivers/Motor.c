#include "Motor.h"
#include <stdlib.h>

/* PWM_PERIOD:
 * 与 SysConfig 中 PWM 定时器的周期保持一致。
 * 当前使用 1000，意味着比较值范围为 0~1000。 */
#define PWM_PERIOD 1000

void Motor_Init(void)
{
    /* 电机方向引脚和 PWM 外设已经由 SysConfig 初始化。
     * 这里暂时不额外做配置，保留空函数作为统一入口。 */
}

void Set_PWM(int pwm_l, int pwm_r)
{
    /* 输入约定：
     * pwm_l / pwm_r 取值通常在 -100~100。
     * 正负号表示方向，绝对值表示占空比百分比。 */
    uint32_t duty_l = abs(pwm_l) * PWM_PERIOD / 100;
    uint32_t duty_r = abs(pwm_r) * PWM_PERIOD / 100;

    /* 先做上限保护，避免异常参数越界。 */
    if (duty_l > PWM_PERIOD)
        duty_l = PWM_PERIOD;
    if (duty_r > PWM_PERIOD)
        duty_r = PWM_PERIOD;

    /* 左轮方向控制：
     * 正转时 AIN1=1, AIN2=0
     * 反转时 AIN1=0, AIN2=1
     * 停转时两个方向脚都拉低 */
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

    /* 右轮方向控制，逻辑与左轮相同。 */
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

    /* 当前实现设置了最小占空比门槛。
     * 这样可以跨过电机死区，但会让很小的非零输出被抬高。 */
    if (duty_l < 25)
        duty_l = 25;
    if (duty_r < 25)
        duty_r = 25;

    /* 把计算得到的比较值写入两个 PWM 通道。 */
    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_l, DL_TIMER_CC_0_INDEX);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, duty_r, DL_TIMER_CC_1_INDEX);
}

void Motor_Stop(void)
{
    /* 提供统一停车入口，便于后续扩展为刹车或软停。 */
    Set_PWM(0, 0);
}
