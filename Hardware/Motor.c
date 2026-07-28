/*
 * ============================================================
 *  motor — TB6612 双路电机驱动实现
 *
 *  PWM: TIMG12, 80MHz, period=1000, 逻辑范围 ±2000
 *  方向: IN1/IN2 GPIO 控制 H 桥
 *  PWM > 0  →  IN1=HIGH, IN2=LOW  (正转)
 *  PWM < 0  →  IN1=LOW,  IN2=HIGH (反转)
 *  PWM = 0  →  IN1=LOW,  IN2=LOW  (滑行)
 * ============================================================
 */

#include "Motor.h"

/* 阶梯化: 每调用一次 PWM 最多变化 ±500, 实现平滑加减速 */
#define SLEW_MAX (500)

static int16_t g_prev_left;
static int16_t g_prev_right;

static int16_t clamp_pwm(int16_t pwm)
{
    if (pwm > 2000) return 2000;
    if (pwm < -2000) return -2000;
    return pwm;
}

static int16_t slew(int16_t target, int16_t current)
{
    int32_t diff = (int32_t)target - (int32_t)current;
    if (diff > SLEW_MAX)       return (int16_t)(current + SLEW_MAX);
    if (diff < -(int32_t)SLEW_MAX) return (int16_t)(current - (int16_t)SLEW_MAX);
    return target;
}

static void set_motor(GPIO_Regs *port, uint32_t in1, uint32_t in2,
    int16_t pwm, DL_TIMER_CC_INDEX channel)
{
    uint32_t compare;

    pwm = clamp_pwm(pwm);
    if (pwm > 0) {
        DL_GPIO_setPins(port, in1);
        DL_GPIO_clearPins(port, in2);
    } else if (pwm < 0) {
        DL_GPIO_clearPins(port, in1);
        DL_GPIO_setPins(port, in2);
        pwm = (int16_t)-pwm;
    } else {
        DL_GPIO_clearPins(port, in1 | in2);
    }
    /* 逻辑 PWM 2000 → 比较值 1000 (100% 占空比) */
    compare = (uint32_t)pwm / 2U;
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, compare, channel);
}

void Motor_Init(void)
{
    g_prev_left  = 0;
    g_prev_right = 0;
    Motor_ShortBrake();
}

void Motor_Drive(int16_t left_pwm, int16_t right_pwm)
{
    g_prev_left  = slew(left_pwm,  g_prev_left);
    g_prev_right = slew(right_pwm, g_prev_right);

    set_motor(MOTOR_PORT, MOTOR_AIN1_PIN, MOTOR_AIN2_PIN,
        g_prev_left, DL_TIMER_CC_0_INDEX);
    set_motor(MOTOR_PORT, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN,
        g_prev_right, DL_TIMER_CC_1_INDEX);
}

/* TB6612 短路刹车: 方向引脚拉低, PWM 100% 占空比 (同相=制动) */
void Motor_ShortBrake(void)
{
    DL_GPIO_clearPins(MOTOR_PORT,
        MOTOR_AIN1_PIN | MOTOR_AIN2_PIN);
    DL_GPIO_clearPins(MOTOR_PORT,
        MOTOR_BIN1_PIN | MOTOR_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 1000U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_0_INST, 1000U, DL_TIMER_CC_1_INDEX);
    g_prev_left  = 0;
    g_prev_right = 0;
}
