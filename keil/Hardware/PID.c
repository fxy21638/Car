#include "PID.h"
#include "Motor.h"
#include <math.h>

extern int BASE_SPEED;
extern int linePos;
extern PID_t leftPID;
extern PID_t rightPID;
extern PID_t steerPID;
extern PID_t anglePID;
extern int PWMleft, PWMright;
extern int targetLeftSpeed, targetRightSpeed;
extern int leftEncSpeed, rightEncSpeed;
extern int is_lost;
extern float angle_err;
extern float yaw;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float output_max, float output_min, float sum_max,
              float filter_alpha)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->sum_max = sum_max;
    pid->filter_alpha = (filter_alpha > 1.0f)
                            ? 1.0f
                        : (filter_alpha < 0.0f) ? 0.0f
                                                : filter_alpha;

    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_filtered = 0.0f;
    pid->err_sum = 0.0f;
    pid->diff = 0.0f;
    pid->output = 0.0f;
}

void PID_Update(PID_t *pid)
{
    pid->err = pid->target - pid->actual;

    /* 对误差做一阶低通滤波，减小编码器抖动对输出的影响。 */
    pid->err_filtered = pid->filter_alpha * pid->err +
                        (1 - pid->filter_alpha) * pid->err_filtered;

    pid->diff = pid->err_filtered - pid->err_last;
    pid->err_last = pid->err_filtered;

    /* 抗积分饱和：
     * 当输出已经顶到上下限，且误差方向还在继续“推着积分变大”时，暂停积分。 */
    if (!(pid->output >= pid->output_max && pid->err_filtered > 0) &&
        !(pid->output <= pid->output_min && pid->err_filtered < 0))
    {
        pid->err_sum += pid->err_filtered;
        if (pid->err_sum > pid->sum_max)
        {
            pid->err_sum = pid->sum_max;
        }
        else if (pid->err_sum < -pid->sum_max)
        {
            pid->err_sum = -pid->sum_max;
        }
    }

    pid->output = pid->Kp * pid->err_filtered +
                  pid->Ki * pid->err_sum +
                  pid->Kd * pid->diff;

    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }
}

void PID_Reset(PID_t *pid)
{
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_filtered = 0.0f;
    pid->err_sum = 0.0f;
    pid->diff = 0.0f;
    pid->output = 0.0f;
}

/* ---- 标准循线控制 ----
 * 串联双环 PID：转向环(位置) → 速度环(编码器) → H桥PWM
 *
 * 数据流：
 *   linePos(Sensor) → steerPID → targetSpeed → leftPID/rightPID → PWMleft/right → Motor
 *                                              ↑ leftEncSpeed/rightEncSpeed (Encoder)
 *
 * 丢线时 (is_lost!=0) 跳过PID，原地旋转回找。 */
void PID_control(void)
{
    /* 转向环：目标=线居中(0)，实际=linePos */
    steerPID.target = 0;
    steerPID.actual = linePos;
    PID_Update(&steerPID);

    /* 转向输出 → 左右轮差速目标 */
    targetLeftSpeed = BASE_SPEED + steerPID.output;
    targetRightSpeed = BASE_SPEED - steerPID.output;

    /* 速度环：目标=上述差速值，实际=编码器测得轮速 */
    leftPID.target = targetLeftSpeed;
    rightPID.target = targetRightSpeed;
    leftPID.actual = leftEncSpeed;
    rightPID.actual = rightEncSpeed;
    PID_Update(&leftPID);
    PID_Update(&rightPID);

    /* 丢线处理：原地旋转回找，不走PID输出 */
    int turn_speed = 35;
    if (is_lost == 0)
    {
        PWMleft = leftPID.output;
        PWMright = rightPID.output;
    }
    else
    {
        if (is_lost == 1)
        {
            PWMleft = -turn_speed;
            PWMright = turn_speed;
        }
        else if (is_lost == -1)
        {
            PWMleft = turn_speed;
            PWMright = -turn_speed;
        }
    }

    Set_PWM(PWMleft, PWMright);
}

/* 纯速度环：固定目标速度直行，无转向干预。供对角线直线段等场景使用。 */
void PID_control_head(int left, int right)
{
    targetLeftSpeed = left;
    targetRightSpeed = right;

    leftPID.target = targetLeftSpeed;
    rightPID.target = targetRightSpeed;
    leftPID.actual = leftEncSpeed;
    rightPID.actual = rightEncSpeed;
    PID_Update(&leftPID);
    PID_Update(&rightPID);

    PWMleft = leftPID.output;
    PWMright = rightPID.output;
    Set_PWM(PWMleft, PWMright);
}

void PID_SetSpeedTunings(float kp, float ki, float kd)
{
    leftPID.Kp = kp;
    leftPID.Ki = ki;
    leftPID.Kd = kd;
    rightPID.Kp = kp;
    rightPID.Ki = ki;
    rightPID.Kd = kd;
    PID_Reset(&leftPID);
    PID_Reset(&rightPID);
}

float Angle_Control(float targetYawDeg, float actualYawDeg)
{
    anglePID.target = targetYawDeg;
    anglePID.actual = actualYawDeg;
    PID_Update(&anglePID);
    return anglePID.output;
}

/* ---- 转向到目标偏航角 ----
 * 角度环 + 速度环串联，原地或低速旋转对准目标航向。
 * 每帧调用一次，直到 yaw 接近目标（由上层判断到达条件）。 */
void TurnToAngle(float targetYawDeg)
{
    float err = targetYawDeg - yaw;
    while (err > 180.0f)  err -= 360.0f;   /* ±180° 归一化 */
    while (err < -180.0f) err += 360.0f;

    float steer = Angle_Control(0.0f, err); /* anglePID: 角度误差→转向量 */

    targetLeftSpeed =  +(int)steer;
    targetRightSpeed = -(int)steer;

    leftPID.target = targetLeftSpeed;
    rightPID.target = targetRightSpeed;
    leftPID.actual = leftEncSpeed;
    rightPID.actual = rightEncSpeed;
    PID_Update(&leftPID);
    PID_Update(&rightPID);

    PWMleft = (int)leftPID.output;
    PWMright = (int)rightPID.output;
    Set_PWM(PWMleft, PWMright);
}
