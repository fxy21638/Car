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

void PID_control(void)
{
    //        // ???PID????
    steerPID.target = 0;
    steerPID.actual = linePos;
    PID_Update(&steerPID);

    targetLeftSpeed = BASE_SPEED + steerPID.output;
    targetRightSpeed = BASE_SPEED - steerPID.output;

    //        // ?????PID
    leftPID.target = targetLeftSpeed;
    rightPID.target = targetRightSpeed;

    leftPID.actual = leftEncSpeed;
    rightPID.actual = rightEncSpeed;
    PID_Update(&leftPID);
    PID_Update(&rightPID);

    int turn_speed = 30;
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

    // PWMleft = BASE_SPEED - linePos;
    // PWMright = BASE_SPEED + linePos;

    Set_PWM(PWMleft, PWMright);
}

void PID_control_head(int left,int right)
{
    // ==================== 纯速度环，无转向 ====================
    // 固定目标速度（你可以在这里改，用来测试加减速、阶跃响应）
    targetLeftSpeed = left;  // 左轮目标速度
    targetRightSpeed = right; // 右轮目标速度

    // 速度环 PID 计算
    leftPID.target = targetLeftSpeed;
    rightPID.target = targetRightSpeed;

    leftPID.actual = leftEncSpeed; // 编码器实际速度
    rightPID.actual = rightEncSpeed;

    PID_Update(&leftPID);
    PID_Update(&rightPID);

    // 直接输出PID结果到电机（无任何干扰）
    PWMleft = leftPID.output;
    PWMright = rightPID.output;

    // 输出PWM
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

/* 转向任务使用的角度环辅助函数。
 * 先把目标角误差（已处理 ±180° 环绕）送入 anglePID，再交给左右轮速度环去执行。 */
void TurnToAngle(float targetYawDeg)
{
    float err = targetYawDeg - yaw;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    float steer = Angle_Control(0.0f, err);

    targetLeftSpeed = BASE_SPEED + (int)steer;
    targetRightSpeed = BASE_SPEED -(int)steer;

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
