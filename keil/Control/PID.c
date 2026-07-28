#include "PID.h"
#include "Motor.h"
#include "Encoder.h"
#include "robot.h"
#include <math.h>

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
 *   linePos(Sensor) → steerPID → targetSpeed → leftPID/rightPID → PWM → Motor
 *                                              ↑ Encoder_GetLeftSpeed/RightSpeed
 *
 * 丢线时 (isLost!=0) 跳过PID，原地旋转回找。 */
void PID_control(RobotState *rs)
{
    /* 转向环：目标=线居中(0)，实际=linePos */
    rs->steerPID.target = 0;
    rs->steerPID.actual = rs->linePos;
    PID_Update(&rs->steerPID);

    /* 转向输出 → 左右轮差速目标 */
    rs->targetLeftSpeed = rs->baseSpeed + rs->steerPID.output;
    rs->targetRightSpeed = rs->baseSpeed - rs->steerPID.output;

    /* 速度环：目标=上述差速值，实际=编码器测得轮速 */
    rs->leftPID.target = rs->targetLeftSpeed;
    rs->rightPID.target = rs->targetRightSpeed;
    rs->leftPID.actual = Encoder_GetLeftSpeed();
    rs->rightPID.actual = Encoder_GetRightSpeed();
    PID_Update(&rs->leftPID);
    PID_Update(&rs->rightPID);

    /* 丢线处理：原地旋转回找，不走PID输出 */
    int turn_speed = 60;
    if (rs->isLost == 0)
    {
        rs->pwmLeft = rs->leftPID.output;
        rs->pwmRight = rs->rightPID.output;
    }
    else
    {
        if (rs->isLost == 1)
        {
            rs->pwmLeft = -turn_speed;
            rs->pwmRight = turn_speed;
        }
        else if (rs->isLost == -1)
        {
            rs->pwmLeft = turn_speed;
            rs->pwmRight = -turn_speed;
        }
    }

    Set_PWM(rs->pwmLeft, rs->pwmRight);
}

/* 角度归一化到 [-180, 180]，消除多处重复的 while 循环 */
float Angle_Normalize(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* 速度环 + 角度修正：以 targetYawDeg 为目标航向，用 anglePID 修正左右轮差速，
 * 在直行过程中主动抵抗跑偏，降低对两轮机械对称性的要求。 */
void PID_control_head(RobotState *rs, int speed, float targetYawDeg)
{
    float err = Angle_Normalize(targetYawDeg - rs->yaw);

    float steer = Angle_Control(rs, 0.0f, err);

    rs->targetLeftSpeed  = speed + (int)steer;
    rs->targetRightSpeed = speed - (int)steer;

    rs->leftPID.target  = rs->targetLeftSpeed;
    rs->rightPID.target = rs->targetRightSpeed;
    rs->leftPID.actual  = Encoder_GetLeftSpeed();
    rs->rightPID.actual = Encoder_GetRightSpeed();
    PID_Update(&rs->leftPID);
    PID_Update(&rs->rightPID);

    rs->pwmLeft  = (int)rs->leftPID.output;
    rs->pwmRight = (int)rs->rightPID.output;
    Set_PWM(rs->pwmLeft, rs->pwmRight);
}

void PID_SetSpeedTunings(RobotState *rs, float kp, float ki, float kd)
{
    rs->leftPID.Kp = kp;
    rs->leftPID.Ki = ki;
    rs->leftPID.Kd = kd;
    rs->rightPID.Kp = kp;
    rs->rightPID.Ki = ki;
    rs->rightPID.Kd = kd;
    PID_Reset(&rs->leftPID);
    PID_Reset(&rs->rightPID);
}

float Angle_Control(RobotState *rs, float targetYawDeg, float actualYawDeg)
{
    rs->anglePID.target = targetYawDeg;
    rs->anglePID.actual = actualYawDeg;
    PID_Update(&rs->anglePID);
    return rs->anglePID.output;
}

/* ---- 转向到目标偏航角 ----
 * 角度环 + 速度环串联，原地或低速旋转对准目标航向。
 * 每帧调用一次，直到 yaw 接近目标（由上层判断到达条件）。 */
void TurnToAngle(RobotState *rs, float targetYawDeg)
{
    float err = Angle_Normalize(targetYawDeg - rs->yaw);

    float steer = Angle_Control(rs, 0.0f, err); /* anglePID: 角度误差→转向量 */

    rs->targetLeftSpeed =  +(int)steer;
    rs->targetRightSpeed = -(int)steer;

    rs->leftPID.target = rs->targetLeftSpeed;
    rs->rightPID.target = rs->targetRightSpeed;
    rs->leftPID.actual = Encoder_GetLeftSpeed();
    rs->rightPID.actual = Encoder_GetRightSpeed();
    PID_Update(&rs->leftPID);
    PID_Update(&rs->rightPID);

    rs->pwmLeft  = (int)rs->leftPID.output;
    rs->pwmRight = (int)rs->rightPID.output;
    Set_PWM(rs->pwmLeft, rs->pwmRight);
}
