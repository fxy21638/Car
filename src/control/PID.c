#include "PID.h"
#include "Motor.h"
#include "control_math.h"
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

/* 速度环 + 角度修正：以 targetYawDeg 为目标航向，用 anglePID 修正左右轮差速，
 * 在直行过程中主动抵抗跑偏，降低对两轮机械对称性的要求。 */
void PID_control_head(int speed, float targetYawDeg)
{
    float err = Control_AngleDifference(targetYawDeg, yaw);

    float steer = Angle_Control(0.0f, err);

    targetLeftSpeed  = speed + (int)steer;
    targetRightSpeed = speed - (int)steer;

    leftPID.target  = targetLeftSpeed;
    rightPID.target = targetRightSpeed;
    leftPID.actual  = leftEncSpeed;
    rightPID.actual = rightEncSpeed;
    PID_Update(&leftPID);
    PID_Update(&rightPID);

    PWMleft  = (int)leftPID.output;
    PWMright = (int)rightPID.output;
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
    float err = Control_AngleDifference(targetYawDeg, yaw);

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
