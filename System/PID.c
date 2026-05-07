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
extern float yaw;

static const int kCenterDeadband = 2;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float output_max, float output_min, float sum_max, float filter_alpha)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->sum_max = sum_max;
    pid->filter_alpha = (filter_alpha > 1.0f) ? 1.0f : (filter_alpha < 0.0f) ? 0.0f
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
    pid->err_filtered = pid->filter_alpha * pid->err + (1 - pid->filter_alpha) * pid->err_filtered;
    pid->diff = pid->err_filtered - pid->err_last;
    pid->err_last = pid->err_filtered;

    if (!(pid->output >= pid->output_max && pid->err_filtered > 0) &&
        !(pid->output <= pid->output_min && pid->err_filtered < 0))
    {
        pid->err_sum += pid->err_filtered;
        if (pid->err_sum > pid->sum_max)
            pid->err_sum = pid->sum_max;
        else if (pid->err_sum < -pid->sum_max)
            pid->err_sum = -pid->sum_max;
    }

    pid->output = pid->Kp * pid->err_filtered +
                  pid->Ki * pid->err_sum +
                  pid->Kd * pid->diff;

    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;
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
    int steerActual = linePos;
    if ((steerActual >= -kCenterDeadband) && (steerActual <= kCenterDeadband))
    {
        steerActual = 0;
    }

    steerPID.target = 0;
    steerPID.actual = steerActual;
    PID_Update(&steerPID);

    targetLeftSpeed = BASE_SPEED + (int)steerPID.output;
    targetRightSpeed = BASE_SPEED - (int)steerPID.output;

    leftPID.target = targetLeftSpeed;
    rightPID.target = targetRightSpeed;
    leftPID.actual = leftEncSpeed;
    rightPID.actual = rightEncSpeed;
    PID_Update(&leftPID);
    PID_Update(&rightPID);

    if (is_lost == 0)
    {
        PWMleft = (int)leftPID.output;
        PWMright = (int)rightPID.output;
    }
    else if (is_lost > 0)
    {
        PWMleft = -30;
        PWMright = 30;
    }
    else
    {
        PWMleft = 30;
        PWMright = -30;
    }

    Set_PWM(PWMleft, PWMright);
}

float Angle_Control(float targetYawDeg, float actualYawDeg)
{
    anglePID.target = targetYawDeg;
    anglePID.actual = actualYawDeg;
    PID_Update(&anglePID);
    return anglePID.output;
}

void TurnToAngle(float targetYawDeg)
{
    float err = targetYawDeg - yaw;
    float steer = Angle_Control(0.0f, err);

    targetLeftSpeed = (int)steer;
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
