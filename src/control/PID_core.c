#include "PID.h"

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
    pid->filter_alpha = (filter_alpha > 1.0f) ? 1.0f :
                        ((filter_alpha < 0.0f) ? 0.0f : filter_alpha);
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
    pid->err_filtered = pid->filter_alpha * pid->err +
                        (1.0f - pid->filter_alpha) * pid->err_filtered;
    pid->diff = pid->err_filtered - pid->err_last;
    pid->err_last = pid->err_filtered;

    if (!(pid->output >= pid->output_max && pid->err_filtered > 0.0f) &&
        !(pid->output <= pid->output_min && pid->err_filtered < 0.0f))
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
