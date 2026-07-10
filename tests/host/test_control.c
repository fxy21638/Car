#include "PID.h"
#include "control_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static int nearly_equal(float a, float b)
{
    return fabsf(a - b) < 0.001f;
}

static void test_angle_difference(void)
{
    assert(nearly_equal(Control_AngleDifference(10.0f, 350.0f), 20.0f));
    assert(nearly_equal(Control_AngleDifference(-170.0f, 170.0f), 20.0f));
    assert(nearly_equal(Control_AngleDifference(170.0f, -170.0f), -20.0f));
}

static void test_pid_limits_and_reset(void)
{
    PID_t pid;
    PID_Init(&pid, 10.0f, 1.0f, 0.0f, 100.0f, -100.0f, 5.0f, 1.0f);
    pid.target = 100.0f;
    pid.actual = 0.0f;
    PID_Update(&pid);
    assert(nearly_equal(pid.output, 100.0f));
    assert(pid.err_sum <= 5.0f);
    PID_Reset(&pid);
    assert(nearly_equal(pid.output, 0.0f));
    assert(nearly_equal(pid.err_sum, 0.0f));
}

int main(void)
{
    test_angle_difference();
    test_pid_limits_and_reset();
    puts("control tests passed");
    return 0;
}
