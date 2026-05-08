#include "speed_loop.h"

extern void PID_control(void);

void SpeedLoop_Task(void)
{
    PID_control();
}
