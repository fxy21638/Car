#include "app_state.h"
#include "pid_test.h"
#include "OLED.h"
#include "speed_loop.h"

void PIDTest_Task(unsigned long nowMs)
{
    (void) nowMs;

    SpeedLoop_Task();

    OLED_Clear();
    OLED_ShowString(0, 0, "=== PID TEST ===", OLED_8X16);
    OLED_ShowString(0, 16, "Yaw:", OLED_8X16);
    OLED_ShowSignedNum(40, 16, (int16_t) yaw, 4, OLED_8X16);
    OLED_ShowString(0, 32, "L:", OLED_8X16);
    OLED_ShowSignedNum(24, 32, leftEncSpeed, 4, OLED_8X16);
    OLED_ShowString(64, 32, "R:", OLED_8X16);
    OLED_ShowSignedNum(88, 32, rightEncSpeed, 4, OLED_8X16);
    OLED_ShowString(0, 48, "SpeedLoop", OLED_8X16);
    OLED_Update();
}
