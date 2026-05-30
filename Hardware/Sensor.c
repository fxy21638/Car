#include "Sensor.h"
#include "OLED.h"

const int posWeight[8] = {-20, -10, -5, -2, 2, 5, 10, 20};

extern int BASE_SPEED;
extern int linePos;
extern int lastSumPos;
extern int is_lost;

/* 硬件：白底→返回1, 黑线→返回0 */
int GPIO_ReadPin(GPIO_Regs *GPIOx, uint32_t pin)
{
    if ((DL_GPIO_readPins(GPIOx, pin)) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void Sensor_Init(void)
{
}

int Sensor_GetState(int i)
{
    switch (i)
    {
    case 0:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S0_PIN);

    case 1:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S1_PIN);

    case 2:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S2_PIN);

    case 3:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S3_PIN);

    case 4:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S4_PIN);

    case 5:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S5_PIN);

    case 6:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S6_PIN);

    case 7:
        return GPIO_ReadPin(TRACK_SENSOR_PORT, TRACK_SENSOR_S7_PIN);

    default:
        return -1; // 错误：传感器编号不合法
    }
}

/* 赛道：白底黑线，Sensor_GetState 返回 1=白底 0=黑线。
 * sumPos = 白色区域加权中心，count = 黑线传感器数。
 * count==0 即全白，连续 5 次判定为丢线，避免过弯/晃动时误触发。 */
int Sensor_GetQuantizedPos(void)
{
    static int s_lostDebounce = 0;
    int sumPos = 0;
    int count = 0;

    for (int i = 0; i < 8; i++)
    {
        if (Sensor_GetState(i) == 1)
        {
            sumPos += posWeight[i];
        }
        else
            count++;
    }

    if (count == 0)
    {
        s_lostDebounce++;
        if (s_lostDebounce >= 5)
        {
            if (lastSumPos > 0)
                is_lost = 1;
            else if (lastSumPos < 0)
                is_lost = -1;
            else
                is_lost = 0;

            if (BASE_SPEED > 0)
                BASE_SPEED -= 10;
        }
        sumPos = lastSumPos;
    }
    else
    {
        s_lostDebounce = 0;
        lastSumPos = sumPos;
        is_lost = 0;
        if (BASE_SPEED < 60)
            BASE_SPEED += 5;
    }
    return sumPos;
}

/* OLED 传感器调试页：显示 8 路原始值、在线数、加权位置、丢线标志 */
void Sensor_ShowDebug(void)
{
    char buf[9];
    int i;
    int onCount = 0;

    OLED_Clear();

    for (i = 0; i < 8; i++)
    {
        int v = Sensor_GetState(i);
        buf[i] = v ? '1' : '0';
        if (v) onCount++;
    }
    buf[8] = '\0';

    OLED_ShowString(0, 0, "S0-3:", OLED_8X16);
    OLED_ShowString(48, 0, buf, OLED_8X16);

    OLED_ShowString(0, 16, "S4-7:", OLED_8X16);
    OLED_ShowString(48, 16, buf + 4, OLED_8X16);

    OLED_ShowString(0, 32, "On:", OLED_8X16);
    OLED_ShowNum(32, 32, onCount, 1, OLED_8X16);

    OLED_ShowString(0, 48, "Pos:", OLED_8X16);
    OLED_ShowSignedNum(40, 48, linePos, 4, OLED_8X16);

    OLED_ShowString(80, 48, "L:", OLED_8X16);
    OLED_ShowSignedNum(104, 48, is_lost, 2, OLED_8X16);

    OLED_Update();
}
