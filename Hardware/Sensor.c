#include "Sensor.h"
#include "OLED.h"

const int posWeight[8] = {-20, -10, -5, -2, 2, 5, 10, 20};

#include "robot.h"

/* 硬件：白底→返回1, 黑线→返回0 */
int GPIO_ReadPin(GPIO_Regs *GPIOx, uint32_t pin)
{
    if ((DL_GPIO_readPins(GPIOx, pin)) == 0)
        return 1;
    return 0;
}

/* 8路传感器端口/引脚查表，O(1)替代 switch-case */
static GPIO_Regs *const s_sensorPorts[8] = {
    TRACK_SENSOR_S0_PORT, TRACK_SENSOR_S1_PORT,
    TRACK_SENSOR_S2_PORT, TRACK_SENSOR_S3_PORT,
    TRACK_SENSOR_S4_PORT, TRACK_SENSOR_S5_PORT,
    TRACK_SENSOR_S6_PORT, TRACK_SENSOR_S7_PORT
};
static const uint32_t s_sensorPins[8] = {
    TRACK_SENSOR_S0_PIN, TRACK_SENSOR_S1_PIN,
    TRACK_SENSOR_S2_PIN, TRACK_SENSOR_S3_PIN,
    TRACK_SENSOR_S4_PIN, TRACK_SENSOR_S5_PIN,
    TRACK_SENSOR_S6_PIN, TRACK_SENSOR_S7_PIN
};

/* 传感器读数缓存 */
static int s_cachedStates[8];

void Sensor_Init(void)
{
}

/* 直接 GPIO 读取 */
int Sensor_GetState(int i)
{
    if (i >= 0 && i < 8)
        return GPIO_ReadPin(s_sensorPorts[i], s_sensorPins[i]);
    return -1;
}

/* 从缓存读取 (需先调用 Sensor_GetQuantizedPos 更新缓存) */
int Sensor_GetStateCached(int i)
{
    if (i >= 0 && i < 8)
        return s_cachedStates[i];
    return -1;
}

/* 赛道：白底黑线，Sensor_GetState 返回 1=白底 0=黑线。
 * sumPos = 白色区域加权中心，count = 黑线传感器数。
 * count==0 即全白，连续 5 次判定为丢线，避免过弯/晃动时误触发。 */
int Sensor_GetQuantizedPos(RobotState *rs)
{
    static int s_lostDebounce = 0;
    int sumPos = 0;
    int count = 0;

    for (int i = 0; i < 8; i++)
    {
        int state = Sensor_GetState(i);
        s_cachedStates[i] = state;          /* 填充缓存 */
        if (state == 1)
            sumPos += posWeight[i];
        else
            count++;
    }

    if (count == 0)
    {
        s_lostDebounce++;
        if (s_lostDebounce >= 5)
        {
            if (rs->lastSumPos > 0)
                rs->isLost = 1;
            else if (rs->lastSumPos < 0)
                rs->isLost = -1;
            else
                rs->isLost = 0;

            if (rs->baseSpeed > 0)
                rs->baseSpeed -= 10;
        }
        sumPos = rs->lastSumPos;
    }
    else
    {
        s_lostDebounce = 0;
        rs->lastSumPos = sumPos;
        rs->isLost = 0;
        if (rs->baseSpeed < 60)
            rs->baseSpeed += 5;
    }
    return sumPos;
}

/* OLED 传感器调试页：显示 8 路原始值、在线数、加权位置、丢线标志 */
void Sensor_ShowDebug(RobotState *rs)
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
    OLED_ShowSignedNum(40, 48, rs->linePos, 4, OLED_8X16);

    OLED_ShowString(80, 48, "L:", OLED_8X16);
    OLED_ShowSignedNum(104, 48, rs->isLost, 2, OLED_8X16);

    OLED_Update();
}
