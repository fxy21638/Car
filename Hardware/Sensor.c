#include "Sensor.h"

const int posWeight[8] = {-20, -10, -5, -2, 2, 5, 10, 20};

extern int BASE_SPEED;
extern int linePos;
extern int lastSumPos;
extern int is_lost;

int GPIO_ReadPin(GPIO_Regs *GPIOx, uint32_t pin)
{
    // ��ȡ���ŵ�ƽ
    if ((DL_GPIO_readPins(GPIOx, pin)) == 0)
    {
        return 1; // �ߵ�ƽ
    }
    else
    {
        return 0; // �͵�ƽ
    }
}
//(1<<pin)>>pin)&1)

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
        return -1; // ���󣺴�������Ų�����
    }
}

int Sensor_GetQuantizedPos(void)
{
    int sumPos = 0;
    int count = 0;

    // ����8��������
    for (int i = 0; i < 8; i++)
    {
        if (Sensor_GetState(i) == 1)
        {                           // ��⵽����
            sumPos += posWeight[i]; // �ۼ�λ��Ȩ��
        }
        else
            count++;
    }

    // �����ȫ���ߣ�ʹ���ϴε�λ��
    if (count == 0)
    {
        sumPos = lastSumPos; // ʹ���ϴ�λ�ü�������
        if (lastSumPos > 0)
            is_lost = 1;
        else if (lastSumPos < 0)
            is_lost = -1;
        else
            is_lost = 0;

        if (BASE_SPEED > 0)
            BASE_SPEED -= 10;
    }
    else
    {
        lastSumPos = sumPos; // �����ϴ�λ��
        is_lost = 0;
        if (BASE_SPEED < 60)
            BASE_SPEED += 5;
    }

    // ����λ�ã���Ȩ��ƥ�䣩
    return sumPos;
}
