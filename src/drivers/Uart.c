#include "Uart.h"
#include "ti_msp_dl_config.h"
#include "PID.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Delay.h"

extern int PWMleft, PWMright;
extern int targetLeftSpeed, targetRightSpeed;
extern int leftEncSpeed, rightEncSpeed;

#define UART_RX_LINE_MAX 64

static char g_uartRxLine[UART_RX_LINE_MAX];
static uint32_t g_uartRxLen = 0;

static void Uart_TxBlocking(uint8_t byte)
{
	DL_UART_Main_transmitDataBlocking(UART_0_INST, byte);
}

void Uart_SendByte(uint8_t byte)
{
	Uart_TxBlocking(byte);
}

void Uart_SendBytes(const uint8_t *data, uint32_t len)
{
	if (data == NULL || len == 0u)
	{
		return;
	}

	for (uint32_t i = 0; i < len; i++)
	{
		Uart_TxBlocking(data[i]);
	}
}

void Uart_SendString(const char *s)
{
	if (s == NULL)
	{
		return;
	}

	while (*s != '\0')
	{
		Uart_TxBlocking((uint8_t)(*s));
		s++;
	}
}

// 新增：发送有符号整数
void Uart_SendInt(int num)
{
    if (num < 0) {
        Uart_SendByte('-');
        num = -num;
    }

    // 处理 0 的情况
    if (num == 0) {
        Uart_SendByte('0');
        return;
    }

    // 将数字逆序存入缓冲区
    char buf[12]; // 足够容纳 32 位有符号整数（最多 10 位 + 符号）
    int i = 0;
    while (num > 0) {
        buf[i++] = '0' + (num % 10);
        num /= 10;
    }

    // 反向发送
    while (--i >= 0) {
        Uart_SendByte(buf[i]);
    }
}

void VOFA_SendSpeedLoop(void)
{
    // 获取左右轮数据
    int targetL = targetLeftSpeed;
    int actualL = leftEncSpeed;
    int outputL = PWMleft;

    int targetR = targetRightSpeed;
    int actualR = rightEncSpeed;
    int outputR = PWMright;

    // 手动构建 CSV 输出: TgtL,ActL,TgtR,ActR,PWML,PWMR\r\n
    Uart_SendInt(targetL);
    Uart_SendByte(',');
    Uart_SendInt(actualL);
    Uart_SendByte(',');
    Uart_SendInt(targetR);
    Uart_SendByte(',');
    Uart_SendInt(actualR);
    Uart_SendByte(',');
    Uart_SendInt(outputL);
    Uart_SendByte(',');
    Uart_SendInt(outputR);
    Uart_SendString("\r\n");
}

void show_dis(void)
{
	static unsigned long lastPrintMs = 0;
	unsigned long nowMs = tick_ms;
	
	if ((nowMs - lastPrintMs) >= 500UL || lastPrintMs == 0)
	{
		lastPrintMs = nowMs;
		Uart_SendString("hello\r\n");
	}
}
