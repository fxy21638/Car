#include "Uart.h"

#include "../../ti_msp_dl_config.h"
#include "../../Hardware/Delay.h"

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
