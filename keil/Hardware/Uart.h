#ifndef _UART_H_
#define _UART_H_

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "Delay.h"

void Uart_TX_Init(void);
void Uart_PollTx(void);
void Uart_SendByte(uint8_t byte);
void Uart_SendBytes(const uint8_t *data, uint32_t len);
void Uart_SendString(const char *s);
void Uart_SendInt(int num);
void VOFA_SendSpeedLoop(void);
void show_dis(void);

#endif 