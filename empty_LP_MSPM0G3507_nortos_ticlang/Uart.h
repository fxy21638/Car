#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>

void Uart_SendByte(uint8_t byte);
void Uart_SendBytes(const uint8_t *data, uint32_t len);
void Uart_SendString(const char *s);

void show_dis(void);

#endif 