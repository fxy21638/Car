#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "ti_msp_dl_config.h"
#include "Delay.h"
#include <stdint.h>
#include <stdbool.h>

void Ultrasonic_Init(void);

/* 主循环中每次调用，80ms 自动触发一次新测量 */
void Ultrasonic_Task(unsigned long nowMs);

/* 返回最近一次距离(cm)，-1 表示无效/超时 */
int16_t Ultrasonic_GetDistanceCm(void);

/* 简单障碍判定：距离 < 20cm 返回 true */
bool Ultrasonic_IsObstacle(void);

#endif
