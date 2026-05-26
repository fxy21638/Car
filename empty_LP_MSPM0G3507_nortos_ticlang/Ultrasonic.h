#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

/* 初始化：
 * - Trig/Echo 引脚来自 SysConfig 的 Ultrasonic_Pins_* 宏
 * - 使用 50us 定时器中断实现非阻塞测量
 */
void Ultrasonic_Init(void);

/* 周期性调用（建议在主循环里每次都调用一次） */
void Ultrasonic_Task(unsigned long nowMs);

/* 最近一次测得的距离（cm）；-1 表示无效/超时 */
int16_t Ultrasonic_GetDistanceCm(void);
void Ultrasonic_GetDebugState(int16_t *distanceCm, uint8_t *measuring,
                              uint32_t *echoTimeUs, uint32_t *waitTimeoutUs, uint32_t *irqCount);

/* 简单判定：距离小于阈值认为有障碍 */
bool Ultrasonic_IsObstacle(void);

#endif
