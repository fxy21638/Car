/*
 * ============================================================
 *  encoder — 正交编码器硬件抽象
 *
 *  引脚: E1A(PB3) E1B(PB2) — 左编码器 (A相中断, B相方向)
 *        E2A(PB16) E2B(PB15) — 右编码器 (A相中断, B相方向)
 *  中断: GROUP1_IRQHandler (GPIOB)
 *  计数: 32位有符号, A相上升沿 ±1 (单沿, 非四倍频)
 * ============================================================
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

extern volatile uint8_t g_mpu6050_flag;

/* 编码器累计脉冲 (ISR 写入, 主循环读取) */
extern volatile int32_t g_encoder_left_count;
extern volatile int32_t g_encoder_right_count;

void Encoder_Init(void);
void Encoder_ResetDistance(void);

/* 原子读取编码器快照 */
void Encoder_Read(int32_t *left, int32_t *right);

/* 圈数与距离 */
float   Get_Current_Circles(void);
int32_t Encoder_GetDistancePulses(void);
void    Encoder_SetPulsesPerCircle(uint32_t pulses);
void    Encoder_SetLeftCountsPerM(uint32_t counts);
void    Encoder_SetRightCountsPerM(uint32_t counts);
void    Encoder_SetLeftSign(int sign);
void    Encoder_SetRightSign(int sign);

/* 速度 (由 TIMA0 ISR 每 10ms 更新) */
int Encoder_GetLeftSpeed(void);
int Encoder_GetRightSpeed(void);

void Encoder_DebugPrint(void);

#endif
