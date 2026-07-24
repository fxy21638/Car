#ifndef ENCODER_H
#define ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

extern volatile int32_t Get_Encoder_countA;
extern volatile int32_t Get_Encoder_countB;
extern volatile int32_t encoderA_cnt;
extern volatile int32_t encoderB_cnt;

extern int leftEncSpeed;
extern int rightEncSpeed;
extern volatile uint8_t g_mpu6050_flag;

void Encoder_Init(void);
void Encoder_ResetDistance(void);
float Get_Current_Circles(void);
int32_t Encoder_GetDistancePulses(void);
void Encoder_SetPulsesPerCircle(uint32_t pulses);
void Encoder_DebugPrint(void);

#endif
