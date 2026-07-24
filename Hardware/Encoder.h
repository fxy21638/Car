#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

void Encoder_Init(void);
void Encoder_ResetDistance(void);
float Get_Current_Circles(void);
int32_t Encoder_GetDistancePulses(void);
void Encoder_SetPulsesPerCircle(uint32_t pulses);
void Encoder_DebugPrint(void);
int Encoder_GetLeftSpeed(void);
int Encoder_GetRightSpeed(void);

#endif
