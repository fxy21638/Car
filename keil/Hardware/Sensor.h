#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

void Sensor_Init(void);
int Sensor_GetState(int i);
int Sensor_GetQuantizedPos(void);
void Sensor_ShowDebug(void);

#endif
