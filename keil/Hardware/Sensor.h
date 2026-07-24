#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "OLED.h"

#include "robot.h"

void Sensor_Init(void);
int Sensor_GetState(int i);
int Sensor_GetStateCached(int i);
int Sensor_GetQuantizedPos(RobotState *rs);
void Sensor_ShowDebug(RobotState *rs);

#endif
