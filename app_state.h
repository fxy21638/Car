#ifndef __APP_STATE_H
#define __APP_STATE_H

#include "MPU6050_MSPM0.h"
#include "PID.h"
#include "ti_msp_dl_config.h"

extern PID_t leftPID;
extern PID_t rightPID;
extern PID_t steerPID;
extern PID_t anglePID;

extern int BASE_SPEED;
extern int linePos;
extern int lastSumPos;
extern int is_lost;
extern int PWMleft;
extern int PWMright;
extern int targetLeftSpeed;
extern int targetRightSpeed;
extern int leftEncSpeed;
extern int rightEncSpeed;
extern int16_t dist;
extern float yaw;
extern float origin_yaw;

extern MPU6050_Handle gImu;
extern uint8_t gMPU6050_OK;

#endif // __APP_STATE_H
