#ifndef MPU6050_MSPM0_H
#define MPU6050_MSPM0_H

#include "robot.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* Initializes GPIO-based (bit-banged) I2C on the configured pins and configures MPU6050 registers.
     * Returns false if WHO_AM_I check fails.
     */
    bool MPU6050_Init(MPU6050_Handle *dev);

    /* Calibrates gyro Z bias while the device is stationary.
     * samples: number of samples to average.
     * intervalMs: delay between samples.
     */
    bool MPU6050_CalibrateGyroZ(MPU6050_Handle *dev, uint16_t samples, uint16_t intervalMs);

    /* Updates yaw by reading GYRO_ZOUT and integrating.
     * Call periodically with current tick in ms (e.g. tick_ms).
     * Internally rate-limited by dev->samplePeriodMs.
     */
    bool MPU6050_UpdateYaw(MPU6050_Handle *dev, uint32_t nowMs);

    static inline float MPU6050_GetYawDeg(const MPU6050_Handle *dev)
    {
        return dev->yaw_deg;
    }

    static inline void MPU6050_ResetYaw(MPU6050_Handle *dev)
    {
        dev->yaw_deg = 0.0f;
    }

#ifdef __cplusplus
}
#endif

#endif
