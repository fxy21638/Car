#ifndef MPU6050_MSPM0_H
#define MPU6050_MSPM0_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float gyroZBias_dps;        /* 零偏 (°/s)，运行中自适应更新 */
        float yaw_deg;              /* 累计航向角 (°) */

        float gyroZFilt_dps;        /* 低通滤波后的陀螺仪 Z 轴角速度 */

        uint32_t lastUpdateMs;
        uint16_t samplePeriodMs;
        int16_t prevRawGyroZ;       /* 上一帧原始陀螺仪 Z 值，用于尖峰检测 */
    } MPU6050_Handle;

    /* 自适应零偏跟踪参数（移植自 ICM-42686 方案） */
    #define MPU6050_GYRO_Z_DEADBAND_DPS      (0.15f) /* 死区：低于此值视为噪声，不积分 */
    #define MPU6050_GYRO_Z_BIAS_TRACK_DPS    (0.80f) /* 偏置跟踪阈值：低于此值认为"不在转向"，更新零偏 */
    #define MPU6050_BIAS_ADAPT_ALPHA         (0.01f) /* 零偏适应速率：越小跟踪越慢，越稳定 */

    /* 尖峰检测：相邻两次原始读数相差超过此阈值视为 I2C 干扰，丢弃当帧。
     * 100 dps × 131 LSB/(dps) ≈ 13100 LSB。小车物理上不可能在 10ms 内角速度突变 100 dps。 */
    #define MPU6050_SPIKE_THRESHOLD_LSB       (13100)

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
