#include "MPU6050_MSPM0.h"
#include "OLED.h"
#include "Motor.h"
#include "PID.h"

#include "robot.h"

/* tick_ms 来自 SysTick，测试函数需要 */
extern volatile unsigned long tick_ms;

/* ---------------- MPU6050 registers ---------------- */
#define MPU6050_ADDR_7BIT (0x68u)
#define MPU6050_ADDR_W ((uint8_t)(MPU6050_ADDR_7BIT << 1))
#define MPU6050_ADDR_R ((uint8_t)((MPU6050_ADDR_7BIT << 1) | 0x01u))

#define MPU6050_REG_SMPLRT_DIV (0x19u)
#define MPU6050_REG_CONFIG (0x1Au)
#define MPU6050_REG_GYRO_CONFIG (0x1Bu)
#define MPU6050_REG_PWR_MGMT_1 (0x6Bu)
#define MPU6050_REG_WHO_AM_I (0x75u)
#define MPU6050_REG_GYRO_ZOUT_H (0x47u)

/* Gyro full-scale = +/- 250 dps => 131 LSB/(deg/s) */
#define MPU6050_GYRO_LSB_PER_DPS (131.0f)

/* 移植自 ICM-42686 方案的滤波参数 */
#define MPU6050_GYRO_FILTER_ALPHA (0.7f) /* 低通滤波系数：0.7 轻度滤波，自适应零偏处理慢漂 */

/* ---------------- Bit-banged I2C on SysConfig OLED pins ----------------
 * This project currently configures OLED pins as GPIO; there is no DriverLib I2C instance.
 * We reuse the same SDA/SCL pins (OLED_Pin_*) so MPU6050 can share the I2C bus with the OLED.
 */
#ifndef MPU6050_I2C_DELAY_CYCLES
#define MPU6050_I2C_DELAY_CYCLES (120u)
#endif

static inline void mpu_i2c_delay(void)
{
    delay_cycles(MPU6050_I2C_DELAY_CYCLES);
}

#define MPU_I2C_PORT (MPU6050_PORT)
#define MPU_SDA_PIN (OLED_Pin_SDA_PIN)
#define MPU_SCL_PIN (OLED_Pin_SCL_PIN)
#define MPU_SDA_IOMUX (OLED_Pin_SDA_IOMUX)
#define MPU_SCL_IOMUX (OLED_Pin_SCL_IOMUX)

static inline void mpu_scl_write(uint8_t level)
{
    if (level != 0u)
    {
        DL_GPIO_setPins(MPU_I2C_PORT, MPU_SCL_PIN);
    }
    else
    {
        DL_GPIO_clearPins(MPU_I2C_PORT, MPU_SCL_PIN);
    }
}

static inline void mpu_sda_drive_low(void)
{
    DL_GPIO_clearPins(MPU_I2C_PORT, MPU_SDA_PIN);
    DL_GPIO_enableOutput(MPU_I2C_PORT, MPU_SDA_PIN);
}

static inline void mpu_sda_release(void)
{
    /* High-Z, rely on pull-up */
    DL_GPIO_disableOutput(MPU_I2C_PORT, MPU_SDA_PIN);
}

static inline void mpu_sda_write(uint8_t level)
{
    if (level != 0u)
    {
        mpu_sda_release();
    }
    else
    {
        mpu_sda_drive_low();
    }
}

static inline uint8_t mpu_sda_read(void)
{
    return (DL_GPIO_readPins(MPU_I2C_PORT, MPU_SDA_PIN) != 0u) ? 1u : 0u;
}

static void mpu_i2c_bus_init(void)
{
    /* SDA as input w/ pull-up so we can release it; SCL as output */
    DL_GPIO_initDigitalInputFeatures(MPU_SDA_IOMUX,
                                     DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalOutput(MPU_SCL_IOMUX);

    DL_GPIO_enableOutput(MPU_I2C_PORT, MPU_SCL_PIN);

    /* Idle: both lines high */
    mpu_sda_release();
    mpu_scl_write(1u);
    mpu_i2c_delay();
}

static void mpu_i2c_start(void)
{
    mpu_sda_write(1u);
    mpu_scl_write(1u);
    mpu_i2c_delay();
    mpu_sda_write(0u);
    mpu_i2c_delay();
    mpu_scl_write(0u);
    mpu_i2c_delay();
}

static void mpu_i2c_stop(void)
{
    mpu_sda_write(0u);
    mpu_i2c_delay();
    mpu_scl_write(1u);
    mpu_i2c_delay();
    mpu_sda_write(1u);
    mpu_i2c_delay();
}

static bool mpu_i2c_write_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8u; i++)
    {
        mpu_sda_write((byte & (0x80u >> i)) != 0u);
        mpu_i2c_delay();
        mpu_scl_write(1u);
        mpu_i2c_delay();
        mpu_scl_write(0u);
        mpu_i2c_delay();
    }

    /* ACK bit */
    mpu_sda_release();
    mpu_i2c_delay();
    mpu_scl_write(1u);
    mpu_i2c_delay();
    uint8_t ack = mpu_sda_read(); /* 0 = ACK */
    mpu_scl_write(0u);
    mpu_i2c_delay();

    return (ack == 0u);
}

static uint8_t mpu_i2c_read_byte(bool nack)
{
    uint8_t byte = 0u;

    mpu_sda_release();
    for (uint8_t i = 0; i < 8u; i++)
    {
        mpu_scl_write(1u);
        mpu_i2c_delay();
        if (mpu_sda_read() != 0u)
        {
            byte |= (uint8_t)(0x80u >> i);
        }
        mpu_scl_write(0u);
        mpu_i2c_delay();
    }

    /* ACK/NACK */
    mpu_sda_write(nack ? 1u : 0u);
    mpu_i2c_delay();
    mpu_scl_write(1u);
    mpu_i2c_delay();
    mpu_scl_write(0u);
    mpu_i2c_delay();
    mpu_sda_release();

    return byte;
}

static bool mpu_write_reg(uint8_t reg, uint8_t val)
{
    mpu_i2c_start();
    if (!mpu_i2c_write_byte(MPU6050_ADDR_W))
    {
        mpu_i2c_stop();
        return false;
    }
    if (!mpu_i2c_write_byte(reg))
    {
        mpu_i2c_stop();
        return false;
    }
    if (!mpu_i2c_write_byte(val))
    {
        mpu_i2c_stop();
        return false;
    }
    mpu_i2c_stop();
    return true;
}

static bool mpu_read_bytes(uint8_t startReg, uint8_t *buf, uint8_t len)
{
    if ((buf == NULL) || (len == 0u))
    {
        return false;
    }

    mpu_i2c_start();
    if (!mpu_i2c_write_byte(MPU6050_ADDR_W))
    {
        mpu_i2c_stop();
        return false;
    }
    if (!mpu_i2c_write_byte(startReg))
    {
        mpu_i2c_stop();
        return false;
    }

    mpu_i2c_start();
    if (!mpu_i2c_write_byte(MPU6050_ADDR_R))
    {
        mpu_i2c_stop();
        return false;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        bool nack = (i == (uint8_t)(len - 1u));
        buf[i] = mpu_i2c_read_byte(nack);
    }

    mpu_i2c_stop();
    return true;
}

static bool mpu_read_u8(uint8_t reg, uint8_t *val)
{
    return mpu_read_bytes(reg, val, 1u);
}

static bool mpu_read_i16(uint8_t regHigh, int16_t *val)
{
    uint8_t b[2];
    if (!mpu_read_bytes(regHigh, b, 2u))
    {
        return false;
    }

    *val = (int16_t)((((uint16_t)b[0]) << 8) | (uint16_t)b[1]);
    return true;
}

bool MPU6050_Init(MPU6050_Handle *dev)
{
    if (dev == NULL)
    {
        return false;
    }

    mpu_i2c_bus_init();

    /* Reset device */
    (void)mpu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80u);
    Delay_ms(100u);

    /* Wake up, use PLL with X gyro as clock */
    if (!mpu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01u))
    {
        return false;
    }

    /* Sample rate: 1kHz / (1 + SMPLRT_DIV). Set to 100Hz => div=9 */
    (void)mpu_write_reg(MPU6050_REG_SMPLRT_DIV, 9u);

    /* DLPF: 42Hz (CONFIG=3) gives decent noise reduction for steering angle */
    (void)mpu_write_reg(MPU6050_REG_CONFIG, 0x03u);

    /* Gyro FS: +/- 250 dps */
    (void)mpu_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00u);

    /* WHO_AM_I */
    uint8_t who = 0u;
    if (!mpu_read_u8(MPU6050_REG_WHO_AM_I, &who))
    {
        return false;
    }
    if ((who & 0x7Fu) != MPU6050_ADDR_7BIT)
    {
        return false;
    }

    dev->gyroZBias_dps = 0.0f;
    dev->yaw_deg = 0.0f;
    dev->gyroZFilt_dps = 0.0f;
    dev->prevRawGyroZ = 0;
    dev->lastUpdateMs = tick_ms;
    dev->samplePeriodMs = 10u; /* default: 100Hz */

    return true;
}

bool MPU6050_CalibrateGyroZ(MPU6050_Handle *dev, uint16_t samples, uint16_t intervalMs)
{
    if ((dev == NULL) || (samples == 0u))
    {
        return false;
    }

    float sum_dps = 0.0f;
    for (uint16_t i = 0; i < samples; i++)
    {
        int16_t gz_raw = 0;
        if (!mpu_read_i16(MPU6050_REG_GYRO_ZOUT_H, &gz_raw))
        {
            return false;
        }
        sum_dps += ((float)gz_raw) / MPU6050_GYRO_LSB_PER_DPS;
        Delay_ms(intervalMs);
    }

    dev->gyroZBias_dps = sum_dps / (float)samples;

    /* Reset integration origin after calibration */
    dev->yaw_deg = 0.0f;
    dev->gyroZFilt_dps = 0.0f;
    dev->prevRawGyroZ = 0;
    dev->lastUpdateMs = tick_ms;
    return true;
}

bool MPU6050_UpdateYaw(MPU6050_Handle *dev, uint32_t nowMs)
{
    if (dev == NULL)
    {
        return false;
    }

    uint32_t elapsedMs = nowMs - dev->lastUpdateMs;
    if (elapsedMs < dev->samplePeriodMs)
    {
        return false;
    }

    /* 钳位 dt：仅在超长时间未更新时保护（>500ms），避免异常跳变。
     * 日常主循环即使因 OLED 刷新较慢（100-200ms），也不应截断积分时间，
     * 否则会累积出成倍的角度欠计数。自适应零偏跟踪已处理静止漂移。 */
    if (elapsedMs > 500u)
    {
        elapsedMs = 500u;
    }

    int16_t gz_raw = 0;
    if (!mpu_read_i16(MPU6050_REG_GYRO_ZOUT_H, &gz_raw))
    {
        return false;
    }

    /* 尖峰检测：电机 EMI 可能干扰 bit-banged I2C，导致单帧读数跳变。
     * 与上一帧有效值对比，超过阈值则丢弃当帧，也不更新时间戳，
     * 使下一帧能累积完整的时间差。 */
    if (dev->prevRawGyroZ != 0)
    {
        int16_t diff = gz_raw - dev->prevRawGyroZ;
        if (diff > MPU6050_SPIKE_THRESHOLD_LSB ||
            diff < -MPU6050_SPIKE_THRESHOLD_LSB)
        {
            return false;
        }
    }
    dev->prevRawGyroZ = gz_raw;

    /* 仅在读取成功后更新时间戳，避免 I2C 失败导致时间空洞 */
    dev->lastUpdateMs = nowMs;

    float gz_dps = ((float)gz_raw) / MPU6050_GYRO_LSB_PER_DPS;

    /* LPF 低通滤波：衰减电机高频振动 */
    dev->gyroZFilt_dps = MPU6050_GYRO_FILTER_ALPHA * gz_dps +
                         (1.0f - MPU6050_GYRO_FILTER_ALPHA) * dev->gyroZFilt_dps;

    /* 去零偏后的角速度 */
    float gyro_z_err = dev->gyroZFilt_dps - dev->gyroZBias_dps;

    /*
     * 自适应零偏跟踪（移植自 ICM-42686 方案）：
     * 当 |gyro_z_err| < BIAS_TRACK_DPS 时，认为小车没有在主动转向，
     * 此时缓慢将零偏估计值向当前读数收敛，自动补偿温度漂移。
     * 注意：用滤波后的值判断，避免振动误触发偏置更新。
     */
    if (gyro_z_err > -MPU6050_GYRO_Z_BIAS_TRACK_DPS &&
        gyro_z_err < MPU6050_GYRO_Z_BIAS_TRACK_DPS)
    {
        dev->gyroZBias_dps = (1.0f - MPU6050_BIAS_ADAPT_ALPHA) * dev->gyroZBias_dps +
                             MPU6050_BIAS_ADAPT_ALPHA * dev->gyroZFilt_dps;
    }

    /* 死区：滤除残余噪声，0.15 dps 仅挡硬件底噪，不伤转向信号 */
    if (gyro_z_err > -MPU6050_GYRO_Z_DEADBAND_DPS &&
        gyro_z_err < MPU6050_GYRO_Z_DEADBAND_DPS)
    {
        gyro_z_err = 0.0f;
    }

    float dt_s = ((float)elapsedMs) / 1000.0f;
    dev->yaw_deg += gyro_z_err * dt_s;

    /* 限制 yaw 在 ±180 度范围内 */
    if (dev->yaw_deg > 180.0f)
        dev->yaw_deg -= 360.0f;
    if (dev->yaw_deg < -180.0f)
        dev->yaw_deg += 360.0f;

    return true;
}

/* ================================================================
 * 单元测试：MPU6050 + TurnToAngle 原地转向
 *
 * 用法：在 main() 的 while(1) 中只调用本函数。
 *       while (1) { Test_MPU6050_TurnToAngle(); }
 *
 * 行为：车原地转到 +90°，到位后转到 -90°，如此循环。
 *       OLED 显示 目标(Tgt) / 当前(Yaw) / 误差(Err)。
 *       到位判定：|Err| < 5 度。
 * ================================================================ */
void Test_MPU6050_TurnToAngle(RobotState *rs)
{
    static int16_t target = 90;
    float err;

    MPU6050_UpdateYaw(&rs->imu, tick_ms);
    rs->yaw = MPU6050_GetYawDeg(&rs->imu);

    err = Angle_Normalize((float)target - rs->yaw);

    if (err < 5.0f && err > -5.0f)
        target = -target;

    if (rs->imuOk)
        TurnToAngle(rs, (float)target);
    else
        Set_PWM(0, 0);

    if (!OLED_IsBusy())
    {
        int16_t y = (int16_t)rs->yaw;
        int16_t e = (int16_t)err;
        OLED_Clear();
        OLED_ShowString(0, 0, "== TURN TEST ==", OLED_8X16);
        OLED_ShowString(0, 16, "Tgt:", OLED_8X16);
        OLED_ShowSignedNum(32, 16, target, 4, OLED_8X16);
        OLED_ShowString(0, 32, "Yaw:", OLED_8X16);
        OLED_ShowSignedNum(32, 32, y, 4, OLED_8X16);
        OLED_ShowString(0, 48, "Err:", OLED_8X16);
        OLED_ShowSignedNum(32, 48, e, 4, OLED_8X16);
        OLED_Update();
    }
    OLED_Task();
}
