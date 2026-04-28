#include "MPU6050_MSPM0.h"

#include "ti_msp_dl_config.h"
#include "Delay.h"

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

    dev->lastUpdateMs = nowMs;

    int16_t gz_raw = 0;
    if (!mpu_read_i16(MPU6050_REG_GYRO_ZOUT_H, &gz_raw))
    {
        return false;
    }

    float gz_dps = ((float)gz_raw) / MPU6050_GYRO_LSB_PER_DPS;
    float dt_s = ((float)elapsedMs) / 1000.0f;

    /* 添加死区过滤：小于 ±0.05 dps 的漂移认为是噪声 */
    float drift_dps = gz_dps - dev->gyroZBias_dps;
    if (drift_dps > -0.5f && drift_dps < 0.5f)
    {
        drift_dps = 0.0f;
    }

    dev->yaw_deg += drift_dps * dt_s;

    /* 限制 yaw 在 ±180 度范围内 */
    if (dev->yaw_deg > 180.0f)
        dev->yaw_deg -= 360.0f;
    if (dev->yaw_deg < -180.0f)
        dev->yaw_deg += 360.0f;

    return true;
}
