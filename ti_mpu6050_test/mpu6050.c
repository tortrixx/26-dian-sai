/*
 * MPU-6050 attitude estimation — implementation.
 *
 * Dependencies: ti_msp_dl_config.h (SysConfig-generated)
 *               math.h (atan2f, sqrtf — linked via -llibc.a)
 */

#include "mpu6050.h"
#include "ti_msp_dl_config.h"
#include <math.h>

/* ================================================================
 *  MPU-6050 register map (used internally)
 * ================================================================ */
#define MPU_ADDR_LOW         0x68u
#define MPU_ADDR_HIGH        0x69u
#define REG_ACCEL_XOUT_H     0x3Bu
#define REG_SMPLRT_DIV       0x19u
#define REG_CONFIG           0x1Au
#define REG_GYRO_CONFIG      0x1Bu
#define REG_ACCEL_CONFIG     0x1Cu
#define REG_PWR_MGMT_1       0x6Bu
#define REG_WHO_AM_I         0x75u

/* Full-scale sensitivities (hardware-configured below) */
#define ACCEL_SENSITIVITY    16384.0f   /* LSB/g   for ±2g  */
#define GYRO_SENSITIVITY     131.0f     /* LSB/°/s for ±250°/s */
#define TEMP_OFFSET          36.53f     /* datasheet */
#define TEMP_SENSITIVITY     340.0f     /* LSB/°C */

/* ================================================================
 *  I2C helpers (private — same as original test)
 * ================================================================ */
#define I2C_TIMEOUT_LOOPS    1000000u

static bool _i2c_wait_idle(void)
{
    uint32_t to = I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0u) {
        if (--to == 0u) return false;
    }
    return true;
}

static bool _i2c_wait_done(void)
{
    uint32_t to = I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0u) {
        if (--to == 0u) return false;
    }
    return (DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) == 0u;
}

static bool _i2c_write(uint8_t addr, const uint8_t *data, uint8_t len)
{
    if (!_i2c_wait_idle()) return false;
    DL_I2C_flushControllerTXFIFO(I2C_INST);
    if (DL_I2C_fillControllerTXFIFO(I2C_INST, data, len) != len) return false;
    DL_I2C_startControllerTransfer(I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, len);
    delay_cycles(1000);
    return _i2c_wait_done();
}

static bool _i2c_read(uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t i;
    if (!_i2c_wait_idle()) return false;
    DL_I2C_startControllerTransfer(I2C_INST, addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, len);
    for (i = 0u; i < len; ++i) {
        uint32_t to = I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST)) {
            if (--to == 0u) return false;
        }
        data[i] = DL_I2C_receiveControllerData(I2C_INST);
    }
    return _i2c_wait_done();
}

static bool _mpu_reg_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    return _i2c_write(addr, &reg, 1u) && _i2c_read(addr, data, len);
}

static bool _mpu_reg_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return _i2c_write(addr, buf, 2u);
}

static int16_t _be_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* ================================================================
 *  Public: MPU-6050 driver
 * ================================================================ */

static uint8_t _mpu_addr = 0u;   /* resolved I2C address, 0 if init failed */

bool mpu6050_init(void)
{
    uint8_t who = 0u;
    uint8_t addr = MPU_ADDR_LOW;

    /* Probe primary address */
    if (!_mpu_reg_read(addr, REG_WHO_AM_I, &who, 1u)) {
        /* Try alternate */
        addr = MPU_ADDR_HIGH;
        if (!_mpu_reg_read(addr, REG_WHO_AM_I, &who, 1u)) {
            _mpu_addr = 0u;
            return false;
        }
    }

    if (who != MPU_ADDR_LOW) {
        _mpu_addr = 0u;
        return false;
    }

    /* Wake + configure: ±2g, ±250°/s, DLPF=3 (44Hz), 200Hz sample rate */
    if (!_mpu_reg_write(addr, REG_PWR_MGMT_1,   0x00u)) { _mpu_addr = 0u; return false; }
    if (!_mpu_reg_write(addr, REG_SMPLRT_DIV,   4u))    { _mpu_addr = 0u; return false; }
    if (!_mpu_reg_write(addr, REG_CONFIG,       0x03u)) { _mpu_addr = 0u; return false; }
    if (!_mpu_reg_write(addr, REG_GYRO_CONFIG,  0x00u)) { _mpu_addr = 0u; return false; }
    if (!_mpu_reg_write(addr, REG_ACCEL_CONFIG, 0x00u)) { _mpu_addr = 0u; return false; }

    _mpu_addr = addr;
    return true;
}

bool mpu6050_read_raw(MpuRaw *raw)
{
    uint8_t buf[14];

    if (_mpu_addr == 0u) return false;
    if (!_mpu_reg_read(_mpu_addr, REG_ACCEL_XOUT_H, buf, sizeof(buf)))
        return false;

    raw->ax_g  = _be_i16(&buf[0])  / ACCEL_SENSITIVITY;
    raw->ay_g  = _be_i16(&buf[2])  / ACCEL_SENSITIVITY;
    raw->az_g  = _be_i16(&buf[4])  / ACCEL_SENSITIVITY;
    raw->temp_c = (_be_i16(&buf[6]) - TEMP_OFFSET) / TEMP_SENSITIVITY + 36.53f;
    raw->gx_dps = _be_i16(&buf[8])  / GYRO_SENSITIVITY;
    raw->gy_dps = _be_i16(&buf[10]) / GYRO_SENSITIVITY;
    raw->gz_dps = _be_i16(&buf[12]) / GYRO_SENSITIVITY;

    return true;
}

/* ================================================================
 *  Public: attitude filter
 * ================================================================ */

void attitude_init(AttitudeFilter *filt, float alpha)
{
    filt->alpha               = alpha;
    filt->bias_samples_needed = 200;   /* 2 seconds @ 100Hz */
    filt->calibrated          = false;
    filt->gyro_bias_dps       = 0.0f;
    filt->bias_accum          = 0.0f;
    filt->bias_count          = 0;
    filt->angle_deg           = 0.0f;
}

void attitude_calibrate_start(AttitudeFilter *filt)
{
    filt->bias_accum = 0.0f;
    filt->bias_count = 0;
    filt->calibrated = false;
}

void attitude_calibrate_sample(AttitudeFilter *filt, float gyro_axis_dps)
{
    if (filt->calibrated) return;
    if (filt->bias_count >= filt->bias_samples_needed) return;
    filt->bias_accum += gyro_axis_dps;
    filt->bias_count++;
}

bool attitude_calibrate_finish(AttitudeFilter *filt)
{
    if (filt->bias_count < 50) {   /* need at least 0.5s of data */
        return false;
    }
    filt->gyro_bias_dps = filt->bias_accum / (float)filt->bias_count;
    filt->angle_deg     = 0.0f;    /* assume pendulum was at 0° */
    filt->calibrated    = true;
    return true;
}

void attitude_update(AttitudeFilter *filt, float dt_s,
                     float accel_angle_deg, float gyro_dps)
{
    float gyro_corrected;
    float gyro_angle;

    if (!filt->calibrated) {
        /* Fallback: just use accelerometer */
        filt->angle_deg = accel_angle_deg;
        return;
    }

    /* Remove bias */
    gyro_corrected = gyro_dps - filt->gyro_bias_dps;

    /* Integrate gyro */
    gyro_angle = filt->angle_deg + gyro_corrected * dt_s;

    /* Complementary filter: high-pass gyro, low-pass accel */
    filt->angle_deg = filt->alpha * gyro_angle
                    + (1.0f - filt->alpha) * accel_angle_deg;
}

float attitude_get_angle(const AttitudeFilter *filt)
{
    return filt->angle_deg;
}

/* ================================================================
 *  Public: accelerometer → pendulum angle
 * ================================================================ */

float accel_to_pendulum_angle(float ax_g, float az_g)
{
    /* atan2(ax, az):  ax = horizontal component (gravity along arm)
     *                 az = vertical component (gravity perpendicular to arm)
     *
     * When pendulum is level (0°):  ax ≈ 0,   az ≈ +1g  → angle ≈ 0°
     * When pendulum tilted +8°:     ax ≈ sin(8°), az ≈ cos(8°) → angle ≈ +8°
     */
    return atan2f(ax_g, az_g) * 57.29578f;   /* rad → deg */
}
