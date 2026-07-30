/*
 * Standalone MPU-6050 electrical/I2C test for LP-MSPM0G3507.
 *
 * Wiring:
 *   LaunchPad 3V3 -> MPU VCC, GND -> MPU GND,
 *   PB2/I2C1_SCL -> MPU SCL, PB3/I2C1_SDA -> MPU SDA.
 *
 * Open the XDS110 Application/User UART (COM5) at 115200 baud.  This program
 * never drives the servo, motor or K230 UART pins.
 */

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"

#define MPU6050_ADDRESS_LOW       0x68u
#define MPU6050_ADDRESS_HIGH      0x69u
#define MPU6050_REG_ACCEL_XOUT_H  0x3Bu
#define MPU6050_REG_SMPLRT_DIV    0x19u
#define MPU6050_REG_CONFIG        0x1Au
#define MPU6050_REG_GYRO_CONFIG   0x1Bu
#define MPU6050_REG_ACCEL_CONFIG  0x1Cu
#define MPU6050_REG_PWR_MGMT_1    0x6Bu
#define MPU6050_REG_WHO_AM_I      0x75u

#define I2C_TIMEOUT_LOOPS         1000000u

static void uart_write_char(char value)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t) value);
}

static void uart_write_text(const char *text)
{
    while (*text != '\0') {
        uart_write_char(*text++);
    }
}

static void uart_write_u16(uint16_t value)
{
    char digits[5];
    uint8_t count = 0u;

    if (value == 0u) {
        uart_write_char('0');
        return;
    }
    while (value > 0u) {
        digits[count++] = (char) ('0' + (value % 10u));
        value /= 10u;
    }
    while (count > 0u) {
        uart_write_char(digits[--count]);
    }
}

static void uart_write_i16(int16_t value)
{
    int32_t expanded = value;
    if (expanded < 0) {
        uart_write_char('-');
        expanded = -expanded;
    }
    uart_write_u16((uint16_t) expanded);
}

static void uart_write_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_write_char(hex[(value >> 4) & 0x0Fu]);
    uart_write_char(hex[value & 0x0Fu]);
}

static bool i2c_wait_for_idle(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }
    return true;
}

static bool i2c_wait_for_done(void)
{
    uint32_t timeout = I2C_TIMEOUT_LOOPS;
    while ((DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY) != 0u) {
        if (--timeout == 0u) {
            return false;
        }
    }
    return (DL_I2C_getControllerStatus(I2C_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) == 0u;
}

static bool i2c_write(uint8_t address, const uint8_t *data, uint8_t length)
{
    if (!i2c_wait_for_idle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_INST);
    if (DL_I2C_fillControllerTXFIFO(I2C_INST, data, length) != length) {
        return false;
    }
    DL_I2C_startControllerTransfer(I2C_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(1000);
    return i2c_wait_for_done();
}

static bool i2c_read(uint8_t address, uint8_t *data, uint8_t length)
{
    uint8_t index;
    if (!i2c_wait_for_idle()) {
        return false;
    }

    DL_I2C_startControllerTransfer(I2C_INST, address,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);
    for (index = 0u; index < length; ++index) {
        uint32_t timeout = I2C_TIMEOUT_LOOPS;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST)) {
            if (--timeout == 0u) {
                return false;
            }
        }
        data[index] = DL_I2C_receiveControllerData(I2C_INST);
    }
    return i2c_wait_for_done();
}

static bool mpu_read(uint8_t address, uint8_t register_address,
                     uint8_t *data, uint8_t length)
{
    return i2c_write(address, &register_address, 1u) &&
           i2c_read(address, data, length);
}

static bool mpu_write(uint8_t address, uint8_t register_address, uint8_t value)
{
    uint8_t data[2] = {register_address, value};
    return i2c_write(address, data, 2u);
}

static int16_t read_i16_be(const uint8_t *data)
{
    return (int16_t) (((uint16_t) data[0] << 8) | data[1]);
}

static bool mpu_start(uint8_t address)
{
    uint8_t who_am_i = 0u;
    if (!mpu_read(address, MPU6050_REG_WHO_AM_I, &who_am_i, 1u)) {
        return false;
    }
    if (who_am_i != MPU6050_ADDRESS_LOW) {
        return false;
    }
    return mpu_write(address, MPU6050_REG_PWR_MGMT_1, 0x00u) &&
           mpu_write(address, MPU6050_REG_SMPLRT_DIV, 4u) &&
           mpu_write(address, MPU6050_REG_CONFIG, 0x03u) &&
           mpu_write(address, MPU6050_REG_GYRO_CONFIG, 0x00u) &&
           mpu_write(address, MPU6050_REG_ACCEL_CONFIG, 0x00u);
}

int main(void)
{
    uint8_t address = MPU6050_ADDRESS_LOW;
    uint8_t who_am_i = 0u;
    uint8_t raw[14];

    SYSCFG_DL_init();
    delay_cycles(3200000u);
    uart_write_text("\r\nMPU6050 I2C test: PB2=SCL PB3=SDA\r\n");

    if (!mpu_read(address, MPU6050_REG_WHO_AM_I, &who_am_i, 1u)) {
        address = MPU6050_ADDRESS_HIGH;
        (void) mpu_read(address, MPU6050_REG_WHO_AM_I, &who_am_i, 1u);
    }

    uart_write_text("WHO_AM_I=0x");
    uart_write_hex8(who_am_i);
    uart_write_text(" address=0x");
    uart_write_hex8(address);
    uart_write_text("\r\n");

    if (!mpu_start(address)) {
        uart_write_text("FAIL: MPU not acknowledged or unexpected ID. Check VCC/GND/SCL/SDA.\r\n");
        while (true) {
            delay_cycles(32000000u);
            uart_write_text("MPU_FAIL\r\n");
        }
    }

    uart_write_text("PASS: MPU awake. Move the board; ax/ay/az/gx/gy/gz must change.\r\n");
    while (true) {
        if (mpu_read(address, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
            uart_write_text("a=");
            uart_write_i16(read_i16_be(&raw[0]));
            uart_write_char(',');
            uart_write_i16(read_i16_be(&raw[2]));
            uart_write_char(',');
            uart_write_i16(read_i16_be(&raw[4]));
            uart_write_text(" g=");
            uart_write_i16(read_i16_be(&raw[8]));
            uart_write_char(',');
            uart_write_i16(read_i16_be(&raw[10]));
            uart_write_char(',');
            uart_write_i16(read_i16_be(&raw[12]));
            uart_write_text("\r\n");
        } else {
            uart_write_text("READ_FAIL\r\n");
        }
        delay_cycles(6400000u);
    }
}
