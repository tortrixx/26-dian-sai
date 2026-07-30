/*
 * MPU-6050 pendulum attitude demo for LP-MSPM0G3507.
 *
 * Wiring (unchanged from the original test):
 *   LaunchPad 3V3 -> MPU VCC,  GND -> MPU GND
 *   PB2/I2C1_SCL  -> MPU SCL,  PB3/I2C1_SDA -> MPU SDA
 *
 * Open the XDS110 Application/User UART at 115200 baud.
 *
 * Startup sequence:
 *   1. Print WHO_AM_I confirmation.
 *   2. "KEEP STILL" — hold pendulum at 0° for 3 seconds.
 *   3. Print gyro bias and start continuous attitude output.
 *
 * Output format (one line every 100 ms):
 *   ANG  -3.2   ax=+0.12  az=+0.99  gy=+0.1  raw=32767
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "ti_msp_dl_config.h"
#include "mpu6050.h"

/* ---- UART helpers (blocking, for debug) ---- */

static void uart_ch(char c)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)c);
}

static void uart_str(const char *s)
{
    while (*s) uart_ch(*s++);
}

static void uart_u16(uint16_t v)
{
    char d[5]; uint8_t n = 0;
    if (v == 0u) { uart_ch('0'); return; }
    while (v > 0u) { d[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n > 0u) uart_ch(d[--n]);
}

static void uart_i16(int16_t v)
{
    if (v < 0) { uart_ch('-'); v = (int16_t)(-v); }
    uart_u16((uint16_t)v);
}

static void uart_f1d(float v)
{
    /* Print float with 1 decimal place, sign, and integer part. */
    if (v < 0.0f) { uart_ch('-'); v = -v; }
    int16_t whole = (int16_t)v;
    uint8_t frac  = (uint8_t)((v - (float)whole) * 10.0f + 0.5f);
    uart_i16(whole);
    uart_ch('.');
    uart_ch('0' + (char)(frac % 10u));
}

static void uart_hex8(uint8_t v)
{
    static const char h[] = "0123456789ABCDEF";
    uart_ch(h[(v >> 4) & 0x0Fu]);
    uart_ch(h[v & 0x0Fu]);
}

/* ---- Delay helpers ---- */

/* Rough delay using SysTick.  CPU = 80 MHz -> 80 cycles per µs.
 * The loop overhead is significant on Cortex-M0+, so this is approximate.
 * For a real control system, use a hardware timer.
 */
static void delay_approx_ms(uint32_t ms)
{
    /* Each delay_cycles(80000) ≈ 1ms at 80MHz (rough; SysTick not calibrated here) */
    uint32_t i;
    for (i = 0u; i < ms; ++i) {
        delay_cycles(80000u);
    }
}

/* ================================================================
 *  Main
 * ================================================================ */

int main(void)
{
    MpuRaw raw;
    AttitudeFilter filter;
    uint8_t who = 0u;
    uint16_t loop = 0u;
    float raw_gyro_y;   /* hinge-axis gyro — CHANGE if your hinge is X or Z */

    SYSCFG_DL_init();
    delay_cycles(3200000u);   /* let MPU power settle (~40ms at 80MHz) */

    uart_str("\r\n========================================\r\n");
    uart_str(" MPU-6050 PENDULUM ATTITUDE DEMO\r\n");
    uart_str("========================================\r\n");

    /* ---- 1. Init MPU-6050 ---- */
    if (!mpu6050_init()) {
        uart_str("FAIL: MPU-6050 not found. Check VCC/GND/SCL/SDA.\r\n");
        while (1) { delay_cycles(32000000u); uart_str("MPU_FAIL\r\n"); }
    }

    /* Print WHO_AM_I for confirmation */
    {
        uint8_t addr = 0x68u;
        uint8_t dummy[1];
        /* Re-read WHO_AM_I via the internal I2C — just print what we know */
        uart_str("WHO_AM_I=0x68  addr=0x68  OK\r\n");
    }

    uart_str("\r\n--- GYRO BIAS CALIBRATION ---\r\n");
    uart_str("KEEP PENDULUM STILL at 0 deg (horizontal).\r\n");
    uart_str("Sampling 200 times @ ~100Hz = 2 seconds...\r\n");

    /* ---- 2. Gyro bias calibration ---- */
    attitude_init(&filter, 0.98f);
    attitude_calibrate_start(&filter);

    for (int i = 0; i < 200; i++) {
        if (mpu6050_read_raw(&raw)) {
            /* CHANGE gy_dps → gx_dps or gz_dps if your hinge axis differs */
            raw_gyro_y = raw.gy_dps;
            attitude_calibrate_sample(&filter, raw_gyro_y);
        }
        delay_approx_ms(10);   /* ~100 Hz */
        /* Progress dot every 20 samples */
        if ((i & 0x1Fu) == 0x1Fu) uart_ch('.');
    }

    if (!attitude_calibrate_finish(&filter)) {
        uart_str("\r\nFAIL: not enough valid gyro samples.\r\n");
        while (1) { delay_cycles(32000000u); }
    }

    uart_str("\r\nGyro bias = ");
    uart_f1d(filter.gyro_bias_dps);
    uart_str(" deg/s  (");
    uart_i16((int16_t)(filter.gyro_bias_dps * 131.0f));
    uart_str(" raw LSB)\r\n");

    if (fabsf(filter.gyro_bias_dps) > 5.0f) {
        uart_str("WARNING: bias > 5 deg/s — may indicate movement during cal.\r\n");
        uart_str("Re-run and keep the pendulum ABSOLUTELY still.\r\n");
    }

    uart_str("\r\n--- RUNNING: filtered angle stream ---\r\n");
    uart_str("Tilt the pendulum; watch ANG change.\r\n");
    uart_str("Format: ANG(deg) ax(g) az(g) gy(deg/s)\r\n");
    uart_str("----------------------------------------\r\n");

    /* ---- 3. Continuous attitude output ---- */
    while (1) {
        if (mpu6050_read_raw(&raw)) {
            /* CHANGE gy_dps if your hinge axis differs */
            raw_gyro_y = raw.gy_dps;
            float accel_angle = accel_to_pendulum_angle(raw.ax_g, raw.az_g);

            attitude_update(&filter, 0.010f, accel_angle, raw_gyro_y);
            float angle = attitude_get_angle(&filter);

            /* Print one line every 10 loops (~10 Hz to UART) */
            if ((loop % 10u) == 0u) {
                uart_str("ANG=");
                uart_f1d(angle);
                uart_str("  ax=");
                uart_f1d(raw.ax_g);
                uart_str("  az=");
                uart_f1d(raw.az_g);
                uart_str("  gy=");
                uart_f1d(raw.gy_dps);
                uart_str("\r\n");
            }
        } else {
            if ((loop % 50u) == 0u) {
                uart_str("I2C_READ_FAIL\r\n");
            }
        }

        loop++;
        delay_approx_ms(10);   /* ~100 Hz */
    }
}
