/*
 * MPU-6050 attitude estimation module for pendulum angle measurement.
 *
 * Hardware: LP-MSPM0G3507, I2C1 PB2=SCL PB3=SDA, MPU address 0x68.
 *
 * Usage:
 *   mpu6050_init();                        // probe, wake, config
 *   attitude_init(&filter, 0.98f);         // alpha = 0.98 (typical)
 *
 *   // Bias calibration: hold pendulum STILL at 0° for 2+ seconds.
 *   attitude_calibrate_start(&filter);
 *   for (int i = 0; i < 200; i++) {        // 200 samples @ 100Hz = 2s
 *       mpu6050_read_raw(&raw);
 *       attitude_calibrate_sample(&filter, raw.gyro_axis_dps);
 *       delay_10ms();
 *   }
 *   attitude_calibrate_finish(&filter);
 *
 *   // Runtime: call at 100Hz
 *   mpu6050_read_raw(&raw);
 *   float accel_angle = accel_to_pendulum_angle(raw.ax_g, raw.az_g);
 *   attitude_update(&filter, 0.01f, accel_angle, raw.gyro_axis_dps);
 *   float pipe_deg = attitude_get_angle(&filter);
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- MPU-6050 raw readings (physical units) ---- */
typedef struct {
    float ax_g, ay_g, az_g;        /* accelerometer, g  (1 g = 9.81 m/s^2) */
    float gx_dps, gy_dps, gz_dps;  /* gyroscope, deg/s                    */
    float temp_c;                   /* temperature, Celsius (approx)       */
} MpuRaw;

/* ---- Attitude filter state ---- */
typedef struct {
    /* Configuration */
    float alpha;                    /* complementary filter coefficient (0.90-0.99) */
    int16_t bias_samples_needed;    /* how many samples to calibrate              */
    bool   calibrated;              /* true after bias calibration finishes        */

    /* Gyro bias calibration state */
    float  gyro_bias_dps;           /* calibrated gyro zero-offset (deg/s)         */
    float  bias_accum;              /* running sum during calibration              */
    int16_t bias_count;             /* samples collected so far                    */

    /* Filter state */
    float angle_deg;                /* current filtered pendulum angle (deg)       */
} AttitudeFilter;

/* ---- MPU-6050 I2C driver ---- */

/* Probe (0x68 then 0x69), verify WHO_AM_I, wake, configure ±2g / ±250dps / 200Hz. */
bool mpu6050_init(void);

/* Read 14 bytes starting at ACCEL_XOUT_H, convert to physical units.
 * Returns false on I2C error (caller should NOT use stale data for control). */
bool mpu6050_read_raw(MpuRaw *raw);

/* ---- Attitude filter ---- */

/* Initialise filter state.  alpha is typically 0.95-0.99:
 *   - Higher alpha trusts gyro more (smoother, but drifts if bias is off)
 *   - Lower alpha trusts accel more (less drift, but vibration-sensitive)
 *   - 0.98 is a safe starting point for a pendulum.
 */
void attitude_init(AttitudeFilter *filt, float alpha);

/* Start gyro bias calibration.  Pendulum MUST be stationary at 0 deg. */
void attitude_calibrate_start(AttitudeFilter *filt);

/* Feed one gyro reading during the calibration window.
 * Call at fixed interval (e.g. 100Hz) for the requested number of samples. */
void attitude_calibrate_sample(AttitudeFilter *filt, float gyro_axis_dps);

/* Finish calibration and apply the averaged bias.
 * Returns false if not enough samples were collected. */
bool attitude_calibrate_finish(AttitudeFilter *filt);

/* Run one step of the complementary filter.
 *   dt_s           - time delta in seconds (0.010 for 100Hz)
 *   accel_angle_deg - angle derived from accelerometer gravity vector
 *   gyro_dps       - rotation rate around the hinge axis (deg/s)
 */
void attitude_update(AttitudeFilter *filt, float dt_s,
                     float accel_angle_deg, float gyro_dps);

/* Return the current filtered angle in degrees. */
float attitude_get_angle(const AttitudeFilter *filt);

/* ---- Pendulum-specific: accelerometer → angle ----
 *
 * Converts raw accelerometer readings to a pendulum tilt angle.
 * ASSUMES: MPU is rigidly mounted on the pendulum arm such that:
 *   - The hinge (rotation) axis is parallel to MPU Y axis
 *   - X points along the pendulum arm (away from hinge)
 *   - Z points perpendicular to the arm ("up" when arm is horizontal)
 *
 * With this mounting: angle = atan2(ax, az)
 *   - ax = +1g when pendulum is vertical (arm hanging down)   → ~90°
 *   - az = +1g when pendulum is horizontal (arm level)        → ~0°
 *
 * INVERT the sign if your physical mounting differs.
 * VERIFY: at 0° pendulum, the result should be close to 0.
 *         At +8° (right side up), result should be ~+8 (or ~ -8, then flip sign).
 */
float accel_to_pendulum_angle(float ax_g, float az_g);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
