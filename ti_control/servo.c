#include "servo.h"

#include "system_time.h"
#include "ti_msp_dl_config.h"

#define SERVO_PIN_IOMUX                          IOMUX_PINCM19
#define SERVO_PIN_MASK                           DL_GPIO_PIN_8

#define SERVO_DEFAULT_ANGLE_DEG                  0
#define SERVO_MIN_ANGLE_DEG                      0
#define SERVO_MAX_ANGLE_DEG                      180

#define SERVO_MIN_PULSE_US                       1000U
#define SERVO_MAX_PULSE_US                       2000U
#define SERVO_PWM_PERIOD_MS                      20U
#define SERVO_CYCLES_PER_US                      (CPUCLK_FREQ / 1000000U)

static bool gServoAttached = false;
static int16_t gServoAngleDeg = SERVO_DEFAULT_ANGLE_DEG;
static uint32_t gLastServoPulseMs = 0U;

static int16_t Servo_ClampAngle(int16_t angleDeg)
{
    if (angleDeg < SERVO_MIN_ANGLE_DEG) {
        return SERVO_MIN_ANGLE_DEG;
    }

    if (angleDeg > SERVO_MAX_ANGLE_DEG) {
        return SERVO_MAX_ANGLE_DEG;
    }

    return angleDeg;
}

static uint32_t Servo_AngleToPulseUs(int16_t angleDeg)
{
    uint32_t spanUs = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;

    angleDeg = Servo_ClampAngle(angleDeg);
    return SERVO_MIN_PULSE_US +
           (uint32_t)((spanUs * (uint32_t)angleDeg) / 180U);
}

static void Servo_SetPinLow(void)
{
    DL_GPIO_clearPins(GPIOA, SERVO_PIN_MASK);
}

static void Servo_SendPulse(void)
{
    uint32_t pulseUs = Servo_AngleToPulseUs(gServoAngleDeg);

    DL_GPIO_setPins(GPIOA, SERVO_PIN_MASK);
    delay_cycles(pulseUs * SERVO_CYCLES_PER_US);
    Servo_SetPinLow();
}

void Servo_Init(void)
{
    DL_GPIO_enablePower(GPIOA);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_GPIO_initDigitalOutput(SERVO_PIN_IOMUX);
    DL_GPIO_enableOutput(GPIOA, SERVO_PIN_MASK);
    Servo_Detach();
}

void Servo_Attach(void)
{
    gServoAttached = true;
    gLastServoPulseMs = 0U;
}

void Servo_Detach(void)
{
    gServoAttached = false;
    gLastServoPulseMs = 0U;
    Servo_SetPinLow();
}

void Servo_SetAngle(int16_t angleDeg)
{
    gServoAngleDeg = Servo_ClampAngle(angleDeg);
}

void Servo_Task(void)
{
    uint32_t nowMs;

    if (!gServoAttached) {
        return;
    }

    nowMs = SystemTime_Millis();
    if ((gLastServoPulseMs == 0U) ||
        ((nowMs - gLastServoPulseMs) >= SERVO_PWM_PERIOD_MS)) {
        gLastServoPulseMs = nowMs;
        Servo_SendPulse();
    }
}

bool Servo_IsAttached(void)
{
    return gServoAttached;
}

void Servo_GetStatus(ServoStatus *status)
{
    if (status == 0) {
        return;
    }

    status->attached = gServoAttached;
    status->angleDeg = gServoAngleDeg;
}
