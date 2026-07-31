#include "motor.h"

#include <stdbool.h>

#include "encoder.h"
#include "line_follow_config.h"
#include "ti_msp_dl_config.h"

typedef struct {
    int32_t integral;
    int16_t target;
    int16_t measured;
    int16_t filteredMeasured;
    int16_t pwm;
    int16_t minPwm;
    int16_t pwmBias;
} WheelController;

static WheelController gLeftWheel = {
    0, 0, 0, 0, 0, MOTOR_LEFT_MIN_PWM, MOTOR_LEFT_PWM_BIAS
};
static WheelController gRightWheel = {
    0, 0, 0, 0, 0, MOTOR_RIGHT_MIN_PWM, MOTOR_RIGHT_PWM_BIAS
};
static uint32_t gLastMotorUpdateMs = 0U;

static const DL_TimerG_ClockConfig gMotorTimerClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U,
};

static const DL_TimerG_PWMConfig gMotorPwmConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = MOTOR_PWM_PERIOD,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

static int16_t Motor_ClampInt16(int32_t value, int32_t minValue, int32_t maxValue)
{
    if (value > maxValue) {
        return (int16_t)maxValue;
    }

    if (value < minValue) {
        return (int16_t)minValue;
    }

    return (int16_t)value;
}

static int16_t Motor_ClampTarget(int16_t target)
{
    return (target < 0) ? 0 : target;
}

static int16_t Motor_SlewPwm(int16_t current, int16_t target)
{
    if (target > (int16_t)(current + MOTOR_PWM_SLEW_STEP)) {
        return (int16_t)(current + MOTOR_PWM_SLEW_STEP);
    }

    if (target < (int16_t)(current - MOTOR_PWM_SLEW_STEP)) {
        return (int16_t)(current - MOTOR_PWM_SLEW_STEP);
    }

    return target;
}

static int16_t Motor_FilterMeasured(WheelController *wheel, int16_t rawMeasured)
{
    int32_t filtered = wheel->filteredMeasured;

    filtered += ((int32_t)rawMeasured - filtered) /
                (int32_t)(1U << MOTOR_MEASURE_FILTER_SHIFT);
    wheel->filteredMeasured = (int16_t)filtered;

    return wheel->filteredMeasured;
}

static void Motor_InitPwmTimer(GPTIMER_Regs *timer)
{
    DL_TimerG_reset(timer);
    DL_TimerG_enablePower(timer);

    DL_TimerG_setClockConfig(timer, (DL_TimerG_ClockConfig *) &gMotorTimerClockConfig);
    DL_TimerG_initPWMMode(timer, (DL_TimerG_PWMConfig *) &gMotorPwmConfig);
    DL_TimerG_setCounterControl(timer,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(timer, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(timer,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(timer, 0U, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(timer, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
        DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptCompUpdateMethod(timer,
        DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE,
        DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(timer, 0U, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(timer);
    DL_TimerG_setCCPDirection(timer, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(timer);
}

static void Motor_SetTimerDuty(
    GPTIMER_Regs *timer, DL_TIMER_CC_INDEX channel, uint16_t duty)
{
    if (duty > MOTOR_PWM_PERIOD) {
        duty = MOTOR_PWM_PERIOD;
    }

    DL_TimerG_setCaptureCompareValue(timer, duty, channel);
}

static void Motor_SetBridgePwm(
    GPTIMER_Regs *timer, int16_t pwm, bool invertDirection)
{
    uint16_t duty;

    if (pwm < 0) {
        pwm = 0;
    }

    duty = (uint16_t)pwm;
    if (duty == 0U) {
        Motor_SetTimerDuty(timer, DL_TIMER_CC_0_INDEX, 0U);
        Motor_SetTimerDuty(timer, DL_TIMER_CC_1_INDEX, 0U);
    } else if (!invertDirection) {
        Motor_SetTimerDuty(timer, DL_TIMER_CC_0_INDEX, duty);
        Motor_SetTimerDuty(timer, DL_TIMER_CC_1_INDEX, 0U);
    } else {
        Motor_SetTimerDuty(timer, DL_TIMER_CC_0_INDEX, 0U);
        Motor_SetTimerDuty(timer, DL_TIMER_CC_1_INDEX, duty);
    }
}

static void Motor_SetLeftPwm(int16_t pwm)
{
    Motor_SetBridgePwm(TIMG0, pwm, (MOTOR_LEFT_DIRECTION_INVERT != 0));
}

static void Motor_SetRightPwm(int16_t pwm)
{
    Motor_SetBridgePwm(TIMG7, pwm, (MOTOR_RIGHT_DIRECTION_INVERT != 0));
}

static int16_t Motor_UpdateWheel(WheelController *wheel, int16_t measured)
{
    int32_t target = Motor_ClampTarget(wheel->target);
    int32_t error;
    int32_t feedForward;
    int32_t output;

    measured = Motor_FilterMeasured(wheel, measured);
    wheel->measured = measured;

    if (target == 0) {
        wheel->integral = 0;
        wheel->filteredMeasured = 0;
        wheel->pwm = 0;
        return 0;
    }

    error = target - measured;
    wheel->integral += error * MOTOR_SPEED_KI_NUM;
    if (wheel->integral > MOTOR_SPEED_INTEGRAL_LIMIT) {
        wheel->integral = MOTOR_SPEED_INTEGRAL_LIMIT;
    } else if (wheel->integral < -MOTOR_SPEED_INTEGRAL_LIMIT) {
        wheel->integral = -MOTOR_SPEED_INTEGRAL_LIMIT;
    }

    feedForward = MOTOR_PWM_FEED_FORWARD +
                  (target * MOTOR_PWM_PER_TICK) +
                  wheel->pwmBias;
    output = feedForward +
             ((MOTOR_SPEED_KP_NUM * error) / MOTOR_SPEED_GAIN_DEN) +
             (wheel->integral / MOTOR_SPEED_GAIN_DEN);

    output = Motor_ClampInt16(output, 0, MOTOR_PWM_MAX);

    if ((output > 0) && (output < wheel->minPwm)) {
        output = wheel->minPwm;
    }

    wheel->pwm = Motor_SlewPwm(wheel->pwm, (int16_t)output);
    return wheel->pwm;
}

void Motor_Init(void)
{
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM34,
        IOMUX_PINCM34_PF_TIMG0_CCP0);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM35,
        IOMUX_PINCM35_PF_TIMG0_CCP1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_12 | DL_GPIO_PIN_13);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM3,
        IOMUX_PINCM3_PF_TIMG7_CCP0);
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM6,
        IOMUX_PINCM6_PF_TIMG7_CCP1);
    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_28 | DL_GPIO_PIN_31);

    Motor_InitPwmTimer(TIMG0);
    Motor_InitPwmTimer(TIMG7);
    Motor_Stop();
}

void Motor_ResetController(void)
{
    gLeftWheel.integral = 0;
    gRightWheel.integral = 0;
    gLeftWheel.measured = 0;
    gRightWheel.measured = 0;
    gLeftWheel.filteredMeasured = 0;
    gRightWheel.filteredMeasured = 0;
    gLeftWheel.pwm = 0;
    gRightWheel.pwm = 0;
    gLastMotorUpdateMs = 0U;
}

void Motor_SetSpeedTargets(int16_t leftTarget, int16_t rightTarget)
{
    gLeftWheel.target = Motor_ClampTarget(leftTarget);
    gRightWheel.target = Motor_ClampTarget(rightTarget);
}

void Motor_Task(uint32_t nowMs)
{
    EncoderDelta delta;
    uint32_t elapsedMs;
    int16_t leftMeasured;
    int16_t rightMeasured;

    if (gLastMotorUpdateMs == 0U) {
        gLastMotorUpdateMs = nowMs;
        return;
    }

    elapsedMs = nowMs - gLastMotorUpdateMs;
    if (elapsedMs < MOTOR_CONTROL_PERIOD_MS) {
        return;
    }

    gLastMotorUpdateMs = nowMs;
    Encoder_GetAndResetDeltas(&delta);

    leftMeasured = (int16_t)(((int32_t)delta.left * MOTOR_CONTROL_PERIOD_MS) /
                             (int32_t)elapsedMs);
    rightMeasured = (int16_t)(((int32_t)delta.right * MOTOR_CONTROL_PERIOD_MS) /
                              (int32_t)elapsedMs);

    Motor_SetLeftPwm(Motor_UpdateWheel(&gLeftWheel, leftMeasured));
    Motor_SetRightPwm(Motor_UpdateWheel(&gRightWheel, rightMeasured));
}

void Motor_Stop(void)
{
    gLeftWheel.target = 0;
    gRightWheel.target = 0;
    Motor_ResetController();
    Motor_SetLeftPwm(0);
    Motor_SetRightPwm(0);
}

void Motor_GetStatus(MotorStatus *status)
{
    status->leftTarget = gLeftWheel.target;
    status->rightTarget = gRightWheel.target;
    status->leftMeasured = gLeftWheel.measured;
    status->rightMeasured = gRightWheel.measured;
    status->leftPwm = gLeftWheel.pwm;
    status->rightPwm = gRightWheel.pwm;
}
