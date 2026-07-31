#include "line_follow.h"

#include "encoder.h"
#include "line_follow_config.h"
#include "line_sensor.h"
#include "motor.h"
#include "system_time.h"

#define LINE_POSITION_MAX 3500

static bool gLineFollowActive = false;
static bool gHasValidLine = false;
static uint32_t gLastControlMs = 0U;
static uint32_t gStartMs = 0U;
static uint32_t gLastValidLineMs = 0U;
static uint32_t gLastEdgeTurnMs = 0U;
static uint32_t gWheelBalanceStartMs = 0U;
static uint32_t gWheelBalanceLastStepMs = 0U;
static int16_t gCurrentTurnFine = 0;
static int16_t gLastEdgeTurnTicks = 0;
static int16_t gWheelBalanceTurnFine = 0;
static int8_t gWheelBalanceFasterSide = 0;
static uint32_t gLastTurnSlewMs = 0U;
static int16_t gCurrentLeftTarget = 0;
static int16_t gCurrentRightTarget = 0;
static int32_t gLeftTargetFineRemainder = 0;
static int32_t gRightTargetFineRemainder = 0;
static LineFollowStatus gStatus = {0};

static int16_t LineFollow_Abs16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static void LineFollow_ResetWheelBalance(void)
{
    gWheelBalanceStartMs = 0U;
    gWheelBalanceLastStepMs = 0U;
    gWheelBalanceTurnFine = 0;
    gWheelBalanceFasterSide = 0;
}

static int16_t LineFollow_ClampTrackSpeed(int32_t value)
{
    if (value > LINE_MAX_SPEED_TICKS) {
        return LINE_MAX_SPEED_TICKS;
    }

    if (value < LINE_MIN_TRACK_SPEED_TICKS) {
        return LINE_MIN_TRACK_SPEED_TICKS;
    }

    return (int16_t)value;
}

static int16_t LineFollow_SlewTarget(int16_t current, int16_t target)
{
    if (target > (int16_t)(current + LINE_TARGET_SLEW_TICKS)) {
        return (int16_t)(current + LINE_TARGET_SLEW_TICKS);
    }

    if (target < (int16_t)(current - LINE_TARGET_SLEW_TICKS)) {
        return (int16_t)(current - LINE_TARGET_SLEW_TICKS);
    }

    return target;
}

static int16_t LineFollow_ClampTrackTargetFine(int32_t value)
{
    int32_t maxValue = (int32_t)LINE_MAX_SPEED_TICKS * LINE_TURN_SCALE;

    if (value > maxValue) {
        return (int16_t)maxValue;
    }

    if (value < 0) {
        return 0;
    }

    return (int16_t)value;
}

static int16_t LineFollow_TargetFromFine(int16_t targetFine, int32_t *remainder)
{
    int32_t accumulated = *remainder + targetFine;
    int16_t target = (int16_t)(accumulated / LINE_TURN_SCALE);

    *remainder = accumulated - ((int32_t)target * LINE_TURN_SCALE);
    return LineFollow_ClampTrackSpeed(target);
}

static int16_t LineFollow_SlewTurnFine(int16_t targetFine, uint32_t nowMs)
{
    int16_t slewFine = LINE_TURN_SLEW_FINE;

    if ((targetFine != gCurrentTurnFine) &&
        ((gLastTurnSlewMs == 0U) ||
         ((nowMs - gLastTurnSlewMs) >= LINE_TURN_SLEW_PERIOD_MS))) {
        gLastTurnSlewMs = nowMs;

        if (LineFollow_Abs16(targetFine) <= LINE_CENTER_RELEASE_FINE) {
            slewFine = LINE_CENTER_RELEASE_SLEW_FINE;
        }

        if (((targetFine > 0) && (gCurrentTurnFine < 0)) ||
            ((targetFine < 0) && (gCurrentTurnFine > 0))) {
            gCurrentTurnFine = 0;
        }

        if (targetFine > (int16_t)(gCurrentTurnFine + slewFine)) {
            gCurrentTurnFine = (int16_t)(gCurrentTurnFine + slewFine);
        } else if (targetFine <
                   (int16_t)(gCurrentTurnFine - slewFine)) {
            gCurrentTurnFine = (int16_t)(gCurrentTurnFine - slewFine);
        } else {
            gCurrentTurnFine = targetFine;
        }
    }

    return gCurrentTurnFine;
}

static void LineFollow_ResetMotionState(void)
{
    gLastEdgeTurnMs = 0U;
    LineFollow_ResetWheelBalance();
    gCurrentTurnFine = 0;
    gLastEdgeTurnTicks = 0;
    gLastTurnSlewMs = 0U;
    gCurrentLeftTarget = 0;
    gCurrentRightTarget = 0;
    gLeftTargetFineRemainder = 0;
    gRightTargetFineRemainder = 0;
}

static bool LineFollow_IsStopSample(
    const LineSensorSample *sample, uint32_t nowMs)
{
    if ((nowMs - gStartMs) < LINE_STOP_ENABLE_DELAY_MS) {
        return false;
    }

    return sample->allBlack ||
           (sample->activeCount >= LINE_STOP_BLACK_COUNT);
}

static int16_t LineFollow_GetEdgeTurnTicks(const LineSensorSample *sample)
{
    bool leftEdge = ((sample->blackMask & LINE_EDGE_LEFT_MASK) != 0U);
    bool rightEdge = ((sample->blackMask & LINE_EDGE_RIGHT_MASK) != 0U);

    if (leftEdge && !rightEdge) {
        return (int16_t)-LINE_EDGE_TURN_TICKS;
    }

    if (rightEdge && !leftEdge) {
        return LINE_EDGE_TURN_TICKS;
    }

    return 0;
}

static int16_t LineFollow_GetTurnCommand(
    const LineSensorSample *sample, uint32_t nowMs)
{
    int16_t turnTicks = LineFollow_GetEdgeTurnTicks(sample);

    if (turnTicks != 0) {
        gLastEdgeTurnMs = nowMs;
        gLastEdgeTurnTicks = turnTicks;
        return turnTicks;
    }

    if ((gLastEdgeTurnMs != 0U) &&
        ((nowMs - gLastEdgeTurnMs) <= LINE_EDGE_HOLD_MS)) {
        return gLastEdgeTurnTicks;
    }

    return 0;
}

static int16_t LineFollow_GetWheelBalanceTurnFine(uint32_t nowMs)
{
#if LINE_WHEEL_BALANCE_ENABLE
    MotorStatus motorStatus;
    int16_t speedDiff;
    int8_t fasterSide;

    if ((gCurrentLeftTarget < LINE_WHEEL_BALANCE_MIN_TARGET_TICKS) ||
        (gCurrentRightTarget < LINE_WHEEL_BALANCE_MIN_TARGET_TICKS)) {
        gWheelBalanceStartMs = 0U;
        gWheelBalanceLastStepMs = 0U;
        gWheelBalanceFasterSide = 0;
        return 0;
    }

    Motor_GetStatus(&motorStatus);
    speedDiff = (int16_t)(motorStatus.leftMeasured - motorStatus.rightMeasured);

    if (speedDiff > LINE_WHEEL_BALANCE_DIFF_TICKS) {
        fasterSide = 1;
    } else if (speedDiff < (int16_t)-LINE_WHEEL_BALANCE_DIFF_TICKS) {
        fasterSide = -1;
    } else {
        if ((gWheelBalanceLastStepMs == 0U) ||
            ((nowMs - gWheelBalanceLastStepMs) >=
             LINE_WHEEL_BALANCE_STEP_PERIOD_MS)) {
            gWheelBalanceLastStepMs = nowMs;
            if (gWheelBalanceTurnFine > 0) {
                gWheelBalanceTurnFine -= LINE_WHEEL_BALANCE_DECAY_FINE;
                if (gWheelBalanceTurnFine < 0) {
                    gWheelBalanceTurnFine = 0;
                }
            } else if (gWheelBalanceTurnFine < 0) {
                gWheelBalanceTurnFine += LINE_WHEEL_BALANCE_DECAY_FINE;
                if (gWheelBalanceTurnFine > 0) {
                    gWheelBalanceTurnFine = 0;
                }
            }
        }
        gWheelBalanceStartMs = 0U;
        gWheelBalanceFasterSide = 0;
        return gWheelBalanceTurnFine;
    }

    if (fasterSide != gWheelBalanceFasterSide) {
        gWheelBalanceFasterSide = fasterSide;
        gWheelBalanceStartMs = nowMs;
        return 0;
    }

    if ((nowMs - gWheelBalanceStartMs) < LINE_WHEEL_BALANCE_HOLD_MS) {
        return gWheelBalanceTurnFine;
    }

    if ((gWheelBalanceLastStepMs != 0U) &&
        ((nowMs - gWheelBalanceLastStepMs) <
         LINE_WHEEL_BALANCE_STEP_PERIOD_MS)) {
        return gWheelBalanceTurnFine;
    }

    if (fasterSide > 0) {
        if (gWheelBalanceTurnFine > (int16_t)-LINE_WHEEL_BALANCE_MAX_FINE) {
            gWheelBalanceTurnFine -= LINE_WHEEL_BALANCE_STEP_FINE;
        }
    } else if (gWheelBalanceTurnFine < LINE_WHEEL_BALANCE_MAX_FINE) {
        gWheelBalanceTurnFine += LINE_WHEEL_BALANCE_STEP_FINE;
    }

    if (gWheelBalanceTurnFine > LINE_WHEEL_BALANCE_MAX_FINE) {
        gWheelBalanceTurnFine = LINE_WHEEL_BALANCE_MAX_FINE;
    } else if (gWheelBalanceTurnFine <
               (int16_t)-LINE_WHEEL_BALANCE_MAX_FINE) {
        gWheelBalanceTurnFine = (int16_t)-LINE_WHEEL_BALANCE_MAX_FINE;
    }

    gWheelBalanceLastStepMs = nowMs;
    return gWheelBalanceTurnFine;
#else
    (void)nowMs;
    return 0;
#endif
}

static void LineFollow_SetTargetsFromFine(
    int16_t baseSpeed,
    int16_t turnFine,
    int16_t leftTrimFine,
    int16_t rightTrimFine,
    int16_t *leftTarget,
    int16_t *rightTarget)
{
    int16_t leftTargetFine = LineFollow_ClampTrackTargetFine(
        ((int32_t)baseSpeed * LINE_TURN_SCALE) + leftTrimFine + turnFine);
    int16_t rightTargetFine = LineFollow_ClampTrackTargetFine(
        ((int32_t)baseSpeed * LINE_TURN_SCALE) + rightTrimFine - turnFine);

    *leftTarget =
        LineFollow_TargetFromFine(leftTargetFine, &gLeftTargetFineRemainder);
    *rightTarget =
        LineFollow_TargetFromFine(rightTargetFine, &gRightTargetFineRemainder);
}

static void LineFollow_UpdateStatusFromMotor(void)
{
    MotorStatus motorStatus;

    Motor_GetStatus(&motorStatus);
    gStatus.leftTarget = motorStatus.leftTarget;
    gStatus.rightTarget = motorStatus.rightTarget;
    gStatus.leftMeasured = motorStatus.leftMeasured;
    gStatus.rightMeasured = motorStatus.rightMeasured;
    gStatus.leftPwm = motorStatus.leftPwm;
    gStatus.rightPwm = motorStatus.rightPwm;
}

void LineFollow_Init(void)
{
    LineSensor_Init();
    Encoder_Init();
    Motor_Init();
    LineFollow_Stop();
}

void LineFollow_Start(void)
{
    gLineFollowActive = true;
    gHasValidLine = false;
    gLastControlMs = SystemTime_Millis();
    gStartMs = gLastControlMs;
    gLastValidLineMs = gLastControlMs;
    LineFollow_ResetMotionState();
    gStatus.active = true;
    gStatus.lineLost = false;
    gStatus.allBlack = false;
    gStatus.sensorMask = 0U;
    gStatus.lineError = 0;
    gStatus.turn = 0;
    Encoder_Reset();
    Motor_ResetController();
    Motor_SetSpeedTargets(0, 0);
}

void LineFollow_Stop(void)
{
    gLineFollowActive = false;
    gHasValidLine = false;
    Motor_Stop();
    LineFollow_ResetMotionState();
    gStatus.active = false;
    gStatus.lineLost = false;
    gStatus.allBlack = false;
    gStatus.turn = 0;
    LineFollow_UpdateStatusFromMotor();
}

void LineFollow_Task(void)
{
    LineSensorSample sample;
    uint32_t nowMs;
    uint32_t elapsedMs;
    int16_t turnFine = gCurrentTurnFine;
    int16_t leftTarget = gCurrentLeftTarget;
    int16_t rightTarget = gCurrentRightTarget;

    if (!gLineFollowActive) {
        return;
    }

    nowMs = SystemTime_Millis();
    elapsedMs = nowMs - gLastControlMs;

    if (elapsedMs >= LINE_CONTROL_PERIOD_MS) {
        gLastControlMs = nowMs;
        LineSensor_Read(&sample);

        gStatus.sensorMask = sample.blackMask;
        gStatus.lineLost = sample.lineLost;
        gStatus.allBlack = sample.allBlack;

        if (sample.lineLost) {
            if (gHasValidLine &&
                ((nowMs - gLastValidLineMs) <= LINE_LOST_HOLD_MS)) {
                int16_t lostTurnFine =
                    (int16_t)(LINE_LOST_TURN_TICKS * LINE_TURN_SCALE);

                if (gLastEdgeTurnTicks < 0) {
                    lostTurnFine = (int16_t)-lostTurnFine;
                } else if (gLastEdgeTurnTicks == 0) {
                    lostTurnFine = gCurrentTurnFine;
                }

                turnFine = LineFollow_SlewTurnFine(lostTurnFine, nowMs);
                LineFollow_SetTargetsFromFine(
                    LINE_LOST_BASE_SPEED_TICKS,
                    turnFine,
                    0,
                    0,
                    &leftTarget,
                    &rightTarget);
            } else {
                leftTarget = 0;
                rightTarget = 0;
                turnFine = 0;
                gHasValidLine = false;
                LineFollow_ResetMotionState();
            }
        } else if (LineFollow_IsStopSample(&sample, nowMs)) {
            leftTarget = 0;
            rightTarget = 0;
            turnFine = 0;
            gHasValidLine = false;
            LineFollow_ResetMotionState();
            gStatus.lineError = 0;
        } else {
            int16_t turnCommand = LineFollow_GetTurnCommand(&sample, nowMs);
            int16_t lineError = 0;
            int16_t baseSpeed = LINE_STRAIGHT_SPEED_TICKS;
            int16_t leftTrimFine = LINE_LEFT_BASE_TRIM_FINE;
            int16_t rightTrimFine = LINE_RIGHT_BASE_TRIM_FINE;

            gHasValidLine = true;
            gLastValidLineMs = nowMs;

            if (turnCommand < 0) {
                LineFollow_ResetWheelBalance();
                lineError = -LINE_POSITION_MAX;
                baseSpeed = LINE_EDGE_TURN_SPEED_TICKS;
                turnFine = LineFollow_SlewTurnFine(
                    (int16_t)(turnCommand * LINE_TURN_SCALE), nowMs);
            } else if (turnCommand > 0) {
                LineFollow_ResetWheelBalance();
                lineError = LINE_POSITION_MAX;
                baseSpeed = LINE_EDGE_TURN_SPEED_TICKS;
                turnFine = LineFollow_SlewTurnFine(
                    (int16_t)(turnCommand * LINE_TURN_SCALE), nowMs);
            } else {
                turnFine = LineFollow_SlewTurnFine(
                    LineFollow_GetWheelBalanceTurnFine(nowMs), nowMs);
            }

            LineFollow_SetTargetsFromFine(
                baseSpeed,
                turnFine,
                leftTrimFine,
                rightTrimFine,
                &leftTarget,
                &rightTarget);
            gStatus.lineError = lineError;
        }

        gStatus.turn = (int16_t)(turnFine / LINE_TURN_SCALE);
        gCurrentLeftTarget = LineFollow_SlewTarget(gCurrentLeftTarget, leftTarget);
        gCurrentRightTarget =
            LineFollow_SlewTarget(gCurrentRightTarget, rightTarget);
        Motor_SetSpeedTargets(gCurrentLeftTarget, gCurrentRightTarget);
    }

    Motor_Task(nowMs);
    LineFollow_UpdateStatusFromMotor();
}

bool LineFollow_IsActive(void)
{
    return gLineFollowActive;
}

void LineFollow_GetStatus(LineFollowStatus *status)
{
    LineFollow_UpdateStatusFromMotor();
    *status = gStatus;
}
