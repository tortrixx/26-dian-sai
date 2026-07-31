#include "static_ball.h"

#include "k230_uart.h"
#include "servo.h"
#include "system_time.h"

#define STATIC_BALL_CONTROL_PERIOD_MS            10U
#define STATIC_BALL_VISION_TIMEOUT_MS            200U
#define STATIC_BALL_MIN_QUALITY                  1U
#define STATIC_BALL_INVALID_LIMIT                3U

#define STATIC_BALL_POS_TARGET_CM_X100           500
#define STATIC_BALL_NEG_TARGET_CM_X100           (-500)
#define STATIC_BALL_ARRIVE_BAND_CM_X100          50
#define STATIC_BALL_DIRECTION_DEADBAND_CM_X100   20

/* Tune these first if the ball moves the wrong way or the beam is not level. */
#define STATIC_BALL_SERVO_NEUTRAL_DEG            90
#define STATIC_BALL_SERVO_DIRECTION              (-1.0f)
#define STATIC_BALL_SERVO_DEG_PER_TILT_DEG       1.0f

/* Simple bang-bang control: only decide which side of the target the ball is on. */
#define STATIC_BALL_MOVE_TILT_DEG                12
#define STATIC_BALL_HOLD_TILT_DEG                8

typedef enum {
    STATIC_BALL_PHASE_WAIT_VISION = 0,
    STATIC_BALL_PHASE_TO_POS,
    STATIC_BALL_PHASE_TO_NEG,
    STATIC_BALL_PHASE_HOLD_NEG
} StaticBallPhase;

static bool gStaticBallActive = false;
static bool gStaticBallServoEnabled = false;
static StaticBallPhase gPhase = STATIC_BALL_PHASE_WAIT_VISION;
static uint32_t gLastControlMs = 0U;
static uint8_t gInvalidCount = 0U;
static StaticBallStatus gStatus = {
    false, false, false, 0U, 0U, 0, STATIC_BALL_POS_TARGET_CM_X100, 0, 0,
    STATIC_BALL_SERVO_NEUTRAL_DEG, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U
};

static int16_t StaticBall_ClampServoAngle(int16_t angleDeg)
{
    if (angleDeg < 0) {
        return 0;
    }

    if (angleDeg > 180) {
        return 180;
    }

    return angleDeg;
}

static uint32_t StaticBall_ElapsedMs(uint32_t nowMs, uint32_t thenMs)
{
    return nowMs - thenMs;
}

static int16_t StaticBall_TiltToServoAngle(int16_t tiltDeg)
{
    float servoAngle = (float)STATIC_BALL_SERVO_NEUTRAL_DEG +
        (STATIC_BALL_SERVO_DIRECTION * (float)tiltDeg *
         STATIC_BALL_SERVO_DEG_PER_TILT_DEG);

    return StaticBall_ClampServoAngle((int16_t)(servoAngle + 0.5f));
}

static void StaticBall_ResetController(void)
{
    gLastControlMs = 0U;
    gInvalidCount = 0U;
}

static void StaticBall_ResetSequence(void)
{
    gPhase = STATIC_BALL_PHASE_WAIT_VISION;
    gStatus.phase = (uint8_t)gPhase;
    gStatus.targetCmX100 = STATIC_BALL_POS_TARGET_CM_X100;
}

static void StaticBall_SetServoNeutral(bool attach)
{
    gStatus.tiltDegX10 = 0;
    gStatus.servoAngleDeg = STATIC_BALL_SERVO_NEUTRAL_DEG;
    Servo_SetAngle(STATIC_BALL_SERVO_NEUTRAL_DEG);
    if (attach) {
        Servo_Attach();
        gStaticBallServoEnabled = true;
    }
}

static void StaticBall_DisableServo(void)
{
    Servo_Detach();
    gStaticBallServoEnabled = false;
}

static void StaticBall_EnableServo(void)
{
    if (!gStaticBallServoEnabled) {
        Servo_Attach();
        gStaticBallServoEnabled = true;
    }
}

static int16_t StaticBall_GetTargetX100(void)
{
    if ((gPhase == STATIC_BALL_PHASE_WAIT_VISION) ||
        (gPhase == STATIC_BALL_PHASE_TO_POS)) {
        return STATIC_BALL_POS_TARGET_CM_X100;
    }

    return STATIC_BALL_NEG_TARGET_CM_X100;
}

static bool StaticBall_ReachedPosTarget(int16_t ballX100)
{
    return ballX100 >=
        (STATIC_BALL_POS_TARGET_CM_X100 - STATIC_BALL_ARRIVE_BAND_CM_X100);
}

static bool StaticBall_ReachedNegTarget(int16_t ballX100)
{
    return ballX100 <=
        (STATIC_BALL_NEG_TARGET_CM_X100 + STATIC_BALL_ARRIVE_BAND_CM_X100);
}

static void StaticBall_UpdateSequence(int16_t ballX100)
{
    switch (gPhase) {
        case STATIC_BALL_PHASE_WAIT_VISION:
            gPhase = STATIC_BALL_PHASE_TO_POS;
            break;
        case STATIC_BALL_PHASE_TO_POS:
            if (StaticBall_ReachedPosTarget(ballX100)) {
                gPhase = STATIC_BALL_PHASE_TO_NEG;
            }
            break;
        case STATIC_BALL_PHASE_TO_NEG:
            if (StaticBall_ReachedNegTarget(ballX100)) {
                gPhase = STATIC_BALL_PHASE_HOLD_NEG;
            }
            break;
        case STATIC_BALL_PHASE_HOLD_NEG:
        default:
            break;
    }

    gStatus.phase = (uint8_t)gPhase;
    gStatus.targetCmX100 = StaticBall_GetTargetX100();
}

void StaticBall_Init(void)
{
    gStaticBallActive = false;
    gStaticBallServoEnabled = false;
    StaticBall_ResetController();
    StaticBall_ResetSequence();

    gStatus.active = false;
    gStatus.visionFresh = false;
    gStatus.visionValid = false;
    gStatus.phase = (uint8_t)gPhase;
    gStatus.quality = 0U;
    gStatus.ballXCmX100 = 0;
    gStatus.targetCmX100 = STATIC_BALL_POS_TARGET_CM_X100;
    gStatus.velocityCmSX100 = 0;
    StaticBall_SetServoNeutral(false);
    StaticBall_DisableServo();
}

void StaticBall_Start(void)
{
    gStaticBallActive = true;
    gStaticBallServoEnabled = false;
    StaticBall_ResetController();
    StaticBall_ResetSequence();

    gStatus.active = true;
    gStatus.visionFresh = false;
    gStatus.visionValid = false;
    gStatus.phase = (uint8_t)gPhase;
    gStatus.quality = 0U;
    gStatus.ballXCmX100 = 0;
    gStatus.targetCmX100 = STATIC_BALL_POS_TARGET_CM_X100;
    gStatus.velocityCmSX100 = 0;

    StaticBall_SetServoNeutral(false);
    StaticBall_DisableServo();
}

void StaticBall_Stop(void)
{
    gStaticBallActive = false;

    gStatus.active = false;
    gStatus.visionFresh = false;
    gStatus.visionValid = false;
    StaticBall_ResetController();
    StaticBall_ResetSequence();
    StaticBall_DisableServo();
}

void StaticBall_Exit(void)
{
    gStaticBallActive = false;

    gStatus.active = false;
    gStatus.visionFresh = false;
    gStatus.visionValid = false;
    StaticBall_ResetController();
    StaticBall_ResetSequence();
    StaticBall_DisableServo();
}

void StaticBall_Task(void)
{
    K230VisionFrame frame;
    K230UartStatus uartStatus;
    uint32_t nowMs = SystemTime_Millis();
    bool hasFrame;
    bool frameFresh;
    bool frameAccepted;
    int16_t targetX100;
    int16_t tiltDeg;
    int16_t servoAngleDeg;

    K230Uart_GetStatus(&uartStatus);
    gStatus.rxBytes = uartStatus.rxBytes;
    gStatus.rxOk = uartStatus.rxOk;
    gStatus.rxBad = uartStatus.rxBad;
    gStatus.rxHeadAA = uartStatus.rxHeadAA;
    gStatus.rxHeadAA55 = uartStatus.rxHeadAA55;
    gStatus.lastByte = uartStatus.lastByte;
    gStatus.lastLength = uartStatus.lastLength;
    gStatus.lastType = uartStatus.lastType;
    gStatus.lastChecksumRx = uartStatus.lastChecksumRx;
    gStatus.lastChecksumCalc = uartStatus.lastChecksumCalc;

    if (!gStaticBallActive) {
        return;
    }

    if ((gLastControlMs != 0U) &&
        (StaticBall_ElapsedMs(nowMs, gLastControlMs) < STATIC_BALL_CONTROL_PERIOD_MS)) {
        return;
    }

    gLastControlMs = nowMs;

    hasFrame = K230Uart_GetLatest(&frame);
    frameFresh = hasFrame &&
        (StaticBall_ElapsedMs(nowMs, frame.receivedMs) <= STATIC_BALL_VISION_TIMEOUT_MS);
    frameAccepted = frameFresh && frame.valid &&
        (frame.quality >= STATIC_BALL_MIN_QUALITY);

    gStatus.visionFresh = frameFresh;
    gStatus.visionValid = frameAccepted;
    gStatus.phase = (uint8_t)gPhase;
    if (hasFrame) {
        gStatus.quality = frame.quality;
        gStatus.ballXCmX100 = frame.xCmX100;
    }

    if (!frameAccepted) {
        if (gInvalidCount < 255U) {
            gInvalidCount++;
        }
        if (gInvalidCount >= STATIC_BALL_INVALID_LIMIT) {
            StaticBall_DisableServo();
        }
        return;
    }

    StaticBall_EnableServo();
    gInvalidCount = 0U;
    gStatus.velocityCmSX100 = 0;

    StaticBall_UpdateSequence(frame.xCmX100);

    targetX100 = gStatus.targetCmX100;
    if (frame.xCmX100 < (targetX100 - STATIC_BALL_DIRECTION_DEADBAND_CM_X100)) {
        tiltDeg = (gPhase == STATIC_BALL_PHASE_HOLD_NEG) ?
            STATIC_BALL_HOLD_TILT_DEG : STATIC_BALL_MOVE_TILT_DEG;
    } else if (frame.xCmX100 >
        (targetX100 + STATIC_BALL_DIRECTION_DEADBAND_CM_X100)) {
        tiltDeg = (gPhase == STATIC_BALL_PHASE_HOLD_NEG) ?
            -STATIC_BALL_HOLD_TILT_DEG : -STATIC_BALL_MOVE_TILT_DEG;
    } else {
        tiltDeg = 0;
    }

    servoAngleDeg = StaticBall_TiltToServoAngle(tiltDeg);
    Servo_SetAngle(servoAngleDeg);

    gStatus.tiltDegX10 = (int16_t)(tiltDeg * 10);
    gStatus.servoAngleDeg = servoAngleDeg;
}

bool StaticBall_IsActive(void)
{
    return gStaticBallActive;
}

void StaticBall_GetStatus(StaticBallStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = gStatus;
}
