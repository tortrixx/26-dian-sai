#include "static_ball.h"

#include "k230_uart.h"
#include "servo.h"
#include "system_time.h"

/* ---- Task timing ---- */
#define STATIC_BALL_CONTROL_PERIOD_MS            10U
#define STATIC_BALL_VISION_TIMEOUT_MS            500U   /* allow tracker to coast */
#define STATIC_BALL_MIN_QUALITY                  1U
#define STATIC_BALL_INVALID_LIMIT                15U    /* ~150ms before detach */

/* ---- Target positions (cm × 100) ---- */
#define STATIC_BALL_POS_TARGET_CM_X100           500
#define STATIC_BALL_NEG_TARGET_CM_X100           (-500)
#define STATIC_BALL_ARRIVE_BAND_CM_X100          50    /* ±0.5 cm */

/* ----
 * Cascade PID controller — nested position & velocity loops.
 *
 *   Outer (position):  targetVel = KP_OUTER × posError      [cm/s]
 *   Inner (velocity):  tiltDeg   = KP_INNER × velError      [deg]
 *
 *   posError = targetCm - ballCm         (positive = ball left of target)
 *   velError = targetVel - measuredVel   (positive = need more speed right)
 *
 * Why cascade beats single-loop PD:
 *   - Outer loop only sees slow position dynamics → easy to tune
 *   - Inner loop only sees fast velocity dynamics → quick braking
 *   - The two loops decouple naturally; no brake-zone hacks needed
 *
 * Tuning guide (tune inner loop first!):
 *   1. Fix outer: set KP_OUTER=0, manually move ball, verify inner tracks speed
 *   2. Tune KP_INNER: increase until ball stops without oscillation
 *   3. Tune KP_OUTER: increase until ball reaches target quickly, no overshoot
 *   4. Add KI (HOLD only): just enough to cancel pipe level offset
 *
 * MOVE phase: fast approach   HOLD phase: precise positioning
 * ----
 */

/* ---- Outer loop: position → target velocity ---- */
#define SB_KP_OUTER_MOVE      2.0f    /* (cm/s)/cm  — speed per cm error */
#define SB_KP_OUTER_HOLD      1.5f    /* (cm/s)/cm  — gentler near target */
#define SB_MAX_SPEED_MOVE    15.0f    /* cm/s cap — prevent runaway */
#define SB_MAX_SPEED_HOLD     8.0f    /* cm/s cap — fine control */

/* ---- Inner loop: velocity → tilt angle ---- */
#define SB_KP_INNER_MOVE      0.5f    /* deg/(cm/s) — tilt per speed error */
#define SB_KP_INNER_HOLD      0.8f    /* deg/(cm/s) — stronger brake at hold */
#define SB_MAX_TILT_MOVE     10.0f    /* deg cap — aggressive tilt */
#define SB_MAX_TILT_HOLD      6.0f    /* deg cap — precise tilt */

/* ---- Integral: pipe level compensation (HOLD phase only) ---- */
#define SB_KI_HOLD           0.05f    /* deg/(cm*s) — weak, slow trim */
#define SB_I_LIMIT_CM_S       3.0f    /* cm*s — integral windup cap */

/* ---- Deadband ---- */
#define SB_POS_DEADBAND_CM    0.3f    /* cm — stop jitter near target */
#define SB_MIN_TILT_DEG        1.0f   /* deg — below this → send 0 */

/* ---- Servo calibration ---- */
#define STATIC_BALL_SERVO_NEUTRAL_DEG            90
#define STATIC_BALL_SERVO_DIRECTION              (-1.0f)
#define STATIC_BALL_SERVO_DEG_PER_TILT_DEG       1.0f

/* ---- Phase state machine ---- */
typedef enum {
    STATIC_BALL_PHASE_SELF_TEST = 0,   /* servo sweep test on entry */
    STATIC_BALL_PHASE_WAIT_VISION,
    STATIC_BALL_PHASE_TO_POS,
    STATIC_BALL_PHASE_TO_NEG,
    STATIC_BALL_PHASE_HOLD_NEG
} StaticBallPhase;

/* Self-test sweep sequence */
#define SELF_TEST_SWEEP_MS              800U    /* dwell at each test angle */
#define SELF_TEST_NUM_ANGLES            3U
static const int16_t gSelfTestAngles[SELF_TEST_NUM_ANGLES] = {60, 120, 90};
static uint8_t  gSelfTestStep     = 0U;
static uint32_t gSelfTestStepMs   = 0U;

/* ---- Persistent state ---- */
static bool            gActive        = false;
static bool            gServoEnabled  = false;
static StaticBallPhase gPhase         = STATIC_BALL_PHASE_WAIT_VISION;
static uint32_t        gLastControlMs = 0U;
static uint8_t         gInvalidCount  = 0U;
static float           gIntegralCmS   = 0.0f; /* cm*s — position error integral */

/* Velocity estimation (EMA-filtered) */
static float    gPrevBallCm    = 0.0f;
static bool     gPrevBallValid = false;
static uint32_t gPrevFrameMs   = 0U;
static float    gFilteredVelCmS = 0.0f;  /* cm/s, EMA α=0.3 */

/* Public status snapshot */
static StaticBallStatus gStatus = {
    false, false, false, 0U, 0U, 0, STATIC_BALL_POS_TARGET_CM_X100, 0, 0,
    STATIC_BALL_SERVO_NEUTRAL_DEG, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U
};

/* ================================================================
 *  Helpers
 * ================================================================ */

static int16_t Clamp16(int32_t value, int32_t lo, int32_t hi)
{
    if (value > hi) { return (int16_t)hi; }
    if (value < lo) { return (int16_t)lo; }
    return (int16_t)value;
}

static int16_t ServoAngleFromTilt(int16_t tiltDeg)
{
    float f = (float)STATIC_BALL_SERVO_NEUTRAL_DEG
            + STATIC_BALL_SERVO_DIRECTION * (float)tiltDeg
            * STATIC_BALL_SERVO_DEG_PER_TILT_DEG;
    return Clamp16((int32_t)(f + 0.5f), 0, 180);
}

static uint32_t ElapsedMs(uint32_t now, uint32_t then) { return now - then; }

static bool IsHoldPhase(void)
{
    return (gPhase == STATIC_BALL_PHASE_HOLD_NEG);
}

static int16_t GetTargetX100(void)
{
    if ((gPhase == STATIC_BALL_PHASE_SELF_TEST) ||
        (gPhase == STATIC_BALL_PHASE_WAIT_VISION) ||
        (gPhase == STATIC_BALL_PHASE_TO_POS)) {
        return STATIC_BALL_POS_TARGET_CM_X100;
    }
    return STATIC_BALL_NEG_TARGET_CM_X100;
}

/* ================================================================
 *  Sequence
 * ================================================================ */

static bool ReachedPos(int16_t ballX100)
{
    return ballX100 >= (STATIC_BALL_POS_TARGET_CM_X100
                        - STATIC_BALL_ARRIVE_BAND_CM_X100);
}

static bool ReachedNeg(int16_t ballX100)
{
    return ballX100 <= (STATIC_BALL_NEG_TARGET_CM_X100
                        + STATIC_BALL_ARRIVE_BAND_CM_X100);
}

static void AdvanceSequence(int16_t ballX100)
{
    switch (gPhase) {
    case STATIC_BALL_PHASE_SELF_TEST:
        /* self-test complete → wait for first valid frame */
        gPhase = STATIC_BALL_PHASE_WAIT_VISION;
        break;
    case STATIC_BALL_PHASE_WAIT_VISION:
        gPhase = STATIC_BALL_PHASE_TO_POS;
        break;
    case STATIC_BALL_PHASE_TO_POS:
        if (ReachedPos(ballX100)) {
            gIntegralCmS = 0.0f;   /* reset integrator on phase change */
            gPhase = STATIC_BALL_PHASE_TO_NEG;
        }
        break;
    case STATIC_BALL_PHASE_TO_NEG:
        if (ReachedNeg(ballX100)) {
            gIntegralCmS = 0.0f;
            gPhase = STATIC_BALL_PHASE_HOLD_NEG;
        }
        break;
    case STATIC_BALL_PHASE_HOLD_NEG:
    default:
        break;
    }
    gStatus.phase        = (uint8_t)gPhase;
    gStatus.targetCmX100 = GetTargetX100();
}

/* ================================================================
 *  Velocity estimation (EMA-filtered, called once per valid frame)
 * ================================================================ */

static void EstimateVelocity(float ballCm, uint32_t frameMs)
{
    float rawVel = 0.0f;
    int32_t dtMs;

    if (gPrevBallValid) {
        dtMs = (int32_t)(frameMs - gPrevFrameMs);
        if ((dtMs > 0) && (dtMs < 500)) {
            rawVel = (ballCm - gPrevBallCm) * 1000.0f / (float)dtMs;
        }
    }

    /* EMA low-pass: α=0.3 on new sample — cuts pixel jitter */
    if (!gPrevBallValid) {
        gFilteredVelCmS = rawVel;
    } else {
        gFilteredVelCmS = gFilteredVelCmS * 0.7f + rawVel * 0.3f;
    }

    gPrevBallCm    = ballCm;
    gPrevBallValid = true;
    gPrevFrameMs   = frameMs;
    gStatus.velocityCmSX100 = (int16_t)(gFilteredVelCmS * 100.0f);
}

/* ================================================================
 *  Cascade PID controller (float)
 * ================================================================ */

static float ComputeCascadeTilt(float ballCm)
{
    bool  isHold  = IsHoldPhase();
    float kpOuter = isHold ? SB_KP_OUTER_HOLD : SB_KP_OUTER_MOVE;
    float kpInner = isHold ? SB_KP_INNER_HOLD : SB_KP_INNER_MOVE;
    float maxSpd  = isHold ? SB_MAX_SPEED_HOLD  : SB_MAX_SPEED_MOVE;
    float maxTilt = isHold ? SB_MAX_TILT_HOLD   : SB_MAX_TILT_MOVE;
    float targetCm, posError, targetVel, velError, tilt;

    targetCm = (float)gStatus.targetCmX100 / 100.0f;
    posError = targetCm - ballCm;

    /* ---- deadband ---- */
    if ((posError < SB_POS_DEADBAND_CM) && (posError > -SB_POS_DEADBAND_CM)) {
        if (isHold) {
            gIntegralCmS += posError * 0.01f;  /* dt ≈ 10ms */
            if (gIntegralCmS >  SB_I_LIMIT_CM_S) gIntegralCmS =  SB_I_LIMIT_CM_S;
            if (gIntegralCmS < -SB_I_LIMIT_CM_S) gIntegralCmS = -SB_I_LIMIT_CM_S;
        }
        return 0.0f;
    }

    /* ==== Outer loop: position → target velocity ==== */
    targetVel = kpOuter * posError;
    if (targetVel >  maxSpd) targetVel =  maxSpd;
    if (targetVel < -maxSpd) targetVel = -maxSpd;

    /* ==== Inner loop: velocity → tilt angle ==== */
    velError = targetVel - gFilteredVelCmS;
    tilt     = kpInner * velError;

    /* ==== Integral: pipe level compensation (HOLD only) ==== */
    if (isHold) {
        gIntegralCmS += posError * 0.01f;
        if (gIntegralCmS >  SB_I_LIMIT_CM_S) gIntegralCmS =  SB_I_LIMIT_CM_S;
        if (gIntegralCmS < -SB_I_LIMIT_CM_S) gIntegralCmS = -SB_I_LIMIT_CM_S;
        tilt += SB_KI_HOLD * gIntegralCmS;
    }

    /* ==== Clamp ==== */
    if (tilt >  maxTilt) tilt =  maxTilt;
    if (tilt < -maxTilt) tilt = -maxTilt;

    /* ==== Minimum tilt threshold (anti-jitter) ==== */
    if ((tilt < SB_MIN_TILT_DEG) && (tilt > -SB_MIN_TILT_DEG)) {
        return 0.0f;
    }

    return tilt;
}

/* ================================================================
 *  Servo helpers
 * ================================================================ */

static void ResetController(void)
{
    gLastControlMs  = 0U;
    gInvalidCount   = 0U;
    gPrevBallValid  = false;
    gPrevBallCm     = 0.0f;
    gFilteredVelCmS = 0.0f;
    gPrevFrameMs    = 0U;
    gIntegralCmS    = 0.0f;
}

static void ResetSequence(void)
{
    gPhase          = STATIC_BALL_PHASE_WAIT_VISION;
    gSelfTestStep   = 0U;
    gSelfTestStepMs = 0U;
    gStatus.phase        = (uint8_t)gPhase;
    gStatus.targetCmX100 = STATIC_BALL_POS_TARGET_CM_X100;
}

static void SetServoDeg(int16_t deg, bool attach)
{
    gStatus.tiltDegX10    = 0;
    gStatus.servoAngleDeg = deg;
    Servo_SetAngle(deg);
    if (attach) { Servo_Attach(); gServoEnabled = true; }
}

static void DisableServo(void) { Servo_Detach(); gServoEnabled = false; }
static void EnableServo(void)  { if (!gServoEnabled) { Servo_Attach(); gServoEnabled = true; } }

/* ================================================================
 *  Public API
 * ================================================================ */

void StaticBall_Init(void)
{
    gActive = false; gServoEnabled = false;
    ResetController(); ResetSequence();
    gStatus.active = false; gStatus.visionFresh = false; gStatus.visionValid = false;
    gStatus.phase = (uint8_t)gPhase; gStatus.quality = 0U;
    gStatus.ballXCmX100 = 0; gStatus.targetCmX100 = STATIC_BALL_POS_TARGET_CM_X100;
    gStatus.velocityCmSX100 = 0;
    SetServoDeg(STATIC_BALL_SERVO_NEUTRAL_DEG, false);
    DisableServo();
}

void StaticBall_Start(void)
{
    gActive = true; gServoEnabled = false;
    ResetController(); ResetSequence();

    /* start with servo self-test sweep to verify hardware */
    gPhase          = STATIC_BALL_PHASE_SELF_TEST;
    gSelfTestStep   = 0U;
    gSelfTestStepMs = 0U;

    gStatus.active = true; gStatus.visionFresh = false; gStatus.visionValid = false;
    gStatus.phase = (uint8_t)gPhase; gStatus.quality = 0U;
    gStatus.ballXCmX100 = 0; gStatus.targetCmX100 = STATIC_BALL_POS_TARGET_CM_X100;
    gStatus.velocityCmSX100 = 0;

    /* immediately attach servo at neutral so the self-test can sweep it */
    SetServoDeg(STATIC_BALL_SERVO_NEUTRAL_DEG, true);
    gServoEnabled = true;
    gStatus.servoAngleDeg = STATIC_BALL_SERVO_NEUTRAL_DEG;
}

void StaticBall_Stop(void)
{
    gActive = false;
    gStatus.active = false; gStatus.visionFresh = false; gStatus.visionValid = false;
    ResetController(); ResetSequence(); DisableServo();
}

void StaticBall_Exit(void)
{
    gActive = false;
    gStatus.active = false; gStatus.visionFresh = false; gStatus.visionValid = false;
    ResetController(); ResetSequence(); DisableServo();
}

void StaticBall_Task(void)
{
    K230VisionFrame frame;
    K230UartStatus  uartStatus;
    uint32_t        nowMs = SystemTime_Millis();
    bool            hasFrame, frameFresh, frameAccepted;
    float           ballCm, tiltDegFloat;
    int16_t         tiltDeg, servoAngleDeg;

    /* mirror UART stats */
    K230Uart_GetStatus(&uartStatus);
    gStatus.rxBytes         = uartStatus.rxBytes;
    gStatus.rxOk            = uartStatus.rxOk;
    gStatus.rxBad           = uartStatus.rxBad;
    gStatus.rxHeadAA        = uartStatus.rxHeadAA;
    gStatus.rxHeadAA55      = uartStatus.rxHeadAA55;
    gStatus.lastByte        = uartStatus.lastByte;
    gStatus.lastLength      = uartStatus.lastLength;
    gStatus.lastType        = uartStatus.lastType;
    gStatus.lastChecksumRx  = uartStatus.lastChecksumRx;
    gStatus.lastChecksumCalc = uartStatus.lastChecksumCalc;

    if (!gActive) { return; }

    /* ---- self-test sweep on entry (verifies servo hardware) ---- */
    if (gPhase == STATIC_BALL_PHASE_SELF_TEST) {
        if (gSelfTestStep < SELF_TEST_NUM_ANGLES) {
            if (gSelfTestStepMs == 0U) {
                gSelfTestStepMs = nowMs;
                gStatus.servoAngleDeg = gSelfTestAngles[gSelfTestStep];
                Servo_SetAngle(gStatus.servoAngleDeg);
            } else if (ElapsedMs(nowMs, gSelfTestStepMs) >= SELF_TEST_SWEEP_MS) {
                gSelfTestStep++;
                gSelfTestStepMs = 0U;
            }
            gStatus.tiltDegX10 = 0;
            gStatus.phase      = (uint8_t)gPhase;
            return;  /* stay in self-test until sweep done */
        }
        /* sweep complete — transition to wait-for-vision */
        gStatus.servoAngleDeg = STATIC_BALL_SERVO_NEUTRAL_DEG;
        Servo_SetAngle(gStatus.servoAngleDeg);
        gPhase               = STATIC_BALL_PHASE_WAIT_VISION;
        gStatus.phase        = (uint8_t)gPhase;
        gStatus.targetCmX100 = GetTargetX100();
    }

    if ((gLastControlMs != 0U) &&
        (ElapsedMs(nowMs, gLastControlMs) < STATIC_BALL_CONTROL_PERIOD_MS)) {
        return;
    }
    gLastControlMs = nowMs;

    hasFrame      = K230Uart_GetLatest(&frame);
    frameFresh    = hasFrame && (ElapsedMs(nowMs, frame.receivedMs) <= STATIC_BALL_VISION_TIMEOUT_MS);
    frameAccepted = frameFresh && frame.valid && (frame.quality >= STATIC_BALL_MIN_QUALITY);

    gStatus.visionFresh = frameFresh;
    gStatus.visionValid = frameAccepted;
    gStatus.phase       = (uint8_t)gPhase;
    if (hasFrame) { gStatus.quality = frame.quality; gStatus.ballXCmX100 = frame.xCmX100; }

    if (!frameAccepted) {
        if (gInvalidCount < 255U) { gInvalidCount++; }
        if (gInvalidCount >= STATIC_BALL_INVALID_LIMIT) { DisableServo(); }
        return;
    }

    EnableServo();
    gInvalidCount = 0U;

    ballCm = (float)frame.xCmX100 / 100.0f;

    AdvanceSequence(frame.xCmX100);

    EstimateVelocity(ballCm, frame.receivedMs);
    tiltDegFloat  = ComputeCascadeTilt(ballCm);
    tiltDeg       = (int16_t)(tiltDegFloat >= 0.0f ? tiltDegFloat + 0.5f : tiltDegFloat - 0.5f);
    servoAngleDeg = ServoAngleFromTilt(tiltDeg);
    Servo_SetAngle(servoAngleDeg);

    gStatus.tiltDegX10   = (int16_t)(tiltDegFloat * 10.0f);
    gStatus.servoAngleDeg = servoAngleDeg;
}

bool StaticBall_IsActive(void)            { return gActive; }
void StaticBall_GetStatus(StaticBallStatus *status) { if (status != 0) { *status = gStatus; } }
