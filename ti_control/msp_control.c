/*
 * msp_control.c — MSPM0G3507 ball-balance firmware (vision-only, no IMU).
 *
 * Architecture:
 *   K230 YOLO → UART (AA 55, x_cm_x100) → MSPM0 PD controller → MG996 PWM
 *
 * No MPU-6050 needed.  Safety relies on:
 *   - Vision timeout (200ms) → auto return to neutral
 *   - Tilt limit (±8°), rate limit (45°/s)
 *   - Edge rescue (ball near tube ends → force inward)
 *
 * Pinout (LP-MSPM0G3507):
 *   PA9 =UART1_RX  ← K230 IO9 (TX)      vision data
 *   PA8 =UART1_TX  → K230 IO10 (RX)     optional debug
 *   PA10=UART0_TX  → XDS110             debug console, 115200
 *   PA11=UART0_RX  ← XDS110
 *   PA6 =TIMG0_C0  → MG996 signal       servo PWM, 50Hz
 *
 * MG996 power: independent high-current supply.  GNDs common:
 *   K230 GND — MSPM0 GND — servo power GND
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "ti_msp_dl_config.h"

/* ================================================================
 *  0. Calibration constants — VERIFY on real hardware
 * ================================================================ */

/* ---- Servo PWM ---- */
#define SERVO_NEUTRAL_US    1500u      /* pulse width at pendulum horizontal */
#define SERVO_MIN_US         900u      /* safe minimum (mechanical stop!)   */
#define SERVO_MAX_US        2100u      /* safe maximum                      */
#define SERVO_US_PER_DEG      10.0f    /* pulse change per degree of tilt   */
#define SERVO_DIRECTION        1.0f    /* +1 or -1, swap if ball goes wrong way */

/* ---- Pendulum limits ---- */
#define MAX_TILT_DEG           8.0f    /* hard limit — never exceed */
#define NORMAL_RATE_DEG_S     45.0f    /* max tilt change per second */
#define RECOVERY_RATE_DEG_S   20.0f    /* slower rate when vision lost */

/* ---- Ball parameters ---- */
#define BALL_USABLE_MIN_CM   -10.0f    /* tube left edge */
#define BALL_USABLE_MAX_CM    10.0f    /* tube right edge */
#define BALL_EDGE_MARGIN_CM    1.5f    /* rescue zone near edges */
#define BALL_DEADBAND_CM       0.20f   /* ignore tiny errors */

/* ---- Vision timeout ---- */
#define VISION_TIMEOUT_MS    200u      /* max ms without valid frame */
#define INVALID_FRAME_LIMIT    3u      /* consecutive bad frames → invalid */

/* ---- PD gains (initial; tune after calibration) ---- */
#define PID_KP                 0.80f
#define PID_KI                 0.00f   /* keep 0 until static passes */
#define PID_KD                 0.15f

/* ---- UART protocol (shared with k230_code/k230_yolo.py) ---- */
#define PROTO_H0             0xAAu
#define PROTO_H1             0x55u
#define MSG_VISION           0x01u
#define VISION_FLAG_VALID    0x01u
#define VISION_PAYLOAD_LEN   6u
#define VISION_MIN_QUALITY   50u
#define PROTO_MAX_PAYLOAD    32u

/* ================================================================
 *  1. Types
 * ================================================================ */

typedef enum {
    MODE_IDLE = 0,
    MODE_LINE_STOP,        /* car stopped on the line */
    MODE_STATIC_BALL,      /* static ball positioning (-5/0/+5 cm) */
    MODE_DYN_AB,           /* dynamic A→B movement */
    MODE_DYN_LAP_CENTER,   /* full lap, ball at center */
    MODE_DYN_LAP_TARGET,   /* full lap, ball at specified position */
} Mode;

typedef struct {
    uint32_t ts_ms;          /* local timestamp of last valid frame */
    int16_t  x_cm_x100;      /* ball position, 0.01 cm units */
    float    vel_cm_s;       /* derived velocity (low-pass filtered) */
    uint8_t  quality;        /* 0-100, from K230 */
    bool     valid;
} Ball;

typedef enum {
    RS_IDLE = 0,
    RS_H1,
    RS_LEN,
    RS_BODY,
    RS_CSUM,
} RxSt;

/* ================================================================
 *  2. Global state
 * ================================================================ */

static Mode    g_mode       = MODE_IDLE;
static Ball    g_ball       = {0};
static float   g_target_cm  = 0.0f;
static float   g_kp         = PID_KP;
static float   g_ki         = PID_KI;
static float   g_kd         = PID_KD;
static float   g_integrator = 0.0f;
static float   g_last_tilt  = 0.0f;
static uint32_t g_ms        = 0u;      /* 10ms tick counter */
static uint8_t  g_bad_cnt   = 0u;
static uint32_t g_rx_ok     = 0u;
static uint32_t g_rx_bad    = 0u;

/* ---- UART frame parser ---- */
static RxSt    g_rs         = RS_IDLE;
static uint8_t g_rlen, g_rpos, g_rcsum;
static uint8_t g_rbuf[PROTO_MAX_PAYLOAD + 2];

/* ================================================================
 *  3. Math (soft-float, M0+ friendly)
 * ================================================================ */

static inline float _min(float a, float b) { return a < b ? a : b; }
static inline float _max(float a, float b) { return a > b ? a : b; }
static float _clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static float _abs(float v) { return v < 0.0f ? -v : v; }

static int16_t _le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ================================================================
 *  4. UART helpers
 * ================================================================ */

static void u0_ch(char c) {
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)c);
}
static void u0_s(const char *s) { while (*s) u0_ch(*s++); }
static void u0_dec(uint32_t v) {
    char b[10]; int n = 0;
    if (!v) { u0_ch('0'); return; }
    while (v) { b[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    while (n--) u0_ch(b[n]);
}
static void u0_f1(float v) {
    if (v < 0.0f) { u0_ch('-'); v = -v; }
    int16_t w = (int16_t)v;
    uint8_t f = (uint8_t)((v - (float)w) * 10.0f + 0.5f);
    if (f > 9u) { w++; f = 0u; }
    u0_dec((uint32_t)w); u0_ch('.'); u0_ch('0' + (char)f);
}

/* ================================================================
 *  5. K230 vision frame parser (AA 55 protocol)
 * ================================================================ */

static void on_vision_frame(const uint8_t *pay, uint8_t seq) {
    bool flag = (pay[0] & VISION_FLAG_VALID) != 0;
    uint8_t q = pay[5];

    if (!flag || q < VISION_MIN_QUALITY) {
        if (g_bad_cnt < 255u) g_bad_cnt++;
        if (g_bad_cnt >= INVALID_FRAME_LIMIT) {
            g_ball.valid = false;
            g_ball.vel_cm_s = 0.0f;
            g_integrator = 0.0f;
        }
        g_rx_bad++;
        return;
    }

    int16_t x100 = _le16(&pay[1]);
    float pos_cm = x100 / 100.0f;
    float vel = 0.0f;
    uint32_t dt = g_ms - g_ball.ts_ms;

    if (g_ball.valid && dt > 0u && dt <= VISION_TIMEOUT_MS) {
        float raw = (pos_cm - g_ball.x_cm_x100 / 100.0f) / (dt / 1000.0f);
        vel = 0.70f * g_ball.vel_cm_s + 0.30f * raw;
    }

    g_ball.ts_ms     = g_ms;
    g_ball.x_cm_x100 = x100;
    g_ball.quality   = q;
    g_ball.vel_cm_s  = vel;
    g_ball.valid     = true;
    g_bad_cnt        = 0u;
    g_rx_ok++;
}

static void rx_byte(uint8_t b) {
    switch (g_rs) {
    case RS_IDLE:
        if (b == PROTO_H0) g_rs = RS_H1;
        break;
    case RS_H1:
        g_rs = (b == PROTO_H1) ? RS_LEN : (b == PROTO_H0 ? RS_H1 : RS_IDLE);
        break;
    case RS_LEN:
        if (b < 2u || b > PROTO_MAX_PAYLOAD + 2u) { g_rs = RS_IDLE; break; }
        g_rlen = b; g_rpos = 0u; g_rcsum = b; g_rs = RS_BODY;
        break;
    case RS_BODY:
        g_rbuf[g_rpos++] = b;
        g_rcsum = (uint8_t)(g_rcsum + b);
        if (g_rpos >= g_rlen) g_rs = RS_CSUM;
        break;
    case RS_CSUM:
        if (b == g_rcsum && g_rlen == VISION_PAYLOAD_LEN + 2u &&
            g_rbuf[0] == MSG_VISION) {
            on_vision_frame(&g_rbuf[2], g_rbuf[1]);
        } else { g_rx_bad++; }
        g_rs = RS_IDLE;
        break;
    }
}

static void rx_drain(void) {
    while (!DL_UART_isRXFIFOEmpty(UART_1_INST))
        rx_byte(DL_UART_receiveData(UART_1_INST));
}

/* ================================================================
 *  6. Servo PWM (TIMG0: 1MHz clock, 20000 period = 50Hz)
 * ================================================================ */

static uint16_t tilt_to_us(float tilt_deg) {
    /* tilt_deg > 0 means right-side-up → ball rolls left, or vice versa.
     * The actual direction depends on linkage geometry.
     * SWAP SERVO_DIRECTION if the ball moves opposite to the command. */
    float deg = SERVO_DIRECTION * tilt_deg;
    float us  = (float)SERVO_NEUTRAL_US + deg * SERVO_US_PER_DEG;
    us = _clamp(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
    return (uint16_t)(us + 0.5f);
}

static void servo_write(float tilt_deg) {
    DL_Timer_setCaptureCompareValue(SERVO_TIM_INST,
        tilt_to_us(tilt_deg), DL_TIMER_CC_0_INDEX);
}

static float rate_limit(float wanted, float max_rate_dps, float dt_s) {
    float step = max_rate_dps * _clamp(dt_s, 0.001f, 0.050f);
    float delta = wanted - g_last_tilt;
    delta = _clamp(delta, -step, step);
    g_last_tilt += delta;
    return g_last_tilt;
}

/* ================================================================
 *  7. PD controller
 * ================================================================ */

static bool vision_ok(void) {
    return g_ball.valid && (g_ms - g_ball.ts_ms) <= VISION_TIMEOUT_MS;
}

static float edge_rescue(float nominal) {
    float pos = g_ball.x_cm_x100 / 100.0f;
    float lo  = BALL_USABLE_MIN_CM + BALL_EDGE_MARGIN_CM;
    float hi  = BALL_USABLE_MAX_CM - BALL_EDGE_MARGIN_CM;
    if (pos < lo) return lo + BALL_EDGE_MARGIN_CM;
    if (pos > hi) return hi - BALL_EDGE_MARGIN_CM;
    return nominal;
}

static float pd_tilt(void) {
    float target = edge_rescue(g_target_cm);
    float pos    = g_ball.x_cm_x100 / 100.0f;
    float err    = target - pos;

    if (_abs(err) <= BALL_DEADBAND_CM) err = 0.0f;

    /* Integrator: enable only after PD tuning passes */
    if (g_ki > 0.0f && _abs(g_ball.vel_cm_s) < 2.0f) {
        g_integrator += err * 0.010f;   /* dt ≈ 0.01s */
        g_integrator = _clamp(g_integrator, -3.0f, 3.0f);
    } else {
        g_integrator *= 0.95f;          /* leak to prevent windup */
    }

    float tilt = g_kp * err + g_ki * g_integrator - g_kd * g_ball.vel_cm_s;

    /* Hard limit */
    tilt = _clamp(tilt, -MAX_TILT_DEG, MAX_TILT_DEG);

    return tilt;
}

/* ================================================================
 *  8. Control tick (100 Hz)
 * ================================================================ */

static void tick(float dt_s) {
    float wanted_tilt, max_rate, tilt;

    /* Drain K230 UART */
    rx_drain();

    switch (g_mode) {
    case MODE_IDLE:
        /* No servo output — beam disabled */
        servo_write(0.0f);
        return;

    case MODE_LINE_STOP:
        wanted_tilt = 0.0f;
        max_rate    = RECOVERY_RATE_DEG_S;
        break;

    case MODE_STATIC_BALL:
    case MODE_DYN_AB:
    case MODE_DYN_LAP_CENTER:
    case MODE_DYN_LAP_TARGET:
        if (!vision_ok()) {
            /* Lost vision → slow return to neutral */
            wanted_tilt = 0.0f;
            max_rate    = RECOVERY_RATE_DEG_S;
            g_integrator = 0.0f;
        } else {
            wanted_tilt = pd_tilt();
            max_rate    = NORMAL_RATE_DEG_S;
        }
        break;

    default:
        g_mode = MODE_IDLE;
        return;
    }

    tilt = rate_limit(wanted_tilt, max_rate, dt_s);
    servo_write(tilt);
}

/* ================================================================
 *  9. Status reporting (every ~500ms)
 * ================================================================ */

static void status(uint32_t loop) {
    u0_s("T=");   u0_dec(g_ms / 1000u);
    u0_s("s m="); u0_dec((uint32_t)g_mode);
    u0_s(" ball=");
    if (g_ball.valid) {
        u0_f1(g_ball.x_cm_x100 / 100.0f);
        u0_s("cm v=");
        u0_f1(g_ball.vel_cm_s);
        u0_s("cm/s");
    } else {
        u0_s("NONE");
    }
    u0_s(" tgt=");   u0_f1(g_target_cm);
    u0_s(" tilt=");  u0_f1(g_last_tilt);
    u0_s("deg ok="); u0_dec(g_rx_ok);
    u0_s(" bad=");   u0_dec(g_rx_bad);
    u0_s(" kp=");    u0_f1(g_kp);
    u0_s(" kd=");    u0_f1(g_kd);
    u0_s("\r\n");
}

/* ================================================================
 *  9.5  Debug command parser (UART0 RX ← XDS110)
 * ================================================================ */

static char _cmd[16];
static uint8_t _cmd_pos = 0u;

static float _parse_float(const char *s) {
    float sign = 1.0f, val = 0.0f, frac = 0.0f, div = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }
    while (*s >= '0' && *s <= '9') { val = val * 10.0f + (*s++ - '0'); }
    if (*s == '.') { s++; while (*s >= '0' && *s <= '9') { frac = frac * 10.0f + (*s++ - '0'); div *= 10.0f; } }
    return sign * (val + frac / div);
}

static void _run_cmd(void) {
    _cmd[_cmd_pos] = '\0';

    if (_cmd[0] == 'm') {
        /* m0=mode_idle ... m5=mode_lap_target */
        uint32_t m = _cmd[1] - '0';
        if (m <= 5u) {
            g_mode = (Mode)m;
            u0_s("mode="); u0_dec(m);
            if (m == (uint32_t)MODE_STATIC_BALL) {
                g_integrator = 0.0f;
                u0_s(" target="); u0_f1(g_target_cm);
            }
            u0_s("\r\n");
        }
    } else if (_cmd[0] == 't') {
        g_target_cm = _parse_float(&_cmd[1]);
        g_integrator = 0.0f;
        u0_s("target="); u0_f1(g_target_cm); u0_s("cm\r\n");
    } else if (_cmd[0] == 'p' && _cmd[1] == 'k') {
        g_kp = _parse_float(&_cmd[2]);
        u0_s("Kp="); u0_f1(g_kp); u0_s("\r\n");
    } else if (_cmd[0] == 'd' && _cmd[1] == 'k') {
        g_kd = _parse_float(&_cmd[2]);
        u0_s("Kd="); u0_f1(g_kd); u0_s("\r\n");
    } else if (_cmd[0] == 'i' && _cmd[1] == 'k') {
        g_ki = _parse_float(&_cmd[2]);
        g_integrator = 0.0f;
        u0_s("Ki="); u0_f1(g_ki); u0_s("\r\n");
    } else if (_cmd[0] == 'g' && _cmd[1] == 'o') {
        g_mode = MODE_STATIC_BALL;
        g_integrator = 0.0f;
        u0_s("GO: STATIC_BALL target="); u0_f1(g_target_cm); u0_s("cm\r\n");
    } else if (_cmd[0] == 's') {
        /* 's' = status print on demand */
        u0_s("STATUS: ball=");
        if (g_ball.valid) {
            u0_f1(g_ball.x_cm_x100 / 100.0f); u0_s("cm v=");
            u0_f1(g_ball.vel_cm_s); u0_s("cm/s q=");
            u0_dec(g_ball.quality);
        } else { u0_s("NONE"); }
        u0_s(" tgt="); u0_f1(g_target_cm);
        u0_s(" tilt="); u0_f1(g_last_tilt);
        u0_s(" kp="); u0_f1(g_kp);
        u0_s(" kd="); u0_f1(g_kd);
        u0_s("\r\n");
    }
}

static void _cmd_feed(char c) {
    if (c == '\r' || c == '\n') {
        if (_cmd_pos > 0u) { _run_cmd(); _cmd_pos = 0u; }
    } else if (_cmd_pos < sizeof(_cmd) - 1u) {
        _cmd[_cmd_pos++] = c;
    }
}

static void _cmd_poll(void) {
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        _cmd_feed((char)DL_UART_receiveData(UART_0_INST));
    }
}

/* ================================================================
 * 10. Main
 * ================================================================ */

int main(void) {
    uint32_t last_ms = 0u, loop = 0u;

    SYSCFG_DL_init();
    delay_cycles(3200000u);

    u0_s("\r\n=== MSPM0 BALL-BALANCE (vision-only) ===\r\n");
    u0_s("Pinout: PA9=K230_RX PA6=MG996_PWM\r\n");
    u0_s("Mode=IDLE. Commands:\r\n");
    u0_s("  m0=idle m1=line_stop m2=static m3=AB m4=lap_c m5=lap_tgt\r\n");
    u0_s("  t+5.0   (set target cm)\r\n");
    u0_s("  pk0.80  (set Kp)  dk0.15 (set Kd)  ik0.00 (set Ki)\r\n");
    u0_s("  go      (enter STATIC_BALL at current target)\r\n");
    u0_s("  s       (print status on demand)\r\n");
    u0_s("----------------------------------------\r\n");

    /* Start servo PWM at neutral */
    DL_Timer_startCounter(SERVO_TIM_INST);
    servo_write(0.0f);

    /* Start 100Hz timer */
    DL_Timer_startCounter(CTRL_TIM_INST);

    while (1) {
        /* Poll debug commands */
        _cmd_poll();

        /* Wait for 100Hz IRQ */
        if (g_ms == last_ms) continue;

        float dt = (g_ms - last_ms) / 1000.0f;
        if (dt > 0.050f) dt = 0.050f;
        last_ms = g_ms;

        tick(dt);

        loop++;
        if ((loop % 50u) == 0u) status(loop);
    }
}

/* ================================================================
 * 11. TIMG4 ISR — 100 Hz
 * ================================================================ */

void CTRL_TIM_IRQHandler(void) {
    DL_Timer_clearZeroInterruptStatus(CTRL_TIM_INST);
    g_ms += 10u;   /* 100Hz → 10ms per tick */
}
