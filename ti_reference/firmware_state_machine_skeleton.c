/*
 * MSPM0 ball-control module for the H-topic vehicle.
 *
 * Hardware adaptation required by the final Ti project:
 *   - Call app_uart_rx_bytes() from the UART RX ISR/DMA callback.
 *   - Call app_update(now_ms, 0.01f) from a 100 Hz task.
 *   - Implement platform_set_beam_enable() and platform_servo_write_us() with
 *     the MG996 PWM timer.  Servo output is 50 Hz; control calculations are
 *     100 Hz and update the latest pulse command.
 *
 * UART frame accepted here:
 *   AA 55 | LEN=8 | TYPE=01 | SEQ | flags | x_cm_x100:i16_le |
 *   y_offset_px:i16_le | quality:u8 | sum
 *
 * The receive timestamp is generated on this MSPM0.  Do not use a K230 clock
 * for the vision watchdog because the boards do not share a time base.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MODE_IDLE = 0,
    MODE_LINE_STOP,
    MODE_STATIC_BALL,
    MODE_DYN_AB,
    MODE_DYN_LAP_CENTER,
    MODE_DYN_LAP_TARGET,
} Mode;

typedef struct {
    uint32_t received_ms;       /* Local MSPM0 time of the last valid frame. */
    int16_t x_cm_x100;          /* Ball center along the pipe, calibrated. */
    int16_t y_offset_px;        /* Diagnostics only; never used for control. */
    uint8_t quality;            /* 0..100 visual quality. */
    float velocity_cm_s;        /* Derived on the MSPM0 for PD damping. */
    bool valid;
} BallObs;

typedef struct {
    int16_t target_deg_x10;       /* Relative to calibrated pipe-neutral angle. */
    int16_t max_speed_deg_s_x10;
    bool enable;
    bool homing;
} BeamCmd;

typedef struct {
    float kp;
    float ki;                    /* Kept at zero for first commissioning. */
    float kd;
    float integrator;
} PID;

/* UART protocol constants, shared with k230_mspm0_uart_protocol.py. */
#define PROTO_HEAD_0             0xAAu
#define PROTO_HEAD_1             0x55u
#define PROTO_MAX_PAYLOAD        32u
#define MSG_VISION_TARGET        0x01u
#define VISION_FLAG_VALID        0x01u
#define VISION_PAYLOAD_LEN       6u

/* Safety and first-pass controller configuration.  Calibrate before testing. */
#define CONTROL_PERIOD_S         0.010f
#define VISION_TIMEOUT_MS        200u
#define VISION_SEQ_RESYNC_MS     500u
#define INVALID_FRAME_LIMIT      3u
#define VISION_MIN_QUALITY       50u

#define PIPE_TRIM_DEG            0.0f
#define MAX_PIPE_TILT_DEG         8.0f
#define NORMAL_TILT_RATE_DEG_S   45.0f
#define RECOVERY_TILT_RATE_DEG_S 20.0f
#define BALL_DEADBAND_CM          0.20f

/* Replace after the physical 25 cm pipe and anti-drop zone are calibrated. */
#define BALL_USABLE_MIN_CM      -10.0f
#define BALL_USABLE_MAX_CM       10.0f
#define BALL_EDGE_MARGIN_CM       1.5f

/* MG996 PWM calibration: adjust all three values on the real mechanism. */
#define SERVO_PERIOD_HZ          50u
#define SERVO_NEUTRAL_US       1500u
#define SERVO_MIN_US            900u
#define SERVO_MAX_US           2100u
#define SERVO_US_PER_DEG        10.0f
/* Change to -1.0f if increasing pulse width moves the ball opposite to target. */
#define SERVO_BALL_DIRECTION     1.0f

/* Dynamic modes remain disabled until their prerequisite bench tests pass. */
#define ENFORCE_STAGE_GATE        1u

/* Implement these two functions in the actual MSPM0 board-support project. */
extern void platform_set_beam_enable(bool enable);
extern void platform_servo_write_us(uint16_t pulse_us);

static Mode g_mode = MODE_IDLE;
static BallObs g_ball = {0};
static BeamCmd g_beam = {0};
static PID g_ball_pid = {.kp = 0.80f, .ki = 0.0f, .kd = 0.15f, .integrator = 0.0f};
static float g_requested_target_cm = 0.0f;
static float g_accel_feedforward_deg = 0.0f;
static float g_last_pipe_deg = PIPE_TRIM_DEG;
static uint8_t g_invalid_frame_count = 0u;
static bool g_have_sequence = false;
static uint8_t g_last_sequence = 0u;
static bool g_static_qualified = false;
static bool g_ab_qualified = false;
static bool g_lap_center_qualified = false;

static float clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static float absf(float value) {
    return value < 0.0f ? -value : value;
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t then_ms) {
    /* Unsigned subtraction is correct across the 32-bit millisecond rollover. */
    return now_ms - then_ms;
}

static int16_t get_i16_le(const uint8_t *data) {
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)value;
}

static uint16_t pipe_deg_to_pulse_us(float pipe_deg) {
    float pulse = (float)SERVO_NEUTRAL_US +
                  (pipe_deg - PIPE_TRIM_DEG) * SERVO_US_PER_DEG;
    pulse = clampf(pulse, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
    return (uint16_t)(pulse + 0.5f);
}

static float rate_limit_pipe_deg(float wanted_deg, float max_rate_deg_s, float dt_s) {
    float step = max_rate_deg_s * clampf(dt_s, 0.001f, 0.050f);
    float delta = wanted_deg - g_last_pipe_deg;
    delta = clampf(delta, -step, step);
    g_last_pipe_deg += delta;
    return g_last_pipe_deg;
}

static void apply_beam_command(const BeamCmd *command) {
    if (!command->enable) {
        platform_set_beam_enable(false);
        return;
    }

    platform_set_beam_enable(true);
    platform_servo_write_us(pipe_deg_to_pulse_us(command->target_deg_x10 / 10.0f));
}

static void reset_ball_controller(void) {
    g_ball_pid.integrator = 0.0f;
}

static bool vision_is_fresh(uint32_t now_ms) {
    return g_ball.valid &&
           elapsed_ms(now_ms, g_ball.received_ms) <= VISION_TIMEOUT_MS;
}

static bool mode_is_allowed(Mode mode) {
    if (ENFORCE_STAGE_GATE == 0u) return true;
    if (mode == MODE_DYN_AB) return g_static_qualified;
    if (mode == MODE_DYN_LAP_CENTER) return g_static_qualified && g_ab_qualified;
    if (mode == MODE_DYN_LAP_TARGET) {
        return g_static_qualified && g_ab_qualified && g_lap_center_qualified;
    }
    return true;
}

/* Call after the documented acceptance test for the completed stage. */
void app_mark_stage_qualified(Mode mode, bool qualified) {
    if (mode == MODE_STATIC_BALL) g_static_qualified = qualified;
    if (mode == MODE_DYN_AB) g_ab_qualified = qualified;
    if (mode == MODE_DYN_LAP_CENTER) g_lap_center_qualified = qualified;
}

bool app_try_set_mode(Mode mode) {
    if (!mode_is_allowed(mode)) return false;
    g_mode = mode;
    reset_ball_controller();
    return true;
}

/* Compatibility wrapper for a simple button/state-machine caller. */
void app_set_mode(Mode mode) {
    (void)app_try_set_mode(mode);
}

void app_set_target_cm(float target_cm) {
    g_requested_target_cm = clampf(target_cm, BALL_USABLE_MIN_CM, BALL_USABLE_MAX_CM);
    reset_ball_controller();
}

/* Leave at zero for this no-angle-sensor first version. */
void app_set_accel_feedforward_deg(float feedforward_deg) {
    g_accel_feedforward_deg = clampf(feedforward_deg, -2.0f, 2.0f);
}

static bool sequence_is_new(uint8_t sequence, uint32_t now_ms) {
    if (!g_have_sequence || elapsed_ms(now_ms, g_ball.received_ms) > VISION_SEQ_RESYNC_MS) {
        g_have_sequence = true;
        g_last_sequence = sequence;
        return true;
    }

    {
        uint8_t delta = (uint8_t)(sequence - g_last_sequence);
        if (delta == 0u || delta > 127u) return false;
        g_last_sequence = sequence;
    }
    return true;
}

static void on_vision_payload(const uint8_t *payload, uint8_t sequence, uint32_t now_ms) {
    bool valid_flag;
    uint8_t quality;

    if (!sequence_is_new(sequence, now_ms)) return;

    valid_flag = (payload[0] & VISION_FLAG_VALID) != 0u;
    quality = payload[5];
    if (!valid_flag || quality < VISION_MIN_QUALITY) {
        if (g_invalid_frame_count < 255u) g_invalid_frame_count++;
        if (g_invalid_frame_count >= INVALID_FRAME_LIMIT) {
            g_ball.valid = false;
            g_ball.velocity_cm_s = 0.0f;
            reset_ball_controller();
        }
        return;
    }

    {
        int16_t x_cm_x100 = get_i16_le(&payload[1]);
        float new_position_cm = x_cm_x100 / 100.0f;
        float velocity_cm_s = 0.0f;
        uint32_t dt_ms = elapsed_ms(now_ms, g_ball.received_ms);

        if (g_ball.valid && dt_ms > 0u && dt_ms <= VISION_TIMEOUT_MS) {
            float raw_velocity = (new_position_cm - g_ball.x_cm_x100 / 100.0f) /
                                 (dt_ms / 1000.0f);
            /* Low-pass the discrete derivative to avoid amplifying Blob jitter. */
            velocity_cm_s = 0.70f * g_ball.velocity_cm_s + 0.30f * raw_velocity;
        }

        g_ball.received_ms = now_ms;
        g_ball.x_cm_x100 = x_cm_x100;
        g_ball.y_offset_px = get_i16_le(&payload[3]);
        g_ball.quality = quality;
        g_ball.velocity_cm_s = velocity_cm_s;
        g_ball.valid = true;
        g_invalid_frame_count = 0u;
    }
}

typedef enum {
    RX_WAIT_HEAD_0 = 0,
    RX_WAIT_HEAD_1,
    RX_WAIT_LENGTH,
    RX_WAIT_BODY,
    RX_WAIT_CHECKSUM,
} RxState;

typedef struct {
    RxState state;
    uint8_t length;
    uint8_t body_length;
    uint8_t checksum;
    uint8_t body[PROTO_MAX_PAYLOAD + 2u]; /* TYPE + SEQ + payload */
} VisionRx;

static VisionRx g_vision_rx = {.state = RX_WAIT_HEAD_0};

void app_uart_rx_byte(uint8_t byte, uint32_t now_ms) {
    switch (g_vision_rx.state) {
        case RX_WAIT_HEAD_0:
            if (byte == PROTO_HEAD_0) g_vision_rx.state = RX_WAIT_HEAD_1;
            break;

        case RX_WAIT_HEAD_1:
            if (byte == PROTO_HEAD_1) {
                g_vision_rx.state = RX_WAIT_LENGTH;
            } else if (byte != PROTO_HEAD_0) {
                g_vision_rx.state = RX_WAIT_HEAD_0;
            }
            break;

        case RX_WAIT_LENGTH:
            if (byte < 2u || byte > (PROTO_MAX_PAYLOAD + 2u)) {
                g_vision_rx.state = RX_WAIT_HEAD_0;
                break;
            }
            g_vision_rx.length = byte;
            g_vision_rx.body_length = 0u;
            g_vision_rx.checksum = byte;
            g_vision_rx.state = RX_WAIT_BODY;
            break;

        case RX_WAIT_BODY:
            g_vision_rx.body[g_vision_rx.body_length++] = byte;
            g_vision_rx.checksum = (uint8_t)(g_vision_rx.checksum + byte);
            if (g_vision_rx.body_length >= g_vision_rx.length) {
                g_vision_rx.state = RX_WAIT_CHECKSUM;
            }
            break;

        case RX_WAIT_CHECKSUM:
            if (byte == g_vision_rx.checksum &&
                    g_vision_rx.length == (VISION_PAYLOAD_LEN + 2u) &&
                    g_vision_rx.body[0] == MSG_VISION_TARGET) {
                on_vision_payload(&g_vision_rx.body[2], g_vision_rx.body[1], now_ms);
            }
            g_vision_rx.state = RX_WAIT_HEAD_0;
            break;

        default:
            g_vision_rx.state = RX_WAIT_HEAD_0;
            break;
    }
}

void app_uart_rx_bytes(const uint8_t *data, size_t length, uint32_t now_ms) {
    size_t index;
    for (index = 0u; index < length; ++index) {
        app_uart_rx_byte(data[index], now_ms);
    }
}

/* Useful for a host test or a non-UART vision transport; timestamp must be local. */
void app_on_ball_obs(BallObs observation) {
    if (!observation.valid || observation.quality < VISION_MIN_QUALITY) return;
    g_ball = observation;
    g_invalid_frame_count = 0u;
}

static float target_for_mode(void) {
    switch (g_mode) {
        case MODE_STATIC_BALL:
        case MODE_DYN_LAP_TARGET:
            return g_requested_target_cm;
        case MODE_DYN_AB:
        case MODE_DYN_LAP_CENTER:
            return 0.0f;
        default:
            return 0.0f;
    }
}

static float apply_edge_rescue(float nominal_target_cm) {
    float position_cm = g_ball.x_cm_x100 / 100.0f;
    float low_edge_cm = BALL_USABLE_MIN_CM + BALL_EDGE_MARGIN_CM;
    float high_edge_cm = BALL_USABLE_MAX_CM - BALL_EDGE_MARGIN_CM;

    if (position_cm < low_edge_cm) return low_edge_cm + BALL_EDGE_MARGIN_CM;
    if (position_cm > high_edge_cm) return high_edge_cm - BALL_EDGE_MARGIN_CM;
    return nominal_target_cm;
}

static BeamCmd neutral_command(float dt_s, bool enabled) {
    BeamCmd command = {0};
    float angle = rate_limit_pipe_deg(PIPE_TRIM_DEG, RECOVERY_TILT_RATE_DEG_S, dt_s);
    command.target_deg_x10 = (int16_t)(angle * 10.0f);
    command.max_speed_deg_s_x10 = (int16_t)(RECOVERY_TILT_RATE_DEG_S * 10.0f);
    command.enable = enabled;
    command.homing = false;
    return command;
}

static BeamCmd ball_controller_update(uint32_t now_ms, float target_cm, float dt_s) {
    BeamCmd command = {0};
    float ball_cm;
    float error_cm;
    float requested_pipe_deg;
    float limited_pipe_deg;

    if (!vision_is_fresh(now_ms)) {
        reset_ball_controller();
        return neutral_command(dt_s, true);
    }

    target_cm = apply_edge_rescue(target_cm);
    ball_cm = g_ball.x_cm_x100 / 100.0f;
    error_cm = target_cm - ball_cm;
    if (absf(error_cm) <= BALL_DEADBAND_CM) error_cm = 0.0f;

    /* I remains disabled until PD has passed static and low-speed AB testing. */
    requested_pipe_deg = PIPE_TRIM_DEG + SERVO_BALL_DIRECTION * (
        g_ball_pid.kp * error_cm -
        g_ball_pid.kd * g_ball.velocity_cm_s +
        g_accel_feedforward_deg
    );
    requested_pipe_deg = clampf(requested_pipe_deg,
                                PIPE_TRIM_DEG - MAX_PIPE_TILT_DEG,
                                PIPE_TRIM_DEG + MAX_PIPE_TILT_DEG);
    limited_pipe_deg = rate_limit_pipe_deg(requested_pipe_deg,
                                            NORMAL_TILT_RATE_DEG_S, dt_s);

    command.target_deg_x10 = (int16_t)(limited_pipe_deg * 10.0f);
    command.max_speed_deg_s_x10 = (int16_t)(NORMAL_TILT_RATE_DEG_S * 10.0f);
    command.enable = true;
    command.homing = false;
    return command;
}

void app_update(uint32_t now_ms, float dt_s) {
    dt_s = clampf(dt_s, 0.001f, 0.050f);

    switch (g_mode) {
        case MODE_IDLE:
            g_beam = neutral_command(dt_s, false);
            break;

        case MODE_LINE_STOP:
            g_beam = neutral_command(dt_s, true);
            break;

        case MODE_STATIC_BALL:
        case MODE_DYN_AB:
        case MODE_DYN_LAP_CENTER:
        case MODE_DYN_LAP_TARGET:
            g_beam = ball_controller_update(now_ms, target_for_mode(), dt_s);
            break;

        default:
            g_mode = MODE_IDLE;
            g_beam = neutral_command(dt_s, false);
            break;
    }

    apply_beam_command(&g_beam);
}

const BallObs *app_ball_observation(void) {
    return &g_ball;
}

const BeamCmd *app_beam_command(void) {
    return &g_beam;
}
