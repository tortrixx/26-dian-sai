#ifndef LINE_FOLLOW_CONFIG_H
#define LINE_FOLLOW_CONFIG_H

#include <stdint.h>

/*
 * All line-follow tuning lives here. Speeds are encoder counts per
 * MOTOR_CONTROL_PERIOD_MS, not raw PWM.
 */

#define LINE_SENSOR_BLACK_IS_LOW              1
#define LINE_SENSOR_REVERSE_ORDER             0
#define LINE_SENSOR_ENABLE_MASK               0xFFU

#define LINE_CONTROL_PERIOD_MS                10U
#define LINE_MIN_TRACK_SPEED_TICKS            0
#define LINE_MAX_SPEED_TICKS                  28
#define LINE_TURN_SCALE                       8
#define LINE_TARGET_SLEW_TICKS                2
#define LINE_STRAIGHT_SPEED_TICKS             22
#define LINE_EDGE_TURN_SPEED_TICKS            16
#define LINE_EDGE_TURN_TICKS                  7
#define LINE_EDGE_LEFT_MASK                   0x03U
#define LINE_EDGE_RIGHT_MASK                  0xC0U
#define LINE_EDGE_HOLD_MS                     70U
#define LINE_WHEEL_BALANCE_ENABLE             1
#define LINE_WHEEL_BALANCE_DIFF_TICKS         6
#define LINE_WHEEL_BALANCE_HOLD_MS            500U
#define LINE_WHEEL_BALANCE_STEP_PERIOD_MS     300U
#define LINE_WHEEL_BALANCE_STEP_FINE          1
#define LINE_WHEEL_BALANCE_MAX_FINE           1
#define LINE_WHEEL_BALANCE_DECAY_FINE         1
#define LINE_WHEEL_BALANCE_MIN_TARGET_TICKS   10
#define LINE_TURN_SLEW_FINE                   4
#define LINE_TURN_SLEW_PERIOD_MS              10U
#define LINE_CENTER_RELEASE_FINE              16
#define LINE_CENTER_RELEASE_SLEW_FINE         12
#define LINE_LEFT_BASE_TRIM_FINE              28
#define LINE_RIGHT_BASE_TRIM_FINE             30
#define LINE_STOP_BLACK_COUNT                 6U
#define LINE_STOP_ENABLE_DELAY_MS             5000U
#define LINE_LOST_HOLD_MS                     180U
#define LINE_LOST_BASE_SPEED_TICKS            8
#define LINE_LOST_TURN_TICKS                  8

#define MOTOR_CONTROL_PERIOD_MS               10U
#define MOTOR_PWM_PERIOD                      1000U
#define MOTOR_PWM_MAX                         550
#define MOTOR_PWM_FEED_FORWARD                35
#define MOTOR_PWM_PER_TICK                    10
#define MOTOR_SPEED_KP_NUM                    12
#define MOTOR_SPEED_KI_NUM                    1
#define MOTOR_SPEED_GAIN_DEN                  10
#define MOTOR_SPEED_INTEGRAL_LIMIT            800
#define MOTOR_PWM_SLEW_STEP                   25
#define MOTOR_MEASURE_FILTER_SHIFT            1

/*
 * The left wheel has more mechanical resistance, so it gets a higher
 * minimum PWM and a small feed-forward bias before the PI loop corrects.
 */
#define MOTOR_LEFT_MIN_PWM                    183
#define MOTOR_RIGHT_MIN_PWM                   135
#define MOTOR_LEFT_PWM_BIAS                   25
#define MOTOR_RIGHT_PWM_BIAS                  0

/*
 * Flip these from 0 to 1 if the car moves backward for a positive command,
 * or if an encoder count decreases while its wheel moves forward.
 */
#define MOTOR_LEFT_DIRECTION_INVERT           1
#define MOTOR_RIGHT_DIRECTION_INVERT          0
#define MOTOR_LEFT_ENCODER_REVERSE            0
#define MOTOR_RIGHT_ENCODER_REVERSE           1

#endif
