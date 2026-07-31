#include "encoder.h"

#include "line_follow_config.h"
#include "ti_msp_dl_config.h"

#define ENCODER_LEFT_A_PIN     DL_GPIO_PIN_8
#define ENCODER_LEFT_B_PIN     DL_GPIO_PIN_12
#define ENCODER_RIGHT_A_PIN    DL_GPIO_PIN_6
#define ENCODER_RIGHT_B_PIN    DL_GPIO_PIN_7
#define ENCODER_GPIOB_PINS \
    (ENCODER_LEFT_A_PIN | ENCODER_LEFT_B_PIN | \
     ENCODER_RIGHT_A_PIN | ENCODER_RIGHT_B_PIN)

typedef struct {
    uint32_t pinA;
    uint32_t pinB;
    volatile int32_t count;
    uint8_t previousState;
} EncoderState;

static EncoderState gLeftEncoder = {ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN, 0, 0U};
static EncoderState gRightEncoder = {ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN, 0, 0U};

static const int8_t gQuadratureDelta[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0
};

static uint8_t Encoder_ReadState(const EncoderState *encoder)
{
    uint8_t state = 0U;
    uint32_t pins = DL_GPIO_readPins(GPIOB, encoder->pinA | encoder->pinB);

    if ((pins & encoder->pinA) != 0U) {
        state |= 0x01U;
    }

    if ((pins & encoder->pinB) != 0U) {
        state |= 0x02U;
    }

    return state;
}

static void Encoder_Update(EncoderState *encoder, int8_t direction)
{
    uint8_t currentState = Encoder_ReadState(encoder);
    uint8_t transition = (uint8_t)((encoder->previousState << 2U) | currentState);
    int8_t delta = gQuadratureDelta[transition & 0x0FU];

    encoder->previousState = currentState;
    encoder->count += (int32_t)delta * direction;
}

void Encoder_Init(void)
{
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM25,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM29,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM23,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM24,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    gLeftEncoder.previousState = Encoder_ReadState(&gLeftEncoder);
    gRightEncoder.previousState = Encoder_ReadState(&gRightEncoder);

    DL_GPIO_setLowerPinsPolarity(GPIOB,
        DL_GPIO_PIN_6_EDGE_RISE_FALL |
        DL_GPIO_PIN_7_EDGE_RISE_FALL |
        DL_GPIO_PIN_8_EDGE_RISE_FALL |
        DL_GPIO_PIN_12_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_GPIOB_PINS);
    DL_GPIO_enableInterrupt(GPIOB, ENCODER_GPIOB_PINS);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Encoder_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    gLeftEncoder.count = 0;
    gRightEncoder.count = 0;
    gLeftEncoder.previousState = Encoder_ReadState(&gLeftEncoder);
    gRightEncoder.previousState = Encoder_ReadState(&gRightEncoder);
    if (primask == 0U) {
        __enable_irq();
    }
}

void Encoder_GetAndResetDeltas(EncoderDelta *delta)
{
    uint32_t primask = __get_PRIMASK();
    int32_t left;
    int32_t right;

    __disable_irq();
    left = gLeftEncoder.count;
    right = gRightEncoder.count;
    gLeftEncoder.count = 0;
    gRightEncoder.count = 0;
    if (primask == 0U) {
        __enable_irq();
    }

    if (left > INT16_MAX) {
        left = INT16_MAX;
    } else if (left < INT16_MIN) {
        left = INT16_MIN;
    }

    if (right > INT16_MAX) {
        right = INT16_MAX;
    } else if (right < INT16_MIN) {
        right = INT16_MIN;
    }

    delta->left = (int16_t)left;
    delta->right = (int16_t)right;
}

int32_t Encoder_GetLeftCount(void)
{
    return gLeftEncoder.count;
}

int32_t Encoder_GetRightCount(void)
{
    return gRightEncoder.count;
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(GPIOB, ENCODER_GPIOB_PINS);

    if (pending != 0U) {
#if MOTOR_LEFT_ENCODER_REVERSE
        Encoder_Update(&gLeftEncoder, -1);
#else
        Encoder_Update(&gLeftEncoder, 1);
#endif

#if MOTOR_RIGHT_ENCODER_REVERSE
        Encoder_Update(&gRightEncoder, -1);
#else
        Encoder_Update(&gRightEncoder, 1);
#endif
        DL_GPIO_clearInterruptStatus(GPIOB, pending);
    }
}
