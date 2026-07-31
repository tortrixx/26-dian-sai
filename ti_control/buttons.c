#include "buttons.h"

#include "ti_msp_dl_config.h"

#define BUTTON_PORT GPIOB
#define BUTTON_S1_PIN DL_GPIO_PIN_15
#define BUTTON_S2_PIN DL_GPIO_PIN_17
#define BUTTON_S3_PIN DL_GPIO_PIN_18
#define BUTTON_S4_PIN DL_GPIO_PIN_19

#define BUTTON_S1_IOMUX IOMUX_PINCM32
#define BUTTON_S2_IOMUX IOMUX_PINCM43
#define BUTTON_S3_IOMUX IOMUX_PINCM44
#define BUTTON_S4_IOMUX IOMUX_PINCM45

#define BUTTON_DEBOUNCE_TICKS 4U

typedef struct {
    uint32_t pinMask;
    uint32_t iopincm;
    ButtonEvent event;
    bool stablePressed;
    uint8_t debounceTicks;
} ButtonState;

static ButtonState gButtons[BUTTON_ID_COUNT] = {
    {BUTTON_S1_PIN, BUTTON_S1_IOMUX, BUTTON_EVENT_S1_PRESSED, false, 0},
    {BUTTON_S2_PIN, BUTTON_S2_IOMUX, BUTTON_EVENT_S2_PRESSED, false, 0},
    {BUTTON_S3_PIN, BUTTON_S3_IOMUX, BUTTON_EVENT_S3_PRESSED, false, 0},
    {BUTTON_S4_PIN, BUTTON_S4_IOMUX, BUTTON_EVENT_S4_PRESSED, false, 0},
};

static bool Buttons_ReadRaw(uint32_t pinMask)
{
    return (DL_GPIO_readPins(BUTTON_PORT, pinMask) == 0U);
}

void Buttons_Init(void)
{
    uint8_t index;

    DL_GPIO_enablePower(BUTTON_PORT);
    delay_cycles(POWER_STARTUP_DELAY);

    for (index = 0; index < BUTTON_ID_COUNT; index++) {
        DL_GPIO_initDigitalInputFeatures(gButtons[index].iopincm,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
            DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    }
}

ButtonEvent Buttons_Poll(void)
{
    uint8_t index;

    for (index = 0; index < BUTTON_ID_COUNT; index++) {
        bool rawPressed = Buttons_ReadRaw(gButtons[index].pinMask);

        if (rawPressed == gButtons[index].stablePressed) {
            gButtons[index].debounceTicks = 0;
            continue;
        }

        gButtons[index].debounceTicks++;
        if (gButtons[index].debounceTicks < BUTTON_DEBOUNCE_TICKS) {
            continue;
        }

        gButtons[index].stablePressed = rawPressed;
        gButtons[index].debounceTicks = 0;

        if (rawPressed) {
            return gButtons[index].event;
        }
    }

    return BUTTON_EVENT_NONE;
}

bool Buttons_IsPressed(ButtonId id)
{
    if (id >= BUTTON_ID_COUNT) {
        return false;
    }

    return gButtons[id].stablePressed;
}
