#include "line_sensor.h"

#include "line_follow_config.h"
#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t pinMask;
    uint32_t iopincm;
} LineSensorPin;

static const LineSensorPin gLineSensorPins[LINE_SENSOR_COUNT] = {
    {GPIOA, DL_GPIO_PIN_15, IOMUX_PINCM37},
    {GPIOA, DL_GPIO_PIN_17, IOMUX_PINCM39},
    {GPIOA, DL_GPIO_PIN_22, IOMUX_PINCM47},
    {GPIOA, DL_GPIO_PIN_24, IOMUX_PINCM54},
    {GPIOA, DL_GPIO_PIN_25, IOMUX_PINCM55},
    {GPIOB, DL_GPIO_PIN_9, IOMUX_PINCM26},
    {GPIOA, DL_GPIO_PIN_27, IOMUX_PINCM60},
    {GPIOB, DL_GPIO_PIN_20, IOMUX_PINCM48},
};

static const int16_t gLineSensorWeights[LINE_SENSOR_COUNT] = {
    -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
};

void LineSensor_Init(void)
{
    uint8_t index;

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);

    for (index = 0U; index < LINE_SENSOR_COUNT; index++) {
        DL_GPIO_initDigitalInputFeatures(gLineSensorPins[index].iopincm,
            DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
            DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    }
}

void LineSensor_Read(LineSensorSample *sample)
{
    int32_t weightedSum = 0;
    uint8_t index;
    uint8_t enabledCount = 0U;

    sample->blackMask = 0U;
    sample->activeCount = 0U;
    sample->lineLost = false;
    sample->allBlack = false;
    sample->position = 0;

    for (index = 0U; index < LINE_SENSOR_COUNT; index++) {
        bool isHigh = (DL_GPIO_readPins(gLineSensorPins[index].port,
                          gLineSensorPins[index].pinMask) != 0U);
        uint8_t logicalIndex = index;

#if LINE_SENSOR_REVERSE_ORDER
        logicalIndex = (uint8_t)(LINE_SENSOR_COUNT - 1U - index);
#endif

        if ((LINE_SENSOR_ENABLE_MASK & (uint8_t)(1U << index)) == 0U) {
            continue;
        }

        enabledCount++;

#if LINE_SENSOR_BLACK_IS_LOW
        if (!isHigh)
#else
        if (isHigh)
#endif
        {
            sample->blackMask |= (uint8_t)(1U << logicalIndex);
            sample->activeCount++;
            weightedSum += gLineSensorWeights[logicalIndex];
        }
    }

    if (sample->activeCount == 0U) {
        sample->lineLost = true;
        return;
    }

    sample->allBlack = (sample->activeCount == enabledCount);
    sample->position = (int16_t)(weightedSum / sample->activeCount);
}
