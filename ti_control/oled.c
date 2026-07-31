#include "oled.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define OLED_I2C_ADDR 0x3CU
#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U
#define OLED_PAGE_COUNT (OLED_HEIGHT / 8U)
#define OLED_BUFFER_SIZE (OLED_WIDTH * OLED_PAGE_COUNT)
#define OLED_CONTROL_CMD 0x00U
#define OLED_CONTROL_DATA 0x40U
#define OLED_I2C_TIMEOUT 1000000U
#define OLED_TRANSFER_GAP_CYCLES 1000U
#define OLED_INIT_POWER_WAIT_TICKS 40U
#define OLED_INIT_RETRY_DELAY_SHORT_TICKS 80U
#define OLED_INIT_RETRY_DELAY_LONG_TICKS 160U

static uint8_t gOledBuffer[OLED_BUFFER_SIZE];

typedef enum {
    OLED_STATE_OFF = 0,
    OLED_STATE_POWER_WAIT,
    OLED_STATE_INIT_SEND,
    OLED_STATE_READY,
    OLED_STATE_RETRY_WAIT
} OledState;

static const uint8_t gOledInitCommands[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
    0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xA4, 0xA6,
    0xD9, 0xF1, 0xDB, 0x40, 0x20, 0x00, 0x8D, 0x14, 0xAF
};

static OledState gOledState = OLED_STATE_OFF;
static uint8_t gOledInitIndex = 0U;
static uint16_t gOledWaitTicks = 0U;
static uint8_t gOledRetryCount = 0U;

static const uint8_t gFont6x8[][6] = {
    [0] = {0, 0, 0, 0, 0, 0},
    [' '] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    ['+'] = {0x10, 0x10, 0x7C, 0x10, 0x10, 0x00},
    ['-'] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x00},
    ['.'] = {0x00, 0x60, 0x60, 0x00, 0x00, 0x00},
    ['/'] = {0x20, 0x10, 0x08, 0x04, 0x02, 0x00},
    ['0'] = {0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00},
    ['1'] = {0x00, 0x42, 0x7F, 0x40, 0x00, 0x00},
    ['2'] = {0x42, 0x61, 0x51, 0x49, 0x46, 0x00},
    ['3'] = {0x21, 0x41, 0x45, 0x4B, 0x31, 0x00},
    ['4'] = {0x18, 0x14, 0x12, 0x7F, 0x10, 0x00},
    ['5'] = {0x27, 0x45, 0x45, 0x45, 0x39, 0x00},
    ['6'] = {0x3C, 0x4A, 0x49, 0x49, 0x30, 0x00},
    ['7'] = {0x01, 0x71, 0x09, 0x05, 0x03, 0x00},
    ['8'] = {0x36, 0x49, 0x49, 0x49, 0x36, 0x00},
    ['9'] = {0x06, 0x49, 0x49, 0x29, 0x1E, 0x00},
    [':'] = {0x00, 0x36, 0x36, 0x00, 0x00, 0x00},
    ['A'] = {0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00},
    ['B'] = {0x7F, 0x49, 0x49, 0x49, 0x36, 0x00},
    ['C'] = {0x3E, 0x41, 0x41, 0x41, 0x22, 0x00},
    ['D'] = {0x7F, 0x41, 0x41, 0x22, 0x1C, 0x00},
    ['E'] = {0x7F, 0x49, 0x49, 0x49, 0x41, 0x00},
    ['F'] = {0x7F, 0x09, 0x09, 0x09, 0x01, 0x00},
    ['G'] = {0x3E, 0x41, 0x49, 0x49, 0x7A, 0x00},
    ['H'] = {0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00},
    ['I'] = {0x00, 0x41, 0x7F, 0x41, 0x00, 0x00},
    ['J'] = {0x20, 0x40, 0x41, 0x3F, 0x01, 0x00},
    ['K'] = {0x7F, 0x08, 0x14, 0x22, 0x41, 0x00},
    ['L'] = {0x7F, 0x40, 0x40, 0x40, 0x40, 0x00},
    ['M'] = {0x7F, 0x02, 0x0C, 0x02, 0x7F, 0x00},
    ['N'] = {0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00},
    ['O'] = {0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00},
    ['P'] = {0x7F, 0x09, 0x09, 0x09, 0x06, 0x00},
    ['Q'] = {0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00},
    ['R'] = {0x7F, 0x09, 0x19, 0x29, 0x46, 0x00},
    ['S'] = {0x46, 0x49, 0x49, 0x49, 0x31, 0x00},
    ['T'] = {0x01, 0x01, 0x7F, 0x01, 0x01, 0x00},
    ['U'] = {0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00},
    ['V'] = {0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00},
    ['W'] = {0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00},
    ['X'] = {0x63, 0x14, 0x08, 0x14, 0x63, 0x00},
    ['Y'] = {0x07, 0x08, 0x70, 0x08, 0x07, 0x00},
    ['Z'] = {0x61, 0x51, 0x49, 0x45, 0x43, 0x00},
    ['_'] = {0x40, 0x40, 0x40, 0x40, 0x40, 0x00},
    ['a'] = {0x20, 0x54, 0x54, 0x54, 0x78, 0x00},
    ['b'] = {0x7F, 0x48, 0x44, 0x44, 0x38, 0x00},
    ['c'] = {0x38, 0x44, 0x44, 0x44, 0x20, 0x00},
    ['d'] = {0x38, 0x44, 0x44, 0x48, 0x7F, 0x00},
    ['e'] = {0x38, 0x54, 0x54, 0x54, 0x18, 0x00},
    ['f'] = {0x08, 0x7E, 0x09, 0x01, 0x02, 0x00},
    ['g'] = {0x0C, 0x52, 0x52, 0x52, 0x3E, 0x00},
    ['h'] = {0x7F, 0x08, 0x04, 0x04, 0x78, 0x00},
    ['i'] = {0x00, 0x44, 0x7D, 0x40, 0x00, 0x00},
    ['j'] = {0x20, 0x40, 0x44, 0x3D, 0x00, 0x00},
    ['k'] = {0x7F, 0x10, 0x28, 0x44, 0x00, 0x00},
    ['l'] = {0x00, 0x41, 0x7F, 0x40, 0x00, 0x00},
    ['m'] = {0x7C, 0x04, 0x18, 0x04, 0x78, 0x00},
    ['n'] = {0x7C, 0x08, 0x04, 0x04, 0x78, 0x00},
    ['o'] = {0x38, 0x44, 0x44, 0x44, 0x38, 0x00},
    ['p'] = {0x7C, 0x14, 0x14, 0x14, 0x08, 0x00},
    ['q'] = {0x08, 0x14, 0x14, 0x18, 0x7C, 0x00},
    ['r'] = {0x7C, 0x08, 0x04, 0x04, 0x08, 0x00},
    ['s'] = {0x48, 0x54, 0x54, 0x54, 0x20, 0x00},
    ['t'] = {0x04, 0x3F, 0x44, 0x40, 0x20, 0x00},
    ['u'] = {0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00},
    ['v'] = {0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00},
    ['w'] = {0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00},
    ['x'] = {0x44, 0x28, 0x10, 0x28, 0x44, 0x00},
    ['y'] = {0x0C, 0x50, 0x50, 0x50, 0x3C, 0x00},
    ['z'] = {0x44, 0x64, 0x54, 0x4C, 0x44, 0x00},
    ['>'] = {0x00, 0x41, 0x22, 0x14, 0x08, 0x00},
};

static void Oled_ClearBuffer(void)
{
    uint16_t index;

    for (index = 0; index < OLED_BUFFER_SIZE; index++) {
        gOledBuffer[index] = 0U;
    }
}

static bool Oled_WaitIdle(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (timeout-- == 0U) {
            return false;
        }
    }

    return true;
}

static bool Oled_WaitDone(void)
{
    uint32_t timeout = OLED_I2C_TIMEOUT;

    while (DL_I2C_getControllerStatus(I2C_0_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
        if (DL_I2C_getControllerStatus(I2C_0_INST) &
            DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        if (timeout-- == 0U) {
            return false;
        }
    }

    return ((DL_I2C_getControllerStatus(I2C_0_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) == 0U);
}

static bool Oled_Write(const uint8_t *data, uint16_t length)
{
    uint16_t transferred = 0U;

    if (!Oled_WaitIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    transferred = DL_I2C_fillControllerTXFIFO(I2C_0_INST, data, length);
    DL_I2C_startControllerTransfer(I2C_0_INST, OLED_I2C_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(OLED_TRANSFER_GAP_CYCLES);

    while (transferred < length) {
        while (DL_I2C_getRawInterruptStatus(
                   I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY) == 0U) {
            if (DL_I2C_getControllerStatus(I2C_0_INST) &
                DL_I2C_CONTROLLER_STATUS_ERROR) {
                return false;
            }
        }

        transferred += DL_I2C_fillControllerTXFIFO(
            I2C_0_INST, &data[transferred], (uint16_t)(length - transferred));
    }

    return Oled_WaitDone();
}

static bool Oled_WriteCommand(uint8_t command)
{
    uint8_t packet[2] = {OLED_CONTROL_CMD, command};
    return Oled_Write(packet, 2U);
}

static bool Oled_SetAddressWindow(uint8_t page)
{
    return Oled_WriteCommand((uint8_t)(0xB0U + page)) &&
           Oled_WriteCommand(0x00U) &&
           Oled_WriteCommand(0x10U);
}

static void Oled_BeginRetry(void)
{
    gOledRetryCount++;
    gOledInitIndex = 0U;
    gOledWaitTicks = (gOledRetryCount < 3U)
        ? OLED_INIT_RETRY_DELAY_SHORT_TICKS
        : OLED_INIT_RETRY_DELAY_LONG_TICKS;
    gOledState = OLED_STATE_RETRY_WAIT;
}

void Oled_StartInit(void)
{
    gOledInitIndex = 0U;
    gOledRetryCount = 0U;
    gOledWaitTicks = OLED_INIT_POWER_WAIT_TICKS;
    gOledState = OLED_STATE_POWER_WAIT;
    Oled_ClearBuffer();
}

void Oled_Task(void)
{
    switch (gOledState) {
        case OLED_STATE_OFF:
            break;
        case OLED_STATE_POWER_WAIT:
        case OLED_STATE_RETRY_WAIT:
            if (gOledWaitTicks > 0U) {
                gOledWaitTicks--;
            } else {
                gOledState = OLED_STATE_INIT_SEND;
            }
            break;
        case OLED_STATE_INIT_SEND:
            if (gOledInitIndex >= sizeof(gOledInitCommands)) {
                Oled_Clear();
                Oled_Update();
                gOledState = OLED_STATE_READY;
                break;
            }

            if (Oled_WriteCommand(gOledInitCommands[gOledInitIndex])) {
                gOledInitIndex++;
            } else {
                Oled_BeginRetry();
            }
            break;
        case OLED_STATE_READY:
        default:
            break;
    }
}

bool Oled_IsReady(void)
{
    return (gOledState == OLED_STATE_READY);
}

void Oled_Clear(void)
{
    Oled_ClearBuffer();
}

void Oled_Update(void)
{
    uint8_t page;
    uint8_t packet[OLED_WIDTH + 1U];
    uint16_t offset;

    packet[0] = OLED_CONTROL_DATA;
    for (page = 0; page < OLED_PAGE_COUNT; page++) {
        offset = (uint16_t) page * OLED_WIDTH;
        if (!Oled_IsReady()) {
            return;
        }

        if (!Oled_SetAddressWindow(page)) {
            return;
        }

        for (uint16_t column = 0; column < OLED_WIDTH; column++) {
            packet[column + 1U] = gOledBuffer[offset + column];
        }

        if (!Oled_Write(packet, sizeof(packet))) {
            return;
        }
    }
}

void Oled_SetPixel(uint8_t x, uint8_t y, bool on)
{
    uint16_t index;
    uint8_t bitMask;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return;
    }

    index = (uint16_t)(y / 8U) * OLED_WIDTH + x;
    bitMask = (uint8_t)(1U << (y % 8U));

    if (on) {
        gOledBuffer[index] |= bitMask;
    } else {
        gOledBuffer[index] &= (uint8_t)(~bitMask);
    }
}

void Oled_DrawChar(uint8_t x, uint8_t y, char c)
{
    uint8_t column;
    const uint8_t *glyph;

    if ((x > (OLED_WIDTH - 6U)) || (y > (OLED_HEIGHT - 8U))) {
        return;
    }

    glyph = gFont6x8[(uint8_t) c];
    for (column = 0; column < 6U; column++) {
        uint8_t bits = glyph[column];
        for (uint8_t row = 0; row < 8U; row++) {
            Oled_SetPixel((uint8_t)(x + column), (uint8_t)(y + row),
                ((bits >> row) & 0x01U) != 0U);
        }
    }
}

void Oled_DrawString(uint8_t x, uint8_t y, const char *str)
{
    while ((*str != '\0') && (x <= (OLED_WIDTH - 6U))) {
        Oled_DrawChar(x, y, *str);
        x = (uint8_t)(x + 6U);
        str++;
    }
}

void Oled_DrawHLine(uint8_t x, uint8_t y, uint8_t width)
{
    uint8_t column;

    for (column = 0; column < width; column++) {
        Oled_SetPixel((uint8_t)(x + column), y, true);
    }
}

void Oled_DrawBox(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool fill)
{
    uint8_t dx;
    uint8_t dy;

    for (dy = 0; dy < height; dy++) {
        for (dx = 0; dx < width; dx++) {
            bool border = (dx == 0U) || (dy == 0U) ||
                          (dx == (uint8_t)(width - 1U)) ||
                          (dy == (uint8_t)(height - 1U));
            if (fill || border) {
                Oled_SetPixel((uint8_t)(x + dx), (uint8_t)(y + dy), true);
            }
        }
    }
}
