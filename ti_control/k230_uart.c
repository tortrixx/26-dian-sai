#include "k230_uart.h"

#include "system_time.h"
#include "ti_msp_dl_config.h"

#define K230_UART_INST                          UART2
#define K230_UART_BAUD_RATE                     115200U

#define K230_UART_RX_IOMUX                      IOMUX_PINCM33
#define K230_UART_RX_IOMUX_FUNC                 IOMUX_PINCM33_PF_UART2_RX
#define K230_UART_TX_IOMUX                      IOMUX_PINCM46
#define K230_UART_TX_IOMUX_FUNC                 IOMUX_PINCM46_PF_UART2_TX

#define K230_PROTO_HEAD0                        0xAAU
#define K230_PROTO_HEAD1                        0x55U
#define K230_PROTO_MAX_PAYLOAD                  32U
#define K230_MSG_VISION_TARGET                  0x01U
#define K230_VISION_FLAG_VALID                  0x01U
#define K230_VISION_FLAG_TRACKED                0x02U
#define K230_VISION_PAYLOAD_LEN                 6U

typedef enum {
    K230_RX_WAIT_HEAD0 = 0,
    K230_RX_WAIT_HEAD1,
    K230_RX_WAIT_LENGTH,
    K230_RX_WAIT_BODY,
    K230_RX_WAIT_CHECKSUM
} K230RxState;

typedef struct {
    K230RxState state;
    uint8_t length;
    uint8_t pos;
    uint8_t checksum;
    uint8_t body[K230_PROTO_MAX_PAYLOAD + 2U];
} K230RxParser;

#define K230_RX_STREAM_BUF_SIZE                 64U
#define K230_UART_RING_SIZE                     128U
#define K230_UART_RING_MASK                     (K230_UART_RING_SIZE - 1U)

static K230RxParser gParser = {K230_RX_WAIT_HEAD0, 0U, 0U, 0U, {0U}};
static uint8_t gStreamBuf[K230_RX_STREAM_BUF_SIZE];
static uint8_t gStreamLen = 0U;
static volatile uint8_t gRingBuf[K230_UART_RING_SIZE];
static volatile uint8_t gRingHead = 0U;
static volatile uint8_t gRingTail = 0U;
static K230UartStatus gStatus = {
    false, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    {false, false, 0, 0, 0U, 0U, 0U}
};

static int16_t K230_GetI16Le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void K230_ResetParser(void)
{
    gParser.state = K230_RX_WAIT_HEAD0;
    gParser.length = 0U;
    gParser.pos = 0U;
    gParser.checksum = 0U;
    gStreamLen = 0U;
    gRingHead = 0U;
    gRingTail = 0U;
}

static void K230_OnVisionPayload(const uint8_t *payload, uint8_t sequence)
{
    uint32_t nowMs = SystemTime_Millis();

    gStatus.latest.valid = (payload[0] & K230_VISION_FLAG_VALID) != 0U;
    gStatus.latest.tracked = (payload[0] & K230_VISION_FLAG_TRACKED) != 0U;
    gStatus.latest.xCmX100 = K230_GetI16Le(&payload[1]);
    gStatus.latest.yOffsetPx = K230_GetI16Le(&payload[3]);
    gStatus.latest.quality = payload[5];
    gStatus.latest.sequence = sequence;
    gStatus.latest.receivedMs = nowMs;
    gStatus.lastFrameMs = nowMs;
    gStatus.hasFrame = true;
    gStatus.rxOk++;
}

static void K230_ParseByte(uint8_t byte)
{
    uint8_t index;

    if (byte == K230_PROTO_HEAD0) {
        gStatus.rxHeadAA++;
    }

    if (gStreamLen >= K230_RX_STREAM_BUF_SIZE) {
        for (index = 1U; index < gStreamLen; index++) {
            gStreamBuf[index - 1U] = gStreamBuf[index];
        }
        gStreamLen--;
    }
    gStreamBuf[gStreamLen] = byte;
    gStreamLen++;

    while (gStreamLen >= 4U) {
        uint8_t length;
        uint8_t frameLen;
        uint8_t checksum;

        if (gStreamBuf[0] != K230_PROTO_HEAD0) {
            for (index = 1U; index < gStreamLen; index++) {
                gStreamBuf[index - 1U] = gStreamBuf[index];
            }
            gStreamLen--;
            continue;
        }

        if (gStreamBuf[1] != K230_PROTO_HEAD1) {
            for (index = 1U; index < gStreamLen; index++) {
                gStreamBuf[index - 1U] = gStreamBuf[index];
            }
            gStreamLen--;
            continue;
        }

        gStatus.rxHeadAA55++;
        length = gStreamBuf[2];
        gStatus.lastLength = length;

        if ((length < 2U) || (length > (K230_PROTO_MAX_PAYLOAD + 2U))) {
            gStatus.rxBad++;
            for (index = 1U; index < gStreamLen; index++) {
                gStreamBuf[index - 1U] = gStreamBuf[index];
            }
            gStreamLen--;
            continue;
        }

        frameLen = (uint8_t)(2U + 1U + length + 1U);
        if (gStreamLen < frameLen) {
            return;
        }

        checksum = 0U;
        for (index = 2U; index < (frameLen - 1U); index++) {
            checksum = (uint8_t)(checksum + gStreamBuf[index]);
        }

        gStatus.lastType = gStreamBuf[3];
        gStatus.lastChecksumRx = gStreamBuf[frameLen - 1U];
        gStatus.lastChecksumCalc = checksum;

        if ((gStreamBuf[frameLen - 1U] == checksum) &&
            (length == (K230_VISION_PAYLOAD_LEN + 2U)) &&
            (gStreamBuf[3] == K230_MSG_VISION_TARGET)) {
            K230_OnVisionPayload(&gStreamBuf[5], gStreamBuf[4]);

            for (index = frameLen; index < gStreamLen; index++) {
                gStreamBuf[index - frameLen] = gStreamBuf[index];
            }
            gStreamLen = (uint8_t)(gStreamLen - frameLen);
        } else {
            gStatus.rxBad++;
            for (index = 1U; index < gStreamLen; index++) {
                gStreamBuf[index - 1U] = gStreamBuf[index];
            }
            gStreamLen--;
        }
    }
}

static void K230_RingPushFromIsr(uint8_t byte)
{
    uint8_t nextHead = (uint8_t)((gRingHead + 1U) & K230_UART_RING_MASK);

    if (nextHead == gRingTail) {
        gRingTail = (uint8_t)((gRingTail + 1U) & K230_UART_RING_MASK);
        gStatus.rxBad++;
    }

    gRingBuf[gRingHead] = byte;
    gRingHead = nextHead;
}

static bool K230_RingPop(uint8_t *byte)
{
    if (gRingTail == gRingHead) {
        return false;
    }

    *byte = gRingBuf[gRingTail];
    gRingTail = (uint8_t)((gRingTail + 1U) & K230_UART_RING_MASK);
    return true;
}

void K230Uart_Init(void)
{
    static const DL_UART_Main_ClockConfig clockConfig = {
        .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
        .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
    };
    static const DL_UART_Main_Config uartConfig = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = DL_UART_MAIN_PARITY_NONE,
        .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits = DL_UART_MAIN_STOP_BITS_ONE
    };

    DL_UART_Main_reset(K230_UART_INST);
    DL_UART_Main_enablePower(K230_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralInputFunctionFeatures(K230_UART_RX_IOMUX,
        K230_UART_RX_IOMUX_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralOutputFunction(K230_UART_TX_IOMUX,
        K230_UART_TX_IOMUX_FUNC);

    DL_UART_Main_setClockConfig(K230_UART_INST,
        (DL_UART_Main_ClockConfig *) &clockConfig);
    DL_UART_Main_init(K230_UART_INST, (DL_UART_Main_Config *) &uartConfig);
    DL_UART_Main_configBaudRate(K230_UART_INST, CPUCLK_FREQ,
        K230_UART_BAUD_RATE);
    DL_UART_Main_enableFIFOs(K230_UART_INST);
    DL_UART_Main_setRXFIFOThreshold(K230_UART_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(K230_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX | DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    DL_UART_Main_enable(K230_UART_INST);

    K230_ResetParser();
    NVIC_ClearPendingIRQ(UART2_INT_IRQn);
    NVIC_EnableIRQ(UART2_INT_IRQn);
}

void K230Uart_Task(void)
{
    uint8_t byte;

    while (K230_RingPop(&byte)) {
        gStatus.rxBytes++;
        gStatus.lastByte = byte;
        K230_ParseByte(byte);
    }

    while (DL_UART_Main_receiveDataCheck(K230_UART_INST, &byte)) {
        gStatus.rxBytes++;
        gStatus.lastByte = byte;
        K230_ParseByte(byte);
    }
}

bool K230Uart_GetLatest(K230VisionFrame *frame)
{
    if ((frame == 0) || !gStatus.hasFrame) {
        return false;
    }

    *frame = gStatus.latest;
    return true;
}

void K230Uart_GetStatus(K230UartStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = gStatus;
}

void UART2_IRQHandler(void)
{
    uint8_t byte;

    switch (DL_UART_Main_getPendingInterrupt(K230_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
            while (DL_UART_Main_receiveDataCheck(K230_UART_INST, &byte)) {
                K230_RingPushFromIsr(byte);
            }
            break;
        default:
            break;
    }

    while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST)) {
        K230_RingPushFromIsr(DL_UART_Main_receiveData(K230_UART_INST));
    }
}
