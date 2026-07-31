#ifndef K230_UART_H
#define K230_UART_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    bool tracked;
    int16_t xCmX100;
    int16_t yOffsetPx;
    uint8_t quality;
    uint8_t sequence;
    uint32_t receivedMs;
} K230VisionFrame;

typedef struct {
    bool hasFrame;
    uint32_t rxBytes;
    uint32_t rxOk;
    uint32_t rxBad;
    uint32_t rxHeadAA;
    uint32_t rxHeadAA55;
    uint8_t lastByte;
    uint8_t lastLength;
    uint8_t lastType;
    uint8_t lastChecksumRx;
    uint8_t lastChecksumCalc;
    uint32_t lastFrameMs;
    K230VisionFrame latest;
} K230UartStatus;

void K230Uart_Init(void);
void K230Uart_Task(void);
bool K230Uart_GetLatest(K230VisionFrame *frame);
void K230Uart_GetStatus(K230UartStatus *status);

#endif
