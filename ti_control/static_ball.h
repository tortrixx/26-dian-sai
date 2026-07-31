#ifndef STATIC_BALL_H
#define STATIC_BALL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    bool visionFresh;
    bool visionValid;
    uint8_t phase;
    uint8_t quality;
    int16_t ballXCmX100;
    int16_t targetCmX100;
    int16_t velocityCmSX100;
    int16_t tiltDegX10;
    int16_t servoAngleDeg;
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
} StaticBallStatus;

void StaticBall_Init(void);
void StaticBall_Start(void);
void StaticBall_Stop(void);
void StaticBall_Exit(void);
void StaticBall_Task(void);
bool StaticBall_IsActive(void);
void StaticBall_GetStatus(StaticBallStatus *status);

#endif
