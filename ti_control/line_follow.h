#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    bool lineLost;
    bool allBlack;
    uint8_t sensorMask;
    int16_t lineError;
    int16_t turn;
    int16_t leftTarget;
    int16_t rightTarget;
    int16_t leftMeasured;
    int16_t rightMeasured;
    int16_t leftPwm;
    int16_t rightPwm;
} LineFollowStatus;

void LineFollow_Init(void);
void LineFollow_Start(void);
void LineFollow_Stop(void);
void LineFollow_Task(void);
bool LineFollow_IsActive(void);
void LineFollow_GetStatus(LineFollowStatus *status);

#endif
