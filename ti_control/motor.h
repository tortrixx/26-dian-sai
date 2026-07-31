#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef struct {
    int16_t leftTarget;
    int16_t rightTarget;
    int16_t leftMeasured;
    int16_t rightMeasured;
    int16_t leftPwm;
    int16_t rightPwm;
} MotorStatus;

void Motor_Init(void);
void Motor_ResetController(void);
void Motor_SetSpeedTargets(int16_t leftTarget, int16_t rightTarget);
void Motor_Task(uint32_t nowMs);
void Motor_Stop(void);
void Motor_GetStatus(MotorStatus *status);

#endif
