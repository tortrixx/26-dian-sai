#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool attached;
    int16_t angleDeg;
} ServoStatus;

void Servo_Init(void);
void Servo_Attach(void);
void Servo_Detach(void);
void Servo_SetAngle(int16_t angleDeg);
void Servo_Task(void);
bool Servo_IsAttached(void);
void Servo_GetStatus(ServoStatus *status);

#endif
