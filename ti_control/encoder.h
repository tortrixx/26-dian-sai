#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int16_t left;
    int16_t right;
} EncoderDelta;

void Encoder_Init(void);
void Encoder_Reset(void);
void Encoder_GetAndResetDeltas(EncoderDelta *delta);
int32_t Encoder_GetLeftCount(void);
int32_t Encoder_GetRightCount(void);

#endif
