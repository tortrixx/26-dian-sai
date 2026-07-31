#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 8U

typedef struct {
    uint8_t blackMask;
    uint8_t activeCount;
    bool lineLost;
    bool allBlack;
    int16_t position;
} LineSensorSample;

void LineSensor_Init(void);
void LineSensor_Read(LineSensorSample *sample);

#endif
