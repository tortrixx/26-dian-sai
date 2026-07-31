#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BUTTON_ID_S1 = 0,
    BUTTON_ID_S2,
    BUTTON_ID_S3,
    BUTTON_ID_S4,
    BUTTON_ID_COUNT
} ButtonId;

typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_S1_PRESSED,
    BUTTON_EVENT_S2_PRESSED,
    BUTTON_EVENT_S3_PRESSED,
    BUTTON_EVENT_S4_PRESSED
} ButtonEvent;

void Buttons_Init(void);
ButtonEvent Buttons_Poll(void);
bool Buttons_IsPressed(ButtonId id);

#endif
