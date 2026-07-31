#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

void Oled_StartInit(void);
void Oled_Task(void);
bool Oled_IsReady(void);
void Oled_Clear(void);
void Oled_Update(void);
void Oled_SetPixel(uint8_t x, uint8_t y, bool on);
void Oled_DrawChar(uint8_t x, uint8_t y, char c);
void Oled_DrawString(uint8_t x, uint8_t y, const char *str);
void Oled_DrawHLine(uint8_t x, uint8_t y, uint8_t width);
void Oled_DrawBox(uint8_t x, uint8_t y, uint8_t width, uint8_t height, bool fill);

#endif
