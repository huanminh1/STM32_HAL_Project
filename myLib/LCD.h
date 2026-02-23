#ifndef LCD_H
#define LCD_H

#include "stm32f1xx_hal.h"
#include "string.h"

/* ===== LCD I2C address & control bits ===== */
#define LCD_ADDRESS     (0x27 << 1)

#define LCD_BACKLIGHT   0x08
#define LCD_EN          0x04
#define LCD_RS          0x01

/* ===== Public functions ===== */
void LCD_Init(void);
void LCD_Print(char *str);
void LCD_Clear(void);
void LCD_MoveCursor(uint8_t row, uint8_t col);
#endif