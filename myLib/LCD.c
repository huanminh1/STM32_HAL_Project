#include "LCD.h"
#include "BSP_Delay.h"

/* ===== I2C handle from main.c ===== */
extern I2C_HandleTypeDef hi2c1;

/* ===== Private functions ===== */
static void LCD_send_CMD(uint8_t cmd);
static void LCD_send_Data(uint8_t data);

static void LCD_send_CMD(uint8_t cmd)
{
    uint8_t data_h, data_l;
    uint8_t data_t[4];

    data_h = cmd & 0xF0;
    data_l = (cmd << 4) & 0xF0;
		// RS = 0 : send cmd
		// Set EN high to latch data into the LCD, then pull it low to complete the write cycle. (2)
		// Send 4 bit data + 4 bit control (BACKLIGHT,EN,R/W,RS) + (2) => send 4 times for 8 bit.
		data_t[0] = data_h | LCD_EN | LCD_BACKLIGHT; 
    data_t[1] = data_h | LCD_BACKLIGHT;
    data_t[2] = data_l | LCD_EN | LCD_BACKLIGHT;
    data_t[3] = data_l | LCD_BACKLIGHT;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, data_t, 4, 100);
    BSP_Delay_ms(2);
}

static void LCD_send_Data(uint8_t data)
{
    uint8_t data_h, data_l;
    uint8_t data_t[4];

    data_h = data & 0xF0;
    data_l = (data << 4) & 0xF0;
		// RS = 1 : send data
		// Set EN high to latch data into the LCD, then pull it low to complete the write cycle. (2)
		// Send 4 bit data + 4 bit control (BACKLIGHT,EN,R/W,RS) + (2) => send 4 times for 8 bit.
    data_t[0] = data_h | LCD_EN | LCD_BACKLIGHT | LCD_RS;
    data_t[1] = data_h | LCD_BACKLIGHT | LCD_RS;
    data_t[2] = data_l | LCD_EN | LCD_BACKLIGHT | LCD_RS;
    data_t[3] = data_l | LCD_BACKLIGHT | LCD_RS;

    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDRESS, data_t, 4, 100);
    BSP_Delay_ms(2);
}

/* =============Public Function================ */

void LCD_Init(void)
{
    BSP_Delay_ms(50);

    LCD_send_CMD(0x30);
    BSP_Delay_ms(5);
    LCD_send_CMD(0x30);
    BSP_Delay_ms(5);
    LCD_send_CMD(0x30);
		BSP_Delay_ms(1);
    LCD_send_CMD(0x20);   // Switch to 4-bit mode

    LCD_send_CMD(0x28);   // 4-bit, 2 line
		LCD_send_CMD(0x0C);   // Display ON, cursor OFF
    LCD_send_CMD(0x01);   // Clear display
    BSP_Delay_ms(5);
    LCD_send_CMD(0x06);   // Entry mode set
    
}

void LCD_Clear(void)
{
	LCD_send_CMD(0x01);   // Clear display
  BSP_Delay_ms(5);
}

void LCD_MoveCursor(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_send_CMD(address);
}

void LCD_Print(char *str)
{
    while (*str)
    {
        LCD_send_Data((uint8_t)*str);
        str++;
    }
}