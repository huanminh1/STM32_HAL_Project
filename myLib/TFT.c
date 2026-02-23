#include "TFT.h"
#include "BSP_Delay.h"

/* ================= EXTERN HANDLE ================= */
extern SPI_HandleTypeDef hspi1;

static void TFT_WriteCmd(uint8_t cmd)
{  
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0); 
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 0); //A0 = 0 -> CMD
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 500);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1); 
}

static void TFT_WriteData(uint8_t data)
{
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);	
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, 1); //A0 = 1 -> DATA
	HAL_SPI_Transmit(&hspi1, &data, 1, 500);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}

static void TFT_SetPos(uint8_t xs, uint8_t xe,
                           uint8_t ys, uint8_t ye)
{
    // Column
    TFT_WriteCmd(0x2A);
    TFT_WriteData(0x00);  // XS high
    TFT_WriteData(xs);		// XS low
    TFT_WriteData(0x00);	// XE high
    TFT_WriteData(xe);		// XE low

    // Row
    TFT_WriteCmd(0x2B);		
    TFT_WriteData(0x00);	// YS high
    TFT_WriteData(ys);		// YS low
    TFT_WriteData(0x00);	// YE high
    TFT_WriteData(ye);		// YE low
}

/* ================= INIT ================= */

static void TFT_SendCmdList(const uint8_t *list)
{
	uint8_t cmd = 0;
	uint8_t len = 0;
	
	while(1)
	{
		cmd = *list++;
		len = *list++;
		if(cmd == LCD_CMD_END)
		{
			break;
		}
		else
		{
			TFT_WriteCmd(cmd);
			for(uint8_t i = 0; i < len; i++ )
			{
				TFT_WriteData(*list++);
			}
		}
	}
}

void TFT_Init(void)
{
    // ===== Hardware reset =====
    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
    BSP_Delay_ms(20);
    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
    BSP_Delay_ms(200);
	
		// ===== Software reset =====
    TFT_WriteCmd(0x01); 
    BSP_Delay_ms(100);

		// ===== Sleep out & booster on =====
    TFT_WriteCmd(0x11);
    BSP_Delay_ms(120);
	
		// ===== Command List =====
    TFT_SendCmdList(u8InitCmdList);

		// ===== Memory Data Access Control =====
    TFT_WriteCmd(0x36);
    TFT_WriteData(0x00); // Set RGB
//  TFT_WriteData(0x08); // Set BRG

		// ===== Interface Pixel Format =====
    TFT_WriteCmd(0x3A);
    TFT_WriteData(0x05); // 16bit

		// ===== Display inversion OFF  =====
    TFT_WriteCmd(0x29); // Display ON
    BSP_Delay_ms(100);
}
void CS_Init(void)
{
    HAL_GPIO_WritePin(CS_PORT, CS_PIN, 1);
}

/* ================= DRAW ================= */
void TFT_FillScreen(uint16_t color)
{
    TFT_SetPos(0, TFT_WIDTH-1, 0, TFT_HEIGHT-1);
    TFT_WriteCmd(0x2C);

    for (uint32_t i = 0; i < TFT_WIDTH*TFT_HEIGHT; i++)
    {
        TFT_WriteData(color >> 8);
        TFT_WriteData(color & 0xFF);
    }
}

void TFT_DrawPixel(uint8_t x, uint8_t y, uint16_t color)
{
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT)
        return;

    TFT_SetPos(x, x+1, y+1, y);
    TFT_WriteCmd(0x2C);

    TFT_WriteData(color >> 8);
    TFT_WriteData(color & 0xFF);
}

void TFT_DrawChar(uint8_t x, uint8_t y,
                  char ch,
                  FontDef font,
                  uint16_t color,
                  uint16_t bg)
{
    for (uint8_t i = 0; i < font.height; i++)
    {
        uint16_t row = font.data[(ch - 32)*font.height + i];

        for (uint8_t j = 0; j < font.width; j++)
        {
            if ((row << j) & 0x8000)
                TFT_DrawPixel(x+j, y+i, color);
            else
                TFT_DrawPixel(x+j, y+i, bg);
        }
    }
}

void TFT_DrawString(uint8_t x, uint8_t y,
                    char *str,
                    FontDef font,
                    uint16_t color,
                    uint16_t bg)
{
    while (*str)
    {
        TFT_DrawChar(x, y, *str, font, color, bg);
        x += font.width;
        str++;
    }	
}

void TFT_DrawBitmap(uint8_t x, uint8_t y,
                    uint8_t w, uint8_t h,
                    const uint16_t *img)
{
    if (x + w > TFT_WIDTH || y + h > TFT_HEIGHT)
        return;

    TFT_SetPos(x, x+w-1, y, y+h-1);
    TFT_WriteCmd(0x2C);

    for (uint32_t i = 0; i < w*h; i++)
    {
        TFT_WriteData(img[i] >> 8);
        TFT_WriteData(img[i] & 0xFF);
    }
}

