#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"

void BSP_Delay_Init(void);
void BSP_Delay_us(uint16_t us);
void BSP_Delay_ms(uint32_t ms);

#endif