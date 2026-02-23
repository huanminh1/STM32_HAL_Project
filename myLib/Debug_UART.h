#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"

void Debug_Init(UART_HandleTypeDef *huart, SemaphoreHandle_t mutex);
void Debug_Print(const char *format, ...);

#endif