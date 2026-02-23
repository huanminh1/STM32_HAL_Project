#include "debug_uart.h"
#include <stdarg.h>
#include <stdio.h>

static UART_HandleTypeDef *debug_uart;
static SemaphoreHandle_t debug_mutex;

void Debug_Init(UART_HandleTypeDef *huart, SemaphoreHandle_t mutex)
{
  debug_uart = huart;
	debug_mutex = mutex;
}

void Debug_Print(const char *format, ...)
{
    char buffer[128];

    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
	
		if(debug_mutex != NULL)
    {
        xSemaphoreTake(debug_mutex, portMAX_DELAY);
    }

    HAL_UART_Transmit(debug_uart, (uint8_t*)buffer, len, 100);

    if(debug_mutex != NULL)
    {
        xSemaphoreGive(debug_mutex);
    }
}