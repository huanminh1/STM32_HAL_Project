#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct
{
    uint8_t temperature;
    uint8_t humidity;
} DHT11_Data_t;

void DHT11_Init(GPIO_TypeDef *port, uint16_t pin);
uint8_t DHT11_Read(DHT11_Data_t *data);

#endif