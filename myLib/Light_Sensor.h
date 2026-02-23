#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "stm32f1xx_hal.h"

void LightSensor_Init(ADC_HandleTypeDef *hadc);
uint16_t LightSensor_ReadRaw(void);

#endif