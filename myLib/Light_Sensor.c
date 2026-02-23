#include "light_sensor.h"

static ADC_HandleTypeDef *light_hadc;

void LightSensor_Init(ADC_HandleTypeDef *hadc)
{
    light_hadc = hadc;
}

uint16_t LightSensor_ReadRaw(void)
{
    HAL_ADC_Start(light_hadc);
    HAL_ADC_PollForConversion(light_hadc, 10);
    uint16_t value = HAL_ADC_GetValue(light_hadc);
    HAL_ADC_Stop(light_hadc);

    return value;
}