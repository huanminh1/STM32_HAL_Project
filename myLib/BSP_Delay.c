#include "BSP_Delay.h"

extern TIM_HandleTypeDef htim2;

void BSP_Delay_Init(void)
{
    HAL_TIM_Base_Start(&htim2);
}

void BSP_Delay_us(uint16_t us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim2);

    while((uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) - start) < us);
}

void BSP_Delay_ms(uint32_t ms)
{
#ifdef USE_FREERTOS
    vTaskDelay(pdMS_TO_TICKS(ms));
#else
    uint32_t start = HAL_GetTick();
    while((HAL_GetTick() - start) < ms);
#endif
}