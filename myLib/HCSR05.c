#include "HCSR05.h"
#include "BSP_Delay.h"

//use TIM2
extern TIM_HandleTypeDef htim2;

static GPIO_TypeDef *TRIG_Port;
static uint16_t TRIG_Pin;

static GPIO_TypeDef *ECHO_Port;
static uint16_t ECHO_Pin;

void HCSR05_INIT(GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin)
{
	HAL_TIM_Base_Start(&htim2);
	TRIG_Port = trig_port;
	TRIG_Pin = trig_pin;
	ECHO_Port = echo_port;
	ECHO_Pin = echo_pin;
}

void HCSR05_Trigger()
{
	HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, 1);
	BSP_Delay_us(20);				// >10us
	HAL_GPIO_WritePin(TRIG_Port, TRIG_Pin, 0);
}
uint16_t HCSR05_GetDis()
{
	uint16_t time;
	uint32_t timeout;
	uint16_t dis;
	HCSR05_Trigger();
	
	//Wait for ECHO HIGH => measure
	timeout = HAL_GetTick();
	while(HAL_GPIO_ReadPin(ECHO_Port,ECHO_Pin) == 0)
	{
			if(HAL_GetTick() - timeout > 50)
					return -1;  
	}
	uint16_t start = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2); // 1 counter = 1 us
	
	//Wait for ECHO LOW => get time
	while(HAL_GPIO_ReadPin(ECHO_Port,ECHO_Pin) == 1)
	{
			if(HAL_GetTick() - timeout > 50)
					return -1; 
	}
	time = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2) - start;
	dis =  time * (34000/2) / 1000000 ;
	return dis;
}
