#include "DHT11.h"
#include "bsp_delay.h"

static GPIO_TypeDef *DHT_Port;
static uint16_t DHT_Pin;

/* ================= GPIO MODE ================= */

static void DHT_SetOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DHT_Port, &GPIO_InitStruct);
}

static void DHT_SetInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = DHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT_Port, &GPIO_InitStruct);
}

/* ================= INIT ================= */

void DHT11_Init(GPIO_TypeDef *port, uint16_t pin)
{
	DHT_Port = port;
	DHT_Pin = pin;
}

/* ================= START SIGNAL ================= */

static void DHT11_Start(void)
{
	uint32_t timeout;
	
	DHT_SetOutput();

	HAL_GPIO_WritePin(DHT_Port, DHT_Pin, 1);
	HAL_GPIO_WritePin(DHT_Port, DHT_Pin, 0);
	BSP_Delay_ms(20);			// >18ms
	HAL_GPIO_WritePin(DHT_Port, DHT_Pin, 1);
	BSP_Delay_us(20); 		// waits 20-40us
	

	//Wait DHT response
	DHT_SetInput();
	
 // Wait DHT pulls LOW
	timeout = HAL_GetTick();
	while(HAL_GPIO_ReadPin(DHT_Port, DHT_Pin) == 1)
	{
			if(HAL_GetTick() - timeout > 2)
					return;   // timeout
	}

	// Wait DHT pulls HIGH
	timeout = HAL_GetTick();
	while(HAL_GPIO_ReadPin(DHT_Port, DHT_Pin) == 0)
	{
			if(HAL_GetTick() - timeout > 2)
					return;
	}

	// Wait DHT pulls LOW again
	timeout = HAL_GetTick();
	while(HAL_GPIO_ReadPin(DHT_Port, DHT_Pin) == 1)
	{
			if(HAL_GetTick() - timeout > 2)
					return;
	}
	
	//DHT starts transmitting data (40 bits)
}

/* ================= READ BYTE ================= */

static uint8_t DHT11_ReadByte(void)
{
	
	uint8_t i, result = 0;
	uint32_t timeout;
	for(i = 0; i < 8; i++)
	{
		timeout = HAL_GetTick();
		while(HAL_GPIO_ReadPin(DHT_Port, DHT_Pin) == 0)
		{
				if(HAL_GetTick() - timeout > 2)
						return 0;
		}

		BSP_Delay_us(50);
		
		if(HAL_GPIO_ReadPin(DHT_Port, DHT_Pin) == 1)
		{ 
			// bit 1
			result = (result << 1) | (1 << 0);
		}
		else
		{	
			// bit 0
			result = (result << 1) & ~(1 << 0);
		}
		
		timeout = HAL_GetTick();
		while(HAL_GPIO_ReadPin(DHT_Port, DHT_Pin) == 1)
		{
				if(HAL_GetTick() - timeout > 2)
						return 0;
		}
	}
	
	return result;
}

/* ================= READ DATA ================= */

uint8_t DHT11_Read(DHT11_Data_t *data)
{
    uint8_t humi_int, humi_dec;
    uint8_t temp_int, temp_dec;
    uint8_t checksum;

    DHT11_Start();

    humi_int = DHT11_ReadByte();
    humi_dec = DHT11_ReadByte();
    temp_int = DHT11_ReadByte();
    temp_dec = DHT11_ReadByte();
    checksum = DHT11_ReadByte();

		if(checksum == (humi_int + humi_dec + temp_int + temp_dec))
		{
				data->humidity = humi_int;
				data->temperature = temp_int;
				return 1;  // success
		}

		return 0;  // checksum error
}
