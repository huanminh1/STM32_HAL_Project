#ifndef __HCSR05_H
#define __HCSR05_H

#include "stm32f1xx_hal.h"

void HCSR05_INIT(GPIO_TypeDef *trig_port, uint16_t trig_pin, GPIO_TypeDef *echo_port, uint16_t echo_pin);
void HCSR05_Trigger();
uint16_t HCSR05_GetDis();
#endif