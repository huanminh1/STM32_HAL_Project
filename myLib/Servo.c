#include "Servo.h"

//use TIM2
static TIM_HandleTypeDef *servo_tim;
static uint32_t servo_channel;
void Servo_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    servo_tim = htim;
		servo_channel = channel;
    HAL_TIM_PWM_Start(servo_tim, servo_channel);
}

void Servo_SetAngle(uint8_t angle)
{
    if(angle > 180) angle = 180;

    // Map 0-180° -> 500-2500 (us)
    uint16_t ccr = (angle * 2000) / 180 + 500;

    __HAL_TIM_SET_COMPARE(servo_tim, servo_channel, ccr);
}