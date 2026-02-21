#include "LED.h"
#include "main.h"
#include "tim.h"

#define LED_MAX 1000 //LED最大亮度，对应定时器的ARR值




void LED_Init(void)
{
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void LED_R_On(void){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, LED_MAX);
}
void LED_R_Off(void){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
}
void LED_R_Toggle(void){
  if (__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_3) == 0)
    LED_R_On();
  else
    LED_R_Off();
}

void LED_G_On(void){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, LED_MAX);
}
void LED_G_Off(void){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
}
void LED_G_Toggle(void){
  if (__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2) == 0)
    LED_G_On();
  else
    LED_G_Off();
}

void LED_B_On(void){
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, LED_MAX);
}
void LED_B_Off(void)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
}
void LED_B_Toggle(void){
  if (__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1) == 0)
    LED_B_On();
  else
    LED_B_Off();
}

// r,g,b的取值范围都是0-255
void LED_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, r * LED_MAX / 255);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, g * LED_MAX / 255);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, b * LED_MAX / 255);
}



