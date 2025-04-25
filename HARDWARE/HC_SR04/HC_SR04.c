#include "HC_SR04.h"


void HC_SR04_Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);
 
  while (Delay --);
}

float HC_SR04_Read(void)
{
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_SET);
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_RESET);
	
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_RESET)
	{
		i++;
		HC_SR04_Delayus(1);
		if(i>100000) return -1;
	}
	i = 0;
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_SET)
	{
		i = i+1;
		HC_SR04_Delayus(1);
		if(i >100000) return -2;
	}
	Distance = i*2*0.033/2;
	return Distance	;
}

