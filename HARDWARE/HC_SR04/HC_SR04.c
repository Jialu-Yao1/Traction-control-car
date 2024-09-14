#include "HC_SR04.h"

/*******************
*  @brief  us delay
*  @param  target us delay
*  @return  
*
*******************/
void HC_SR04_Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);//SystemCoreClock
  do
  {
    __NOP();
  }
  while (Delay --);
}
/*******************
*  @brief  HC_SR04 read distance
*  @param  void
*  @return distance 
*
*******************/
float HC_SR04_Read(void)
{
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_SET);//output 15us high level
	HC_SR04_Delayus(15);
	HAL_GPIO_WritePin(HC_SR04_Trig_GPIO_Port,HC_SR04_Trig_Pin,GPIO_PIN_RESET);
	
	while(HAL_GPIO_ReadPin(HC_SR04_Echo_GPIO_Port,HC_SR04_Echo_Pin) == GPIO_PIN_RESET)//wait 
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

