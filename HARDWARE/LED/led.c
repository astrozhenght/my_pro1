#include "led.h"
#include "sys.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //ʹ��GPIOAʱ��

	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_OUT;	 //���ģʽ
	GPIO_InitTypeStruct.GPIO_OType = GPIO_OType_PP;  //�������
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitTypeStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOH, &GPIO_InitTypeStruct);
	
}

//LED��ת
void LED_Toggle(u16 LEDx)
{
	if(LEDx & GPIO_Pin_2)	  GPIOH->ODR ^= GPIO_Pin_2;
	if(LEDx & GPIO_Pin_3)	  GPIOH->ODR ^= GPIO_Pin_3;
	if(LEDx & GPIO_Pin_4)	  GPIOH->ODR ^= GPIO_Pin_4;
	if(LEDx & GPIO_Pin_5)	  GPIOH->ODR ^= GPIO_Pin_5;
}



