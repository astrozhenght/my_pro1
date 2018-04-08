#include "stm32f4xx.h"
#include "usart.h"

u16 Parameter = 2048;   		/*编码器的精度*/
u16 Increment = 0;          	/*读取计数器的值*/
 
void TIM4_Encode_Init(void)
{
	GPIO_InitTypeDef GPIO_InitTypeStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_ICInitTypeDef TIM_ICInitTypeStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); /*开启复用时钟!!!*/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);
	
	GPIO_InitTypeStruct.GPIO_Mode = GPIO_Mode_AF;  		   /*复用功能模式 */
	GPIO_InitTypeStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitTypeStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitTypeStruct.GPIO_OType = GPIO_OType_OD; 
	GPIO_InitTypeStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitTypeStruct);
	
	TIM_DeInit(TIM4);
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period = Parameter - 1;
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	
	TIM_ICStructInit(&TIM_ICInitTypeStruct);	
	TIM_ICInitTypeStruct.TIM_ICFilter = 6;
	TIM_ICInit(TIM4, &TIM_ICInitTypeStruct);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);  //清除所有标志位
	
	TIM4->CNT = 0;	
	TIM_Cmd(TIM4, ENABLE);
}


