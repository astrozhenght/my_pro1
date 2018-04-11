#include "timer.h"

//定时器2初始化函数
void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  //使能TIM2时钟
	
	TIM_TimeBaseInitStruct.TIM_Period = arr-1; 		//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;  				//定时器分频
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);			 //初始化TIM2
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 			 //定时器2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    //抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   		 //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //清除更新中断请求位	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	//允许定时器2更新中断		
	TIM_Cmd(TIM2, DISABLE); //不使能定时器2
}

//定时器3初始化函数
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //使能TIM3时钟
	
	TIM_TimeBaseInitStruct.TIM_Period = arr-1; 			//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;  		//定时器分频
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	 //向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);			 //初始化TIM3
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 			//定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;   	//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除更新中断请求位	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	//允许定时器3更新中断
	TIM_Cmd(TIM3, ENABLE); 						//使能定时器3
}

//定时器5初始化函数
void TIM5_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  //使能TIM5时钟
	
	TIM_TimeBaseInitStruct.TIM_Period = arr-1; 		//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;  	//定时器分频
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);	//初始化TIM5
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; //定时器5中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   	//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); //清除更新中断请求位	
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); 	//允许定时器5更新中断	
	TIM_Cmd(TIM5, DISABLE); //失能定时器5
}
