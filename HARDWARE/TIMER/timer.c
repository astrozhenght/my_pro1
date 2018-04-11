#include "timer.h"

//��ʱ��2��ʼ������
void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  //ʹ��TIM2ʱ��
	
	TIM_TimeBaseInitStruct.TIM_Period = arr-1; 		//�Զ���װ��ֵ
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;  				//��ʱ����Ƶ
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;//���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);			 //��ʼ��TIM2
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 			 //��ʱ��2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;    //��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   		 //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update); //��������ж�����λ	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 	//����ʱ��2�����ж�		
	TIM_Cmd(TIM2, DISABLE); //��ʹ�ܶ�ʱ��2
}

//��ʱ��3��ʼ������
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStruct.TIM_Period = arr-1; 			//�Զ���װ��ֵ
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;  		//��ʱ����Ƶ
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	 //���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);			 //��ʼ��TIM3
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 			//��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;   	//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //��������ж�����λ	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 	//����ʱ��3�����ж�
	TIM_Cmd(TIM3, ENABLE); 						//ʹ�ܶ�ʱ��3
}

//��ʱ��5��ʼ������
void TIM5_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  //ʹ��TIM5ʱ��
	
	TIM_TimeBaseInitStruct.TIM_Period = arr-1; 		//�Զ���װ��ֵ
	TIM_TimeBaseInitStruct.TIM_Prescaler = psc-1;  	//��ʱ����Ƶ
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1; 	
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);	//��ʼ��TIM5
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn; //��ʱ��5�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   	//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); //��������ж�����λ	
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); 	//����ʱ��5�����ж�	
	TIM_Cmd(TIM5, DISABLE); //ʧ�ܶ�ʱ��5
}
