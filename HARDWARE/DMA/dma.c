#include "dma.h"																	   	  
#include "delay.h"		 
#include "led.h"		 
#include "usart.h"		 
#include "string.h"		 
#include "rs485.h"	 

//232���յ��������ǲ���ģ�һ��20��
//232���͵����ݳ����Ǳ仯��

vu32 Receive_byte_num = 0;  //232���յ����ֽڸ���
vu8  Receive_232_flag = 0;

vu32 Receive_485_num = 0;   //485���յ����ֽڸ���
vu8  Receive_485_flag = 0;

u8 SendBuff_232[LEN_SEND_232] = {0};	//�������ݻ�����
u8 ReceBuff_232[LEN_RECV_232] = {0};	//�������ݻ�����

u8 SendBuff_485[LEN_SEND_485] = {0};	//�������ݻ�����
u8 ReceBuff_485[LEN_RECV_485] = {0};	//�������ݻ�����

//�洢����ַ
#define SEND_232 			(u32)SendBuff_232
#define RECEIVE_232 		(u32)ReceBuff_232
#define SEND_485 			(u32)SendBuff_485
#define RECEIVE_485 		(u32)ReceBuff_485

/**
 *���ã�����232��DMA�շ�
 *���Ͳ��֣�
	  ͨ����DMA2_Stream7_Channel4
	  ���򣺴洢�� -> ����
	  ���ȼ����е����ȼ�
	  ����ģʽ������ģʽ

 *���ղ��֣�
		ͨ����DMA2_Stream5_Channel4
		�������� -> �洢��
		���ȼ����е����ȼ�
		����ģʽ������ģʽ
**/
void DMA2_232_Config(void)
{ 
	NVIC_InitTypeDef NVIC_InitStruct;
	DMA_InitTypeDef  DMA_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//DMA2ʱ��ʹ�� 
	
	DMA_DeInit(DMA2_Stream7);  //��������	
    DMA_DeInit(DMA2_Stream5);  //��������

/******************************* ���� ******************************************/
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){} //�ȴ�DMA������ 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  				//ͨ��ѡ��
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;	//DMA�����ַ
	DMA_InitStruct.DMA_Memory0BaseAddr = SEND_232;		//DMA �洢��0��ַ
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral; 	//�洢�� -> ����ģʽ
	DMA_InitStruct.DMA_BufferSize = LEN_SEND_232;					//���ݴ����� 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //���������ģʽ
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�洢������ģʽ
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//�洢�����ݳ���:8λ
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;						//��ͨ/ѭ��ģʽ 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;		//�е����ȼ�
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA2_Stream7, &DMA_InitStruct);			//��ʼ��DMA Stream
		
	/* Enable the DMA Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;   // ����DMAͨ�����ж�����
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;  // ���ȼ�����
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);// ����DMAͨ����������ж�
	
/******************************* ���� *****************************************/
	//����DMA�������ţ����赽�洢��
	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){} //�ȴ�DMA������ 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  		//ͨ��ѡ��
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;	//DMA�����ַ
	DMA_InitStruct.DMA_Memory0BaseAddr = RECEIVE_232;		//DMA �洢��0��ַ
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory; 	//���� -> �洢��
	DMA_InitStruct.DMA_BufferSize = LEN_RECV_232;					//���ݴ����� 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //���������ģʽ
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�洢������ģʽ
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//�洢�����ݳ���:8λ
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;			//����/ѭ��ģʽ 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;	//�е����ȼ�
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;			//�洢��ͻ�����δ���
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);			//��ʼ��DMA Stream
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  //ʹ�ܴ���1��DMA����
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  //ʹ�ܴ���1��DMA����
	
	DMA_Cmd(DMA2_Stream5, DISABLE); //�ر�DMA�������������������
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5); //���DMA��־λ
	DMA_SetCurrDataCounter(DMA2_Stream5, LEN_RECV_232); //�����´�Ҫ���յ������ֽ��� 
	DMA_Cmd(DMA2_Stream5, ENABLE);  //��DMA
} 

/**
 *���ã�����һ��DMA����
**/
void DMA2_232_Send(u16 size)
{
	DMA_Cmd(DMA2_Stream7, DISABLE);                      //�ر�DMA���� 	
	while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	 //ȷ��DMA���Ա�����  	
	DMA_SetCurrDataCounter(DMA2_Stream7, size);      	 //�������ݴ�����   
	DMA_Cmd(DMA2_Stream7, ENABLE);                       //����DMA���� 
}	  	

/**
 *���ã�232��DMA������� �жϴ�����
**/
void DMA2_Stream7_IRQHandler()
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) == SET)
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
		memset(SendBuff_232, 0, LEN_SEND_232);  //��շ�����ɵ�����
	}
}

/**
 *���ã�232��DMA������� �жϴ�����
 *note��������������ɲŻ���������յ�һ���ֽڲ��������
**/
void USART1_IRQHandler(void)   //����1�жϷ������
{
	u32 temp = 0;  
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //������������ֹͣ�󣬾ͻ���������ж�
	{			
		temp = USART1->SR; //�ȶ�SR��Ȼ���DR������� 
		temp = USART1->DR; //���DR
		temp = temp;
		DMA_Cmd(DMA2_Stream5, DISABLE); //�ر�DMA
		
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5); //���DMA��־λ
		//232���յ����ֽ���
		Receive_byte_num = LEN_RECV_232 - DMA_GetCurrDataCounter(DMA2_Stream5);  //�õ��������ݵĸ���		 
		DMA_SetCurrDataCounter(DMA2_Stream5, LEN_RECV_232);  //�����´�Ҫ���յ������ֽ���  

		/**********************************************/
		//���ݴ���		
		Receive_232_flag = 1;
		/**********************************************/		
		
		DMA_Cmd(DMA2_Stream5, ENABLE);  //��DMA
		
		USART_ClearFlag(USART1, USART_FLAG_IDLE); //��������ж�		
	} 
} 


/**
 *���ã�����485��DMA�շ�
 *���Ͳ��֣�
		ͨ����DMA1_Stream6_Channel4
		���򣺴洢�� -> ����
		���ȼ����е����ȼ�
		����ģʽ������ģʽ

 *���ղ��֣�
		ͨ����DMA1_Stream5_Channel4
		�������� -> �洢��
		���ȼ����е����ȼ�
		����ģʽ������ģʽ
**/
void DMA1_485_Config(void)
{ 
	NVIC_InitTypeDef NVIC_InitStruct;
	DMA_InitTypeDef  DMA_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//DMA2ʱ��ʹ�� 
	
	DMA_DeInit(DMA1_Stream6);  //��������
    DMA_DeInit(DMA1_Stream5);  //��������

/******************************* ���� ******************************************/
	
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){} //�ȴ�DMA������ 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  				//ͨ��ѡ��
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART2->DR;	//DMA�����ַ
	DMA_InitStruct.DMA_Memory0BaseAddr = SEND_485;		//DMA �洢��0��ַ
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral; 	//�洢�� -> ����ģʽ
	DMA_InitStruct.DMA_BufferSize = LEN_SEND_485;					//���ݴ����� 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //���������ģʽ
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�洢������ģʽ
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//�洢�����ݳ���:8λ
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;						//��ͨ/ѭ��ģʽ 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;		//�е����ȼ�
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);			//��ʼ��DMA Stream
		
	/* Enable the DMA Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream6_IRQn;   // ����DMAͨ�����ж�����
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;  // ���ȼ�����
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);// ����DMAͨ����������ж�
	
/******************************* ���� ******************************************/	

	//����DMA�������ţ����赽�洢��
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){} //�ȴ�DMA������ 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  				//ͨ��ѡ��
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART2->DR;	//DMA�����ַ
	DMA_InitStruct.DMA_Memory0BaseAddr = RECEIVE_485;		//DMA �洢��0��ַ
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory; 	//���� -> �洢��
	DMA_InitStruct.DMA_BufferSize = LEN_RECV_485;					//���ݴ����� 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //���������ģʽ
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�洢������ģʽ
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//�洢�����ݳ���:8λ
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;			//��ͨ/ѭ��ģʽ 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;	//�е����ȼ�
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;	//�洢��ͻ�����δ���
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream5, &DMA_InitStruct);			//��ʼ��DMA Stream
	
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  //ʹ�ܴ���2��DMA����
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  //ʹ�ܴ���2��DMA����	
	DMA_Cmd(DMA1_Stream5, DISABLE); //�ر�DMA�������������������
	DMA_SetCurrDataCounter(DMA1_Stream5, LEN_RECV_485); //�����´�Ҫ���յ������ֽ��� 
	DMA_Cmd(DMA1_Stream5, ENABLE);  //��DMA
} 

/**
 *���ã�����һ��DMA����
**/
void DMA1_485_Send(u16 size)
{
	RS485_TX_EN = SEND;		//����Ϊ����ģʽ	
	RS485_RX_EN = SEND;
	
	DMA_Cmd(DMA1_Stream6, DISABLE);                      //�ر�DMA���� 	
	while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}	 //ȷ��DMA���Ա�����  	
	DMA_SetCurrDataCounter(DMA1_Stream6, size);       	 //�������ݴ�����   
	DMA_Cmd(DMA1_Stream6, ENABLE);                       //����DMA����
}	

/**
 *���ã�485��DMA������� �жϴ�����
**/
void DMA1_Stream6_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET)
	{
		//������ݣ���
		memset(SendBuff_485, 0, LEN_SEND_485);		
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);//���DMA2_Steam7������ɱ�־
		
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}; //�ȴ�DMA�������
			
		RS485_TX_EN = RECEIVE;						//����Ϊ����ģʽ
		RS485_RX_EN = RECEIVE;
	}
}

/**
 *���ã�485��DMA������� �жϴ�����
 *note��������������ɲŻ���������յ�һ���ֽڲ��������
**/
void USART2_IRQHandler(void)   //����1�жϷ������
{
	u32 temp = 0;  
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  //������������ֹͣ�󣬾ͻ���������ж�
	{			
		temp = USART2->SR; //�ȶ�SR��Ȼ���DR������� 
		temp = USART2->DR; //���DR
		temp = temp;
		DMA_Cmd(DMA1_Stream5, DISABLE); //�ر�DMA
		
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5); //���DMA��־λ
		Receive_485_num = LEN_RECV_485 - DMA_GetCurrDataCounter(DMA1_Stream5);  //�õ��������ݵĸ���		 
		DMA_SetCurrDataCounter(DMA1_Stream5, LEN_RECV_485);  //�����´�Ҫ���յ������ֽ���  

		//���ݴ���	
		Receive_485_flag = 1; //485������ɱ�־λ��1
		LED_Toggle(LED2);
		
		DMA_Cmd(DMA1_Stream5, ENABLE);  //��DMA
		USART_ClearFlag(USART2, USART_FLAG_IDLE); //��������ж�		
	} 
} 


