#include "dma.h"																	   	  
#include "delay.h"		 
#include "led.h"		 
#include "usart.h"		 
#include "string.h"		 
#include "rs485.h"	 

//232接收的数据量是不变的，一次20个
//232发送的数据长度是变化的

vu32 Receive_byte_num = 0;  //232接收到的字节个数
vu8  Receive_232_flag = 0;

vu32 Receive_485_num = 0;   //485接收到的字节个数
vu8  Receive_485_flag = 0;

u8 SendBuff_232[LEN_SEND_232] = {0};	//发送数据缓冲区
u8 ReceBuff_232[LEN_RECV_232] = {0};	//接收数据缓冲区

u8 SendBuff_485[LEN_SEND_485] = {0};	//发送数据缓冲区
u8 ReceBuff_485[LEN_RECV_485] = {0};	//接收数据缓冲区

//存储器地址
#define SEND_232 			(u32)SendBuff_232
#define RECEIVE_232 		(u32)ReceBuff_232
#define SEND_485 			(u32)SendBuff_485
#define RECEIVE_485 		(u32)ReceBuff_485

/**
 *作用：配置232的DMA收发
 *发送部分：
	  通道：DMA2_Stream7_Channel4
	  方向：存储器 -> 外设
	  优先级：中等优先级
	  发送模式：单次模式

 *接收部分：
		通道：DMA2_Stream5_Channel4
		方向：外设 -> 存储器
		优先级：中等优先级
		发送模式：单次模式
**/
void DMA2_232_Config(void)
{ 
	NVIC_InitTypeDef NVIC_InitStruct;
	DMA_InitTypeDef  DMA_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//DMA2时钟使能 
	
	DMA_DeInit(DMA2_Stream7);  //发送引脚	
    DMA_DeInit(DMA2_Stream5);  //接收引脚

/******************************* 发送 ******************************************/
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){} //等待DMA可配置 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  				//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;	//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = SEND_232;		//DMA 存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral; 	//存储器 -> 外设模式
	DMA_InitStruct.DMA_BufferSize = LEN_SEND_232;					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;						//普通/循环模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;		//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream7, &DMA_InitStruct);			//初始化DMA Stream
		
	/* Enable the DMA Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream7_IRQn;   // 发送DMA通道的中断配置
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;  // 优先级设置
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, ENABLE);// 开启DMA通道传输完成中断
	
/******************************* 接收 *****************************************/
	//配置DMA接收引脚，外设到存储器
	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE){} //等待DMA可配置 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  		//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART1->DR;	//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = RECEIVE_232;		//DMA 存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory; 	//外设 -> 存储器
	DMA_InitStruct.DMA_BufferSize = LEN_RECV_232;					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;			//单次/循环模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;	//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;			//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream5, &DMA_InitStruct);			//初始化DMA Stream
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  //使能串口1的DMA发送
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  //使能串口1的DMA接收
	
	DMA_Cmd(DMA2_Stream5, DISABLE); //关闭DMA后，下面这个函数才有用
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5); //清除DMA标志位
	DMA_SetCurrDataCounter(DMA2_Stream5, LEN_RECV_232); //设置下次要接收的数据字节数 
	DMA_Cmd(DMA2_Stream5, ENABLE);  //打开DMA
} 

/**
 *作用：开启一次DMA传输
**/
void DMA2_232_Send(u16 size)
{
	DMA_Cmd(DMA2_Stream7, DISABLE);                      //关闭DMA传输 	
	while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	 //确保DMA可以被设置  	
	DMA_SetCurrDataCounter(DMA2_Stream7, size);      	 //设置数据传输量   
	DMA_Cmd(DMA2_Stream7, ENABLE);                       //开启DMA传输 
}	  	

/**
 *作用：232的DMA发送完成 中断处理函数
**/
void DMA2_Stream7_IRQHandler()
{
	if(DMA_GetITStatus(DMA2_Stream7, DMA_IT_TCIF7) == SET)
	{
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
		memset(SendBuff_232, 0, LEN_SEND_232);  //清空发送完成的数据
	}
}

/**
 *作用：232的DMA接收完成 中断处理函数
 *note：数据流接收完成才会进来，接收到一个字节并不会进来
**/
void USART1_IRQHandler(void)   //串口1中断服务程序
{
	u32 temp = 0;  
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //当串口数据流停止后，就会产生空闲中断
	{			
		temp = USART1->SR; //先读SR，然后读DR才能清除 
		temp = USART1->DR; //清除DR
		temp = temp;
		DMA_Cmd(DMA2_Stream5, DISABLE); //关闭DMA
		
		DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5); //清除DMA标志位
		//232接收到的字节数
		Receive_byte_num = LEN_RECV_232 - DMA_GetCurrDataCounter(DMA2_Stream5);  //得到接收数据的个数		 
		DMA_SetCurrDataCounter(DMA2_Stream5, LEN_RECV_232);  //设置下次要接收的数据字节数  

		/**********************************************/
		//数据处理		
		Receive_232_flag = 1;
		/**********************************************/		
		
		DMA_Cmd(DMA2_Stream5, ENABLE);  //打开DMA
		
		USART_ClearFlag(USART1, USART_FLAG_IDLE); //清除空闲中断		
	} 
} 


/**
 *作用：配置485的DMA收发
 *发送部分：
		通道：DMA1_Stream6_Channel4
		方向：存储器 -> 外设
		优先级：中等优先级
		发送模式：单次模式

 *接收部分：
		通道：DMA1_Stream5_Channel4
		方向：外设 -> 存储器
		优先级：中等优先级
		发送模式：单次模式
**/
void DMA1_485_Config(void)
{ 
	NVIC_InitTypeDef NVIC_InitStruct;
	DMA_InitTypeDef  DMA_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//DMA2时钟使能 
	
	DMA_DeInit(DMA1_Stream6);  //发送引脚
    DMA_DeInit(DMA1_Stream5);  //接收引脚

/******************************* 发送 ******************************************/
	
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){} //等待DMA可配置 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  				//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART2->DR;	//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = SEND_485;		//DMA 存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral; 	//存储器 -> 外设模式
	DMA_InitStruct.DMA_BufferSize = LEN_SEND_485;					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;						//普通/循环模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;		//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA1_Stream6, &DMA_InitStruct);			//初始化DMA Stream
		
	/* Enable the DMA Interrupt */
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream6_IRQn;   // 发送DMA通道的中断配置
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x02;  // 优先级设置
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);// 开启DMA通道传输完成中断
	
/******************************* 接收 ******************************************/	

	//配置DMA接收引脚，外设到存储器
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){} //等待DMA可配置 
	DMA_InitStruct.DMA_Channel = DMA_Channel_4;  				//通道选择
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)&USART2->DR;	//DMA外设地址
	DMA_InitStruct.DMA_Memory0BaseAddr = RECEIVE_485;		//DMA 存储器0地址
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory; 	//外设 -> 存储器
	DMA_InitStruct.DMA_BufferSize = LEN_RECV_485;					//数据传输量 
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设非增量模式
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;  //存储器增量模式
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//存储器数据长度:8位
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;			//普通/循环模式 
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;	//中等优先级
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;	//存储器突发单次传输
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA1_Stream5, &DMA_InitStruct);			//初始化DMA Stream
	
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);  //使能串口2的DMA发送
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  //使能串口2的DMA接收	
	DMA_Cmd(DMA1_Stream5, DISABLE); //关闭DMA后，下面这个函数才有用
	DMA_SetCurrDataCounter(DMA1_Stream5, LEN_RECV_485); //设置下次要接收的数据字节数 
	DMA_Cmd(DMA1_Stream5, ENABLE);  //打开DMA
} 

/**
 *作用：开启一次DMA传输
**/
void DMA1_485_Send(u16 size)
{
	RS485_TX_EN = SEND;		//设置为发送模式	
	RS485_RX_EN = SEND;
	
	DMA_Cmd(DMA1_Stream6, DISABLE);                      //关闭DMA传输 	
	while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}	 //确保DMA可以被设置  	
	DMA_SetCurrDataCounter(DMA1_Stream6, size);       	 //设置数据传输量   
	DMA_Cmd(DMA1_Stream6, ENABLE);                       //开启DMA传输
}	

/**
 *作用：485的DMA发送完成 中断处理函数
**/
void DMA1_Stream6_IRQHandler()
{
	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET)
	{
		//清空数据！！
		memset(SendBuff_485, 0, LEN_SEND_485);		
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);//清除DMA2_Steam7传输完成标志
		
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET){}; //等待DMA发送完成
			
		RS485_TX_EN = RECEIVE;						//设置为接收模式
		RS485_RX_EN = RECEIVE;
	}
}

/**
 *作用：485的DMA接收完成 中断处理函数
 *note：数据流接收完成才会进来，接收到一个字节并不会进来
**/
void USART2_IRQHandler(void)   //串口1中断服务程序
{
	u32 temp = 0;  
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)  //当串口数据流停止后，就会产生空闲中断
	{			
		temp = USART2->SR; //先读SR，然后读DR才能清除 
		temp = USART2->DR; //清除DR
		temp = temp;
		DMA_Cmd(DMA1_Stream5, DISABLE); //关闭DMA
		
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5); //清除DMA标志位
		Receive_485_num = LEN_RECV_485 - DMA_GetCurrDataCounter(DMA1_Stream5);  //得到接收数据的个数		 
		DMA_SetCurrDataCounter(DMA1_Stream5, LEN_RECV_485);  //设置下次要接收的数据字节数  

		//数据处理	
		Receive_485_flag = 1; //485接收完成标志位置1
		LED_Toggle(LED2);
		
		DMA_Cmd(DMA1_Stream5, ENABLE);  //打开DMA
		USART_ClearFlag(USART2, USART_FLAG_IDLE); //清除空闲中断		
	} 
} 


