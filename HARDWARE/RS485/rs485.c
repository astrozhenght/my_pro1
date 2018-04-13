#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "string.h"
#include "usart.h"
#include "led.h"

u8 	RS485_RX_BUF[RS485_REC_LEN];  	//接收缓冲,最大30个字节.
u16 RS485_RX_STA = 0; 							//接收到的数据状态
//bit7，	接收完成标志
//bit6，	接收到0x0d
//bit5~0，接收到的有效字节数目，最大值31

//初始化串口2
//bound:波特率	 
//8位字长，1停止，无校验
void RS485_Init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOH,ENABLE); //使能GPIOA,GPIOH时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //使能USART2时钟
	
	//串口2引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	//初始化PA2，PA3
	
	//PH13推挽输出，485模式控制  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14; 	//GPIOH13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
	GPIO_Init(GPIOH, &GPIO_InitStructure); //初始化PH13
	
    //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;		//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;		//无校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;									//收发模式
    USART_Init(USART2, &USART_InitStructure); 																			//初始化串口2
	
    USART_Cmd(USART2, ENABLE);  //使能串口 2	
	USART_ClearFlag(USART2, USART_FLAG_TC);	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启空闲中断

	//Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
	
	RS485_TX_EN = RECEIVE;	//默认为接收模式	
	RS485_RX_EN = RECEIVE;	//默认为接收模式	
}


/**
 *作用：RS485发送字符数组
 *形参：buf发送区首地址
**/
void RS485_Send_Data(u8 *buf, u8 len)
{
	u8 t;  											//len是字符串长度
	RS485_TX_EN = SEND;								//设置为发送模式
	RS485_RX_EN = SEND;
	
	for(t = 0; t < len; t++)					//循环发送数据
	{
	  while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET){}; //等待发送结束		
    USART_SendData(USART2, buf[t]); 
	}	 
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 			//等待发送结束
	
	RS485_TX_EN = RECEIVE;						//设置为接收模式
	RS485_RX_EN = RECEIVE;
}


