#include "sys.h"		    
#include "rs485.h"	 
#include "delay.h"
#include "string.h"
#include "usart.h"
#include "led.h"

u8 	RS485_RX_BUF[RS485_REC_LEN];  	//���ջ���,���30���ֽ�.
u16 RS485_RX_STA = 0; 							//���յ�������״̬
//bit7��	������ɱ�־
//bit6��	���յ�0x0d
//bit5~0�����յ�����Ч�ֽ���Ŀ�����ֵ31

//��ʼ������2
//bound:������	 
//8λ�ֳ���1ֹͣ����У��
void RS485_Init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOH,ENABLE); //ʹ��GPIOA,GPIOHʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  //ʹ��USART2ʱ��
	
	//����2���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	//USART2    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;  //GPIOA2��GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	//��ʼ��PA2��PA3
	
	//PH13���������485ģʽ����  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14; 	//GPIOH13
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	//���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //����
	GPIO_Init(GPIOH, &GPIO_InitStructure); //��ʼ��PH13
	
    //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;		//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;		//��У��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;									//�շ�ģʽ
    USART_Init(USART2, &USART_InitStructure); 																			//��ʼ������2
	
    USART_Cmd(USART2, ENABLE);  //ʹ�ܴ��� 2	
	USART_ClearFlag(USART2, USART_FLAG_TC);	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//���������ж�

	//Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
	
	RS485_TX_EN = RECEIVE;	//Ĭ��Ϊ����ģʽ	
	RS485_RX_EN = RECEIVE;	//Ĭ��Ϊ����ģʽ	
}


/**
 *���ã�RS485�����ַ�����
 *�βΣ�buf�������׵�ַ
**/
void RS485_Send_Data(u8 *buf, u8 len)
{
	u8 t;  											//len���ַ�������
	RS485_TX_EN = SEND;								//����Ϊ����ģʽ
	RS485_RX_EN = SEND;
	
	for(t = 0; t < len; t++)					//ѭ����������
	{
	  while(USART_GetFlagStatus(USART2, USART_FLAG_TC)==RESET){}; //�ȴ����ͽ���		
    USART_SendData(USART2, buf[t]); 
	}	 
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET); 			//�ȴ����ͽ���
	
	RS485_TX_EN = RECEIVE;						//����Ϊ����ģʽ
	RS485_RX_EN = RECEIVE;
}


