#include "modbus_master.h"
#include "modbus_slave.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "timer.h"
#include "led.h"
#include "rs485.h"
#include "dma.h"
#include "string.h"
#include "control.h"
#include "encoder.h"
#include "exti.h"
#include "pid.h"

int main(void)
{ 	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168); //��ʱ��ʼ�� 
	
	USART_init(19200); //����1��ʼ��
	RS485_Init(19200);
	PID_Init();    	 //λ��ʽPID������ʼ��

	delay_ms(1000);  //�ȴ���Ƶ����ʼ�����
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);	
	delay_ms(1000);
	delay_ms(1000);	
	
	TIM4_Encode_Init();
	TIM2_Int_Init(8400, 300);    //30ms���ɿ��أ���ָ���Ƶ��
	TIM3_Int_Init(8400, 200);    //20ms������������λ��
	TIM5_Int_Init(8400, 2000);   //200ms���ɿ��أ���ʱ����
		
	LED_Init();
	EXTIX_Init();	
	Modbus_RegMap();	
	DMA2_232_Config();	     
	Motor_Restore(); //�ϵ縴λ��ԭ��
	
	while(1)
	{	
		if(Receive_232_flag)  	 //232���յ�����������
		{			
			Modbus_Parse();  	//modbusЭ�����
			if(Flag_Init == DONE)  //�ϵ縴λ���
				Mode_Control();	  	 //ģʽ�л�		
			memset(ReceBuff_232, 0, LEN_RECV_232);  //���232���յ�������
			Receive_232_flag = 0;
		}	
		FreqChg_Control(); //���ƶ�ʱ������ʱ������ָ���Ƶ��
	}
}

