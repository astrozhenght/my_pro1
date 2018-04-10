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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168); //延时初始化 
	
	USART_init(19200); //串口1初始化
	RS485_Init(19200);
	PID_Init();    	 //位置式PID参数初始化

	delay_ms(1000);  //等待变频器初始化完成
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);	
	delay_ms(1000);
	delay_ms(1000);	
	
	TIM4_Encode_Init();
	TIM2_Int_Init(8400, 300);    //30ms，可开关，发指令到变频器
	TIM3_Int_Init(8400, 200);    //20ms，常开，处理位置
	TIM5_Int_Init(8400, 2000);   //200ms，可开关，延时作用
		
	LED_Init();
	EXTIX_Init();	
	Modbus_RegMap();	
	DMA2_232_Config();	     
	Motor_Restore(); //上电复位回原点
	
	while(1)
	{	
		if(Receive_232_flag)  	 //232接收到触摸屏数据
		{			
			Modbus_Parse();  	//modbus协议解析
			if(Flag_Init == DONE)  //上电复位完成
				Mode_Control();	  	 //模式切换		
			memset(ReceBuff_232, 0, LEN_RECV_232);  //清空232接收到的数据
			Receive_232_flag = 0;
		}	
		FreqChg_Control(); //控制定时器，定时结束发指令到变频器
	}
}

