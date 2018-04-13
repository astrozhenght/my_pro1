#include "modbus_master.h"
#include "modbus_slave.h"
#include "dma.h"
#include "rs485.h"
#include "string.h"
#include "control.h"

#define RS485_ADDR  0x01    //主机地址
#define REG_MAX	    0x4000   

u16     REG_Start_Addr2;  	//寄存器起始地址变量，两个字节组成

/**
 *作用：485发送指令，改变变频器的频率
 *参数：浮点数指定变频器频率
**/
void FREQ_Change_Freq(float val)
{
	u16 freq; 
	u16 crc;
	freq = (u16)(val*100);
	SendBuff_485[0] = 0x01;
	SendBuff_485[1] = 0x06;
	SendBuff_485[2] = 0x20;
	SendBuff_485[3] = 0x01;
	SendBuff_485[4] = (u8)(freq>>8);   
	SendBuff_485[5] = (u8)(freq&0xFF); 
	
	//计算CRC校验码
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC的低字节
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC的高字节
//	RS485_Send_Data(SendBuff_485, 8);	
	DMA1_485_Send(8);
}

/**
 *作用：485发送指令，控制电机右转
**/
void Motor_Right(void)
{
	u16 crc;
	SendBuff_485[0] = 0x01;
	SendBuff_485[1] = 0x06;
	SendBuff_485[2] = 0x20;
	SendBuff_485[3] = 0x00;
	SendBuff_485[4] = 0x00;   
	SendBuff_485[5] = 0x02; 
	
	//计算CRC校验码
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC的低字节
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC的高字节	
//	RS485_Send_Data(SendBuff_485, 8);	
	DMA1_485_Send(8);
}

/**
 *作用：485发送指令，控制电机左转
**/
void Motor_Left(void)
{
	u16 crc;
	SendBuff_485[0] = 0x01;
	SendBuff_485[1] = 0x06;
	SendBuff_485[2] = 0x20;
	SendBuff_485[3] = 0x00;
	SendBuff_485[4] = 0x00;   
	SendBuff_485[5] = 0x03; 
	
	//计算CRC校验码
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC的低字节
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC的高字节	
	DMA1_485_Send(8);
//	RS485_Send_Data(SendBuff_485, 8);	
}

/**
 *作用：485发送指令，控制电机停止
**/
void Motor_Stop(void)
{
	u16 crc;
	SendBuff_485[0] = 0x01;
	SendBuff_485[1] = 0x06;
	SendBuff_485[2] = 0x20;
	SendBuff_485[3] = 0x00;
	SendBuff_485[4] = 0x00;   
	SendBuff_485[5] = 0x07; 
	
	//计算CRC校验码
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC的低字节
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC的高字节	
	DMA1_485_Send(8);
//	RS485_Send_Data(SendBuff_485, 8);	
}


/**  
 *作用：PLC与变频器传输数据使用了Modbus协议，PLC作为主机会回复触摸屏
	   的查询信息，下面就是Modbus协议数据的解析过程。 
**/ 
void Modbus2_Parse(void)
{
	u16 crc, recv_crc;
	if(ReceBuff_485[0] == RS485_ADDR)  //地址正确
	{
		if(ReceBuff_485[1]==0x03 || ReceBuff_485[1]==0x06 || ReceBuff_485[1]==0x08) //功能码正确
		{			
			REG_Start_Addr2 = (((u16)ReceBuff_485[2])<<8)|ReceBuff_485[3];  //获得寄存器起始地址
			if(REG_Start_Addr2 < REG_MAX) 	//寄存器地址在范围内
			{
				crc = CRC16(ReceBuff_485, Receive_485_num - 2);  //计算接收数据的CRC
				recv_crc = ((u16)(ReceBuff_485[Receive_485_num-1])<<8) |  \
								  ReceBuff_485[Receive_485_num-2];  //整合接收到的CRC
				if(crc == recv_crc)  //CRC校验正确
				{
					switch(ReceBuff_485[1])
					{
						case 0x03:
							Modbus2_03_Solve();
							break;
						case 0x06:
							Modbus2_06_Solve();
							break;
						case 0x08:
							Modbus2_08_Solve();
							break;
					}					
				}
				else  //非法数据值
				{
					Data_Error = 3;   //非法数据值
					Status_Alarm = 0; //通信报错
				}
			}
			else  //非法数据地址
			{
				Data_Error = 2;   //非法数据地址
				Status_Alarm = 0; //通信报错
			}		
		}
		else if(ReceBuff_485[1]==0x83 || ReceBuff_485[1]==0x86 || ReceBuff_485[1]==0x88)
		{
			Data_Error = ReceBuff_485[2];   //取出错误码
			Status_Alarm = 0; //通信报错
		}
		else
		{
			Data_Error = 1; //功能码错误
			Status_Alarm = 0; //通信报错
		}	
	}
}

/**  
 *作用：功能码指令03的数据处理过程
 *0x03：读取多个 PSW/PFW 寄存器数据内容 
**/ 
void Modbus2_03_Solve(void)
{
	u8 i;
	u16 reg_num;
	reg_num = ((u16)ReceBuff_485[2]) % 2;   //获取寄存器数量
	if(REG_Start_Addr2 + reg_num < REG_MAX)  //寄存器地址+数量在范围内
	{
		Status_Alarm = 1; //通信正常		
//		for(i = 0; i < reg_num; i++)
//		{
//			(*Modbus_HoldReg[REG_Start_Addr+i]>>8)&0xFF = ReceBuff_485[3+i*2];	//先发送高字节
//			*Modbus_HoldReg[REG_Start_Addr+i]&0xFF = ReceBuff_485[4+i*2];  		//后发送低字节			
//		}
	}
	else  //非法数据地址
	{
		Data_Error = 2; //非法数据地址
		Status_Alarm = 0; //通信报错
	}
}

/**  
 *作用：功能码指令06的数据处理过程
**/ 
void Modbus2_06_Solve(void)
{
	int i;	
	i = strcmp((const char *)SendBuff_485, (const char *)ReceBuff_485);
	if(i == 0) //PLC发送和变频器回复一致
	{
		if(ReceBuff_485[3] == 0x01)
			Last_Speed = Data_RunSpeed;
		else if(ReceBuff_485[3] == 0x00)
			Last_Dir = Motor_Dir; //不会重发，通信完成
		Status_Alarm = 1; //通信正常
	}
	else  //非法数据值
	{
		Data_Error = 3;   //非法数据值
		Status_Alarm = 0; //通信报错
	}
}

/**  
 *作用：功能码指令08的数据处理过程
**/ 
void Modbus2_08_Solve(void)
{
	int i;	
	i = strcmp((const char *)SendBuff_485, (const char *)ReceBuff_485);
	if(i == 0) //PLC发送和变频器回复一致
	{
		Status_Alarm = 1; //通信正常
	}
	else 
	{
		Data_Error = 3;   //非法数据值
		Status_Alarm = 0; //通信报错
	}
}


