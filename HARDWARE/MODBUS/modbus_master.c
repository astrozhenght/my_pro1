#include "modbus_master.h"
#include "modbus_slave.h"
#include "dma.h"
#include "rs485.h"

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
	
	RS485_Send_Data(SendBuff_485, 8);  
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
	
	RS485_Send_Data(SendBuff_485, 8);  
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
	
	RS485_Send_Data(SendBuff_485, 8);  
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
	
	RS485_Send_Data(SendBuff_485, 8);  
}

