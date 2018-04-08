#include "modbus_master.h"
#include "modbus_slave.h"
#include "dma.h"
#include "rs485.h"

/**
 *���ã�485����ָ��ı��Ƶ����Ƶ��
 *������������ָ����Ƶ��Ƶ��
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
	
	//����CRCУ����
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC�ĵ��ֽ�
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC�ĸ��ֽ�
	
	RS485_Send_Data(SendBuff_485, 8);  
}

/**
 *���ã�485����ָ����Ƶ����ת
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
	
	//����CRCУ����
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC�ĵ��ֽ�
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC�ĸ��ֽ�
	
	RS485_Send_Data(SendBuff_485, 8);  
}

/**
 *���ã�485����ָ����Ƶ����ת
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
	
	//����CRCУ����
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC�ĵ��ֽ�
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC�ĸ��ֽ�
	
	RS485_Send_Data(SendBuff_485, 8);  
}

/**
 *���ã�485����ָ����Ƶ��ֹͣ
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
	
	//����CRCУ����
	crc = CRC16(SendBuff_485, 6);
	SendBuff_485[6] = (u8)(crc&0xFF);   //CRC�ĵ��ֽ�
	SendBuff_485[7] = (u8)(crc>>8);   	//CRC�ĸ��ֽ�
	
	RS485_Send_Data(SendBuff_485, 8);  
}

