#include "modbus_master.h"
#include "modbus_slave.h"
#include "dma.h"
#include "rs485.h"
#include "string.h"
#include "control.h"

#define RS485_ADDR  0x01    //������ַ
#define REG_MAX	    0x4000   

u16     REG_Start_Addr2;  	//�Ĵ�����ʼ��ַ�����������ֽ����

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
//	RS485_Send_Data(SendBuff_485, 8);	
	DMA1_485_Send(8);
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
//	RS485_Send_Data(SendBuff_485, 8);	
	DMA1_485_Send(8);
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
	DMA1_485_Send(8);
//	RS485_Send_Data(SendBuff_485, 8);	
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
	DMA1_485_Send(8);
//	RS485_Send_Data(SendBuff_485, 8);	
}


/**  
 *���ã�PLC���Ƶ����������ʹ����ModbusЭ�飬PLC��Ϊ������ظ�������
	   �Ĳ�ѯ��Ϣ���������ModbusЭ�����ݵĽ������̡� 
**/ 
void Modbus2_Parse(void)
{
	u16 crc, recv_crc;
	if(ReceBuff_485[0] == RS485_ADDR)  //��ַ��ȷ
	{
		if(ReceBuff_485[1]==0x03 || ReceBuff_485[1]==0x06 || ReceBuff_485[1]==0x08) //��������ȷ
		{			
			REG_Start_Addr2 = (((u16)ReceBuff_485[2])<<8)|ReceBuff_485[3];  //��üĴ�����ʼ��ַ
			if(REG_Start_Addr2 < REG_MAX) 	//�Ĵ�����ַ�ڷ�Χ��
			{
				crc = CRC16(ReceBuff_485, Receive_485_num - 2);  //����������ݵ�CRC
				recv_crc = ((u16)(ReceBuff_485[Receive_485_num-1])<<8) |  \
								  ReceBuff_485[Receive_485_num-2];  //���Ͻ��յ���CRC
				if(crc == recv_crc)  //CRCУ����ȷ
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
				else  //�Ƿ�����ֵ
				{
					Data_Error = 3;   //�Ƿ�����ֵ
					Status_Alarm = 0; //ͨ�ű���
				}
			}
			else  //�Ƿ����ݵ�ַ
			{
				Data_Error = 2;   //�Ƿ����ݵ�ַ
				Status_Alarm = 0; //ͨ�ű���
			}		
		}
		else if(ReceBuff_485[1]==0x83 || ReceBuff_485[1]==0x86 || ReceBuff_485[1]==0x88)
		{
			Data_Error = ReceBuff_485[2];   //ȡ��������
			Status_Alarm = 0; //ͨ�ű���
		}
		else
		{
			Data_Error = 1; //���������
			Status_Alarm = 0; //ͨ�ű���
		}	
	}
}

/**  
 *���ã�������ָ��03�����ݴ������
 *0x03����ȡ��� PSW/PFW �Ĵ����������� 
**/ 
void Modbus2_03_Solve(void)
{
	u8 i;
	u16 reg_num;
	reg_num = ((u16)ReceBuff_485[2]) % 2;   //��ȡ�Ĵ�������
	if(REG_Start_Addr2 + reg_num < REG_MAX)  //�Ĵ�����ַ+�����ڷ�Χ��
	{
		Status_Alarm = 1; //ͨ������		
//		for(i = 0; i < reg_num; i++)
//		{
//			(*Modbus_HoldReg[REG_Start_Addr+i]>>8)&0xFF = ReceBuff_485[3+i*2];	//�ȷ��͸��ֽ�
//			*Modbus_HoldReg[REG_Start_Addr+i]&0xFF = ReceBuff_485[4+i*2];  		//���͵��ֽ�			
//		}
	}
	else  //�Ƿ����ݵ�ַ
	{
		Data_Error = 2; //�Ƿ����ݵ�ַ
		Status_Alarm = 0; //ͨ�ű���
	}
}

/**  
 *���ã�������ָ��06�����ݴ������
**/ 
void Modbus2_06_Solve(void)
{
	int i;	
	i = strcmp((const char *)SendBuff_485, (const char *)ReceBuff_485);
	if(i == 0) //PLC���ͺͱ�Ƶ���ظ�һ��
	{
		if(ReceBuff_485[3] == 0x01)
			Last_Speed = Data_RunSpeed;
		else if(ReceBuff_485[3] == 0x00)
			Last_Dir = Motor_Dir; //�����ط���ͨ�����
		Status_Alarm = 1; //ͨ������
	}
	else  //�Ƿ�����ֵ
	{
		Data_Error = 3;   //�Ƿ�����ֵ
		Status_Alarm = 0; //ͨ�ű���
	}
}

/**  
 *���ã�������ָ��08�����ݴ������
**/ 
void Modbus2_08_Solve(void)
{
	int i;	
	i = strcmp((const char *)SendBuff_485, (const char *)ReceBuff_485);
	if(i == 0) //PLC���ͺͱ�Ƶ���ظ�һ��
	{
		Status_Alarm = 1; //ͨ������
	}
	else 
	{
		Data_Error = 3;   //�Ƿ�����ֵ
		Status_Alarm = 0; //ͨ�ű���
	}
}


