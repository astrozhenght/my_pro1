#include "modbus_master.h"
#include "modbus_slave.h"
#include "sys.h"
#include "dma.h"
#include "led.h"
#include "rs485.h"

#define RS232_ADDR  0x01   //�ӻ���ַ
#define REG_MAX	    1000   

u16   REG_Start_Addr;  	   //�Ĵ�����ʼ��ַ�����������ֽ����

u16   Present_Mode = 1;    //Ĭ��Ϊģʽѡ��
u16   Status_Recover = 0;  //��λָʾ�� 
u16   Button_Recover = 0;  //��λ��ť 
u16   Button_Auto = 0;     //�Զ�ģʽ�л���ť
u16   Button_Single = 0;   //����ģʽ�л���ť
u16   Button_Manual = 0;   //�ֶ�ģʽ�л���ť
u16   Button_Start = 0;    //������ť 
u16   Button_Stop = 0;     //ֹͣ��ť 
u16   Button_BackMain = 0; //���������水ť 
u16   Status_Alarm = 0;    //����ָʾ��
u16   Status_Pause = 0;    //��ָͣʾ��
u16   Status_Return = 0;   //����ָʾ��
u16   Status_Wait = 0;     //�ȴ�ָʾ��
u16   Status_EndLeft = 0;  //����ָʾ��
u16   Status_EndRight = 0; //�Ҽ���ָʾ��
u16   Status_DirLeft = 0;  //���˶�ָʾ��
u16   Status_DirRight = 0; //���˶�ָʾ��
float Data_RTSpeed = 0.0f;    //ʵʱ�ٶ�
float Data_Location = 0.0f;   //ʵʱλ��
float Data_Speed1 = 10.0f;    //����1
float Data_Speed2 = 10.0f;    //����2
float Data_Speed3 = 10.0f;    //����3
float Data_Speed4 = 10.0f;    //����4
float Data_BKSpeed = 10.0f;   //�����ٶ�
u16   Data_Stage = 0;   	  //��������
float Data_Speed = 10.0f;  	  //��������
float Data_RunSpeed = 10.0f;  //�����ٶ�
u16   Button_Left = 0;        //���˶���ť 
u16   Button_Right = 0;       //���˶���ť 
int   Data_Animation = 0;     //����λ��

u16*  Modbus_HoldReg[REG_MAX];  //���ּĴ���ָ�룬һ���ֽ�

void Modbus_RegMap(void)
{	
	//��ťָ��ָ��	0-19
	Modbus_HoldReg[0]  = &Button_Recover;  //��λ��ť 
	Modbus_HoldReg[1]  = &Button_Auto;     //�Զ�ģʽ�л���ť
	Modbus_HoldReg[2]  = &Button_Single;   //����ģʽ�л���ť
	Modbus_HoldReg[3]  = &Button_Manual;   //�ֶ�ģʽ�л���ť
	Modbus_HoldReg[4]  = &Button_Start;    //������ť
	Modbus_HoldReg[5]  = &Button_Stop;     //ֹͣ��ť 
	Modbus_HoldReg[6]  = &Button_BackMain; //���������水ť	
	Modbus_HoldReg[15] = &Button_Left;     //�ֶ���ť
	Modbus_HoldReg[16] = &Button_Right;    //�ֶ��Ұ�ť		
	//ָʾ��ָ�� 20-49 
	Modbus_HoldReg[22] = &Status_Recover; //��һҳ�׵�ַ	
	Modbus_HoldReg[23] = &Status_Return;  //�ڶ�ҳ�׵�ַ
	Modbus_HoldReg[24] = &Status_Wait;
	Modbus_HoldReg[25] = &Status_EndLeft;
	Modbus_HoldReg[26] = &Status_EndRight;
	Modbus_HoldReg[27] = &Status_DirLeft;
	Modbus_HoldReg[28] = &Status_DirRight;	
	Modbus_HoldReg[40] = &Status_Alarm;	
	//������  50-199
	Modbus_HoldReg[50] = ((u16*)(&Data_RTSpeed))+1;  //ʵʱ�ٶ�
	Modbus_HoldReg[51] = ((u16*)(&Data_RTSpeed))+0;     
	Modbus_HoldReg[52] = ((u16*)(&Data_Location))+1; //ʵʱλ��    
	Modbus_HoldReg[53] = ((u16*)(&Data_Location))+0;	
	Modbus_HoldReg[60] = ((u16*)(&Data_Speed1))+1; //����1
	Modbus_HoldReg[61] = ((u16*)(&Data_Speed1))+0; 	
	Modbus_HoldReg[62] = ((u16*)(&Data_Speed2))+1; //����2    
	Modbus_HoldReg[63] = ((u16*)(&Data_Speed2))+0;	
	Modbus_HoldReg[70] = ((u16*)(&Data_Speed3))+1; //����3
	Modbus_HoldReg[71] = ((u16*)(&Data_Speed3))+0;     
	Modbus_HoldReg[72] = ((u16*)(&Data_Speed4))+1; //����4    
	Modbus_HoldReg[73] = ((u16*)(&Data_Speed4))+0;	
	Modbus_HoldReg[80] = ((u16*)(&Data_BKSpeed))+1;//�����ٶ�   
	Modbus_HoldReg[81] = ((u16*)(&Data_BKSpeed))+0;	
	Modbus_HoldReg[82] = &Data_Stage;  			   //��������
	Modbus_HoldReg[90] = ((u16*)(&Data_Speed))+1;  //��������   
	Modbus_HoldReg[91] = ((u16*)(&Data_Speed))+0;	
	Modbus_HoldReg[92] = ((u16*)(&Data_Animation))+1;//����λ��   
	Modbus_HoldReg[93] = ((u16*)(&Data_Animation))+0;	
	Modbus_HoldReg[100] = ((u16*)(&Data_RunSpeed))+1;//�����ٶ�   
	Modbus_HoldReg[101] = ((u16*)(&Data_RunSpeed))+0;		
	Modbus_HoldReg[200] = &Present_Mode;   //��ʾ��ǰģʽ
}

/**  
 *���ã�PLC�봥������������ʹ����ModbusЭ�飬PLC��Ϊ�ӻ���ظ�������
	   �Ĳ�ѯ��Ϣ���������ModbusЭ�����ݵĽ������̡� 
**/ 
void Modbus_Parse(void)
{
	u16 crc, recv_crc;
	
	if(ReceBuff_232[0] == RS232_ADDR)  //��ַ��ȷ
	{
		if(ReceBuff_232[1]==0x03 || ReceBuff_232[1]==0x05 || \
			 ReceBuff_232[1]==0x06 || ReceBuff_232[1]==0x10 )     //��������ȷ
		{			
			REG_Start_Addr = (((u16)ReceBuff_232[2])<<8)|ReceBuff_232[3];  //��üĴ�����ʼ��ַ
			if(REG_Start_Addr < REG_MAX) 	//�Ĵ�����ַ�ڷ�Χ��
			{
				crc = CRC16(ReceBuff_232, Receive_byte_num - 2);  //����������ݵ�CRC
				recv_crc = ((u16)(ReceBuff_232[Receive_byte_num-1])<<8) |  \
													ReceBuff_232[Receive_byte_num-2];  //���Ͻ��յ���CRC
				if(crc == recv_crc)  //CRCУ����ȷ
				{
					switch(ReceBuff_232[1])
					{
						case 0x03:
							Modbus_03_Solve();
							break;
						case 0x05:
							Modbus_05_Solve();
							break;
						case 0x06:
							Modbus_06_Solve();
							break;
						case 0x10:
							Modbus_10_Solve();
							break;
					}					
				}
				else  //CRCУ�����
				{
					SendBuff_232[0] = ReceBuff_232[0];
					SendBuff_232[1] = ReceBuff_232[1]|0x80;
					SendBuff_232[2] = 0x03;  //�������ݴ���
					DMA2_232_Send(3);
				}
			}
			else  //�������׵�ַ����
			{
				SendBuff_232[0] = ReceBuff_232[0];
				SendBuff_232[1] = ReceBuff_232[1]|0x80;
				SendBuff_232[2] = 0x02;
				DMA2_232_Send(3);
			}		
		}
		else //���������
		{
			SendBuff_232[0] = ReceBuff_232[0];
			SendBuff_232[1] = ReceBuff_232[1]|0x80;
			SendBuff_232[2] = 0x01;
			DMA2_232_Send(3);
		} 
	}
}

/**  
 *���ã�������ָ��03�����ݴ������
 *0x03����ȡ��� PSW/PFW �Ĵ����������� 
**/ 
void Modbus_03_Solve(void)
{
	u8 i;
	u16 reg_num;
	u16 crc;
	reg_num = (((u16)ReceBuff_232[4])<<8) | ReceBuff_232[5];  //��ȡ�Ĵ�������
	if(REG_Start_Addr + reg_num < REG_MAX)  //�Ĵ�����ַ+�����ڷ�Χ��
	{
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1];
		SendBuff_232[2] = reg_num * 2;  //��ȡ�ֽ���		
		
		for(i = 0; i < reg_num; i++)
		{
			SendBuff_232[3+i*2] = (*Modbus_HoldReg[REG_Start_Addr+i]>>8)&0xFF;	//�ȷ��͸��ֽ�
			SendBuff_232[4+i*2] = *Modbus_HoldReg[REG_Start_Addr+i]&0xFF;  			//���͵��ֽ�			
		}
		
		crc = CRC16(SendBuff_232, reg_num*2+3);       //����CRC
		SendBuff_232[reg_num*2+3] = (u8)crc&0xFF;     //CRC�ĵ��ֽ�
		SendBuff_232[reg_num*2+4] = (u8)(crc>>8)&0xFF;//CRC�ĸ��ֽ�
		DMA2_232_Send(reg_num*2+5);  //DMA����
	}
	else  //����������Χ
	{
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1]|0x80;
		SendBuff_232[2] = 0x02;
		DMA2_232_Send(3);
	}
}

/**  
 *���ã�������ָ��05�����ݴ������
 *0x05��д����Ȧ  
**/ 
void Modbus_05_Solve(void)
{
	u16 crc;
	if(REG_Start_Addr < REG_MAX)  //�Ĵ�����ַ�ڷ�Χ��
	{
		if((ReceBuff_232[4]==0xFF)||(ReceBuff_232[5]==0xFF))
		{
			*Modbus_HoldReg[REG_Start_Addr] = 0x0001;			
		}			
    else 
		{
			*Modbus_HoldReg[REG_Start_Addr] = 0x0000;			
		}				
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1];
		SendBuff_232[2] = ReceBuff_232[2];  	
		SendBuff_232[3] = ReceBuff_232[3];  	
		SendBuff_232[4] = ReceBuff_232[4];  	
		SendBuff_232[5] = ReceBuff_232[5]; 
		
		crc = CRC16(SendBuff_232, 6);    //����CRC
		SendBuff_232[6] = (u8)crc&0xFF;   		 //CRC�ĵ��ֽ�
		SendBuff_232[7] = (u8)(crc>>8)&0xFF;   //CRC�ĸ��ֽ�
		DMA2_232_Send(8);  //DMA���ͣ�8���ֽ�
	}
	else  //�Ĵ�����ַ������Χ
	{
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1]|0x80;
		SendBuff_232[2] = 0x02;  //�쳣��
		DMA2_232_Send(3);
	}
		
}
/**  
 *���ã�������ָ��06�����ݴ������
 *0x06�������Ĵ��� PSW/PFW д
**/ 
void Modbus_06_Solve(void)
{
	u16 crc;
	
	//��ָ��ָ������ݸ�ֵ����ֵ��ϾͿ���ʹ��freq		
	*Modbus_HoldReg[REG_Start_Addr]  = ((u16)ReceBuff_232[4])<<8;   //�Žݸ��ֽ���ǰ
	*Modbus_HoldReg[REG_Start_Addr] |= ReceBuff_232[5]; 	 					//���ֽ��ں�
	
	SendBuff_232[0] = ReceBuff_232[0];
	SendBuff_232[1] = ReceBuff_232[1];
	SendBuff_232[2] = ReceBuff_232[2];  	
	SendBuff_232[3] = ReceBuff_232[3];  	
	SendBuff_232[4] = ReceBuff_232[4];  	
	SendBuff_232[5] = ReceBuff_232[5];  	
	
	crc = CRC16(SendBuff_232, 6);       //����CRC
	SendBuff_232[6] = (u8)crc&0xFF;     	 //CRC�ĵ��ֽ�
	SendBuff_232[7] = (u8)(crc>>8)&0xFF;   //CRC�ĸ��ֽ�
	DMA2_232_Send(8);  //DMA���ͣ�8���ֽ�
		
}

/**  
 *���ã�������ָ��10�����ݴ������
 *0x10������Ĵ��� PSW/PFW д
**/ 
void Modbus_10_Solve(void)
{
	u8 i;
	u16 reg_num;
	u16 crc;
	reg_num = (((u16)ReceBuff_232[4])<<8) | ReceBuff_232[5];  //��ȡ�Ĵ�������
	if(REG_Start_Addr + reg_num < REG_MAX)  //�Ĵ�����ַ+�����ڷ�Χ��
	{
		for(i = 0; i < reg_num; i++)
		{
			//��ָ��ָ������ݸ�ֵ����ֵ��ϾͿ���ʹ��freq		
			*Modbus_HoldReg[REG_Start_Addr+i]  = ((u16)ReceBuff_232[7+i*2])<<8; 	 //���ֽ���ǰ
			*Modbus_HoldReg[REG_Start_Addr+i] |= ReceBuff_232[8+i*2];  						 //���ֽ��ں�
		}		
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1];
		SendBuff_232[2] = ReceBuff_232[2];  	
		SendBuff_232[3] = ReceBuff_232[3];  	
		SendBuff_232[4] = ReceBuff_232[4];  	
		SendBuff_232[5] = ReceBuff_232[5];  	
		
		crc = CRC16(SendBuff_232, 6);       //����CRC
		SendBuff_232[6] = (u8)crc&0xFF;     	 //CRC�ĵ��ֽ�
		SendBuff_232[7] = (u8)(crc>>8)&0xFF;   //CRC�ĸ��ֽ�
		DMA2_232_Send(8);  //DMA���ͣ�8���ֽ�	
	}
	else  //����������Χ
	{
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1]|0x80;
		SendBuff_232[2] = 0x02;
		DMA2_232_Send(3);
	}
}
/**  
 *Calculating CRC16-RTUstandard  
 *@para addr,start of data  
 *@para num,length of data  
**/  
u16 CRC16(u8 *addr, int num)  
{  
	int i;
	u16 crc = 0xffff;
	for(;num > 0;num--)  
	{  
		crc = crc^(*addr++);  
		for(i = 0; i < 8; i++)  
		{  
			if(crc & 0x0001)  
				crc = (crc>>1)^0xa001;  
			else  
				crc >>= 1;  
		}  
	}  
	return crc;  
}  


