#include "modbus_master.h"
#include "modbus_slave.h"
#include "sys.h"
#include "dma.h"
#include "led.h"
#include "rs485.h"

#define RS232_ADDR  0x01   //从机地址
#define REG_MAX	    1000   

u16   REG_Start_Addr;  	   //寄存器起始地址变量，两个字节组成

u16   Present_Mode = 1;    //默认为模式选择
u16   Status_Recover = 0;  //复位指示灯 
u16   Button_Recover = 0;  //复位按钮 
u16   Button_Auto = 0;     //自动模式切换按钮
u16   Button_Single = 0;   //单步模式切换按钮
u16   Button_Manual = 0;   //手动模式切换按钮
u16   Button_Start = 0;    //启动按钮 
u16   Button_Stop = 0;     //停止按钮 
u16   Button_BackMain = 0; //返回主界面按钮 
u16   Status_Alarm = 0;    //警报指示灯
u16   Status_Pause = 0;    //暂停指示灯
u16   Status_Return = 0;   //返回指示灯
u16   Status_Wait = 0;     //等待指示灯
u16   Status_EndLeft = 0;  //左极限指示灯
u16   Status_EndRight = 0; //右极限指示灯
u16   Status_DirLeft = 0;  //左运动指示灯
u16   Status_DirRight = 0; //右运动指示灯
float Data_RTSpeed = 0.0f;    //实时速度
float Data_Location = 0.0f;   //实时位置
float Data_Speed1 = 10.0f;    //段速1
float Data_Speed2 = 10.0f;    //段速2
float Data_Speed3 = 10.0f;    //段速3
float Data_Speed4 = 10.0f;    //段速4
float Data_BKSpeed = 10.0f;   //返回速度
u16   Data_Stage = 0;   	  //单步段数
float Data_Speed = 10.0f;  	  //单步段速
float Data_RunSpeed = 10.0f;  //运行速度
u16   Button_Left = 0;        //左运动按钮 
u16   Button_Right = 0;       //右运动按钮 
int   Data_Animation = 0;     //动画位置

u16*  Modbus_HoldReg[REG_MAX];  //保持寄存器指针，一个字节

void Modbus_RegMap(void)
{	
	//按钮指针指向	0-19
	Modbus_HoldReg[0]  = &Button_Recover;  //复位按钮 
	Modbus_HoldReg[1]  = &Button_Auto;     //自动模式切换按钮
	Modbus_HoldReg[2]  = &Button_Single;   //单步模式切换按钮
	Modbus_HoldReg[3]  = &Button_Manual;   //手动模式切换按钮
	Modbus_HoldReg[4]  = &Button_Start;    //启动按钮
	Modbus_HoldReg[5]  = &Button_Stop;     //停止按钮 
	Modbus_HoldReg[6]  = &Button_BackMain; //返回主界面按钮	
	Modbus_HoldReg[15] = &Button_Left;     //手动左按钮
	Modbus_HoldReg[16] = &Button_Right;    //手动右按钮		
	//指示灯指向 20-49 
	Modbus_HoldReg[22] = &Status_Recover; //第一页首地址	
	Modbus_HoldReg[23] = &Status_Return;  //第二页首地址
	Modbus_HoldReg[24] = &Status_Wait;
	Modbus_HoldReg[25] = &Status_EndLeft;
	Modbus_HoldReg[26] = &Status_EndRight;
	Modbus_HoldReg[27] = &Status_DirLeft;
	Modbus_HoldReg[28] = &Status_DirRight;	
	Modbus_HoldReg[40] = &Status_Alarm;	
	//浮点数  50-199
	Modbus_HoldReg[50] = ((u16*)(&Data_RTSpeed))+1;  //实时速度
	Modbus_HoldReg[51] = ((u16*)(&Data_RTSpeed))+0;     
	Modbus_HoldReg[52] = ((u16*)(&Data_Location))+1; //实时位置    
	Modbus_HoldReg[53] = ((u16*)(&Data_Location))+0;	
	Modbus_HoldReg[60] = ((u16*)(&Data_Speed1))+1; //段速1
	Modbus_HoldReg[61] = ((u16*)(&Data_Speed1))+0; 	
	Modbus_HoldReg[62] = ((u16*)(&Data_Speed2))+1; //段速2    
	Modbus_HoldReg[63] = ((u16*)(&Data_Speed2))+0;	
	Modbus_HoldReg[70] = ((u16*)(&Data_Speed3))+1; //段速3
	Modbus_HoldReg[71] = ((u16*)(&Data_Speed3))+0;     
	Modbus_HoldReg[72] = ((u16*)(&Data_Speed4))+1; //段速4    
	Modbus_HoldReg[73] = ((u16*)(&Data_Speed4))+0;	
	Modbus_HoldReg[80] = ((u16*)(&Data_BKSpeed))+1;//返回速度   
	Modbus_HoldReg[81] = ((u16*)(&Data_BKSpeed))+0;	
	Modbus_HoldReg[82] = &Data_Stage;  			   //单步段数
	Modbus_HoldReg[90] = ((u16*)(&Data_Speed))+1;  //单步段速   
	Modbus_HoldReg[91] = ((u16*)(&Data_Speed))+0;	
	Modbus_HoldReg[92] = ((u16*)(&Data_Animation))+1;//动画位置   
	Modbus_HoldReg[93] = ((u16*)(&Data_Animation))+0;	
	Modbus_HoldReg[100] = ((u16*)(&Data_RunSpeed))+1;//运行速度   
	Modbus_HoldReg[101] = ((u16*)(&Data_RunSpeed))+0;		
	Modbus_HoldReg[200] = &Present_Mode;   //表示当前模式
}

/**  
 *作用：PLC与触摸屏传输数据使用了Modbus协议，PLC作为从机会回复触摸屏
	   的查询信息，下面就是Modbus协议数据的解析过程。 
**/ 
void Modbus_Parse(void)
{
	u16 crc, recv_crc;
	
	if(ReceBuff_232[0] == RS232_ADDR)  //地址正确
	{
		if(ReceBuff_232[1]==0x03 || ReceBuff_232[1]==0x05 || \
			 ReceBuff_232[1]==0x06 || ReceBuff_232[1]==0x10 )     //功能码正确
		{			
			REG_Start_Addr = (((u16)ReceBuff_232[2])<<8)|ReceBuff_232[3];  //获得寄存器起始地址
			if(REG_Start_Addr < REG_MAX) 	//寄存器地址在范围内
			{
				crc = CRC16(ReceBuff_232, Receive_byte_num - 2);  //计算接收数据的CRC
				recv_crc = ((u16)(ReceBuff_232[Receive_byte_num-1])<<8) |  \
													ReceBuff_232[Receive_byte_num-2];  //整合接收到的CRC
				if(crc == recv_crc)  //CRC校验正确
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
				else  //CRC校验错误
				{
					SendBuff_232[0] = ReceBuff_232[0];
					SendBuff_232[1] = ReceBuff_232[1]|0x80;
					SendBuff_232[2] = 0x03;  //数据内容错误
					DMA2_232_Send(3);
				}
			}
			else  //域数据首地址错误
			{
				SendBuff_232[0] = ReceBuff_232[0];
				SendBuff_232[1] = ReceBuff_232[1]|0x80;
				SendBuff_232[2] = 0x02;
				DMA2_232_Send(3);
			}		
		}
		else //功能码错误
		{
			SendBuff_232[0] = ReceBuff_232[0];
			SendBuff_232[1] = ReceBuff_232[1]|0x80;
			SendBuff_232[2] = 0x01;
			DMA2_232_Send(3);
		} 
	}
}

/**  
 *作用：功能码指令03的数据处理过程
 *0x03：读取多个 PSW/PFW 寄存器数据内容 
**/ 
void Modbus_03_Solve(void)
{
	u8 i;
	u16 reg_num;
	u16 crc;
	reg_num = (((u16)ReceBuff_232[4])<<8) | ReceBuff_232[5];  //获取寄存器数量
	if(REG_Start_Addr + reg_num < REG_MAX)  //寄存器地址+数量在范围内
	{
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1];
		SendBuff_232[2] = reg_num * 2;  //读取字节数		
		
		for(i = 0; i < reg_num; i++)
		{
			SendBuff_232[3+i*2] = (*Modbus_HoldReg[REG_Start_Addr+i]>>8)&0xFF;	//先发送高字节
			SendBuff_232[4+i*2] = *Modbus_HoldReg[REG_Start_Addr+i]&0xFF;  			//后发送低字节			
		}
		
		crc = CRC16(SendBuff_232, reg_num*2+3);       //计算CRC
		SendBuff_232[reg_num*2+3] = (u8)crc&0xFF;     //CRC的低字节
		SendBuff_232[reg_num*2+4] = (u8)(crc>>8)&0xFF;//CRC的高字节
		DMA2_232_Send(reg_num*2+5);  //DMA发送
	}
	else  //数量超出范围
	{
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1]|0x80;
		SendBuff_232[2] = 0x02;
		DMA2_232_Send(3);
	}
}

/**  
 *作用：功能码指令05的数据处理过程
 *0x05：写单线圈  
**/ 
void Modbus_05_Solve(void)
{
	u16 crc;
	if(REG_Start_Addr < REG_MAX)  //寄存器地址在范围内
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
		
		crc = CRC16(SendBuff_232, 6);    //计算CRC
		SendBuff_232[6] = (u8)crc&0xFF;   		 //CRC的低字节
		SendBuff_232[7] = (u8)(crc>>8)&0xFF;   //CRC的高字节
		DMA2_232_Send(8);  //DMA发送，8个字节
	}
	else  //寄存器地址超出范围
	{
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1]|0x80;
		SendBuff_232[2] = 0x02;  //异常码
		DMA2_232_Send(3);
	}
		
}
/**  
 *作用：功能码指令06的数据处理过程
 *0x06：单个寄存器 PSW/PFW 写
**/ 
void Modbus_06_Solve(void)
{
	u16 crc;
	
	//给指针指向的内容赋值，赋值完毕就可以使用freq		
	*Modbus_HoldReg[REG_Start_Addr]  = ((u16)ReceBuff_232[4])<<8;   //信捷高字节在前
	*Modbus_HoldReg[REG_Start_Addr] |= ReceBuff_232[5]; 	 					//低字节在后
	
	SendBuff_232[0] = ReceBuff_232[0];
	SendBuff_232[1] = ReceBuff_232[1];
	SendBuff_232[2] = ReceBuff_232[2];  	
	SendBuff_232[3] = ReceBuff_232[3];  	
	SendBuff_232[4] = ReceBuff_232[4];  	
	SendBuff_232[5] = ReceBuff_232[5];  	
	
	crc = CRC16(SendBuff_232, 6);       //计算CRC
	SendBuff_232[6] = (u8)crc&0xFF;     	 //CRC的低字节
	SendBuff_232[7] = (u8)(crc>>8)&0xFF;   //CRC的高字节
	DMA2_232_Send(8);  //DMA发送，8个字节
		
}

/**  
 *作用：功能码指令10的数据处理过程
 *0x10：多个寄存器 PSW/PFW 写
**/ 
void Modbus_10_Solve(void)
{
	u8 i;
	u16 reg_num;
	u16 crc;
	reg_num = (((u16)ReceBuff_232[4])<<8) | ReceBuff_232[5];  //获取寄存器数量
	if(REG_Start_Addr + reg_num < REG_MAX)  //寄存器地址+数量在范围内
	{
		for(i = 0; i < reg_num; i++)
		{
			//给指针指向的内容赋值，赋值完毕就可以使用freq		
			*Modbus_HoldReg[REG_Start_Addr+i]  = ((u16)ReceBuff_232[7+i*2])<<8; 	 //高字节在前
			*Modbus_HoldReg[REG_Start_Addr+i] |= ReceBuff_232[8+i*2];  						 //低字节在后
		}		
		SendBuff_232[0] = ReceBuff_232[0];
		SendBuff_232[1] = ReceBuff_232[1];
		SendBuff_232[2] = ReceBuff_232[2];  	
		SendBuff_232[3] = ReceBuff_232[3];  	
		SendBuff_232[4] = ReceBuff_232[4];  	
		SendBuff_232[5] = ReceBuff_232[5];  	
		
		crc = CRC16(SendBuff_232, 6);       //计算CRC
		SendBuff_232[6] = (u8)crc&0xFF;     	 //CRC的低字节
		SendBuff_232[7] = (u8)(crc>>8)&0xFF;   //CRC的高字节
		DMA2_232_Send(8);  //DMA发送，8个字节	
	}
	else  //数量超出范围
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


