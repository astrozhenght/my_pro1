#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 								  

#define RS485_REC_LEN  			30  					//定义最大接收字节数 30

extern u16 RS485_RX_STA; 									//接收到的数据状态
extern u8  RS485_RX_BUF[RS485_REC_LEN]; 	//接收缓冲最大30个字节

//485模式控制
#define RECEIVE       0     							//接收
#define SEND          1     							//发送
#define RS485_TX_EN		PHout(13)						//485模式控制.0,接收;1,发送.


void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf, u8 len);
void RS485_Receive_Data(u8 *buf);

#endif	   
















