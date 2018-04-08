#ifndef __RS485_H
#define __RS485_H			 
#include "sys.h"	 								  

#define RS485_REC_LEN  			30  					//�����������ֽ��� 30

extern u16 RS485_RX_STA; 									//���յ�������״̬
extern u8  RS485_RX_BUF[RS485_REC_LEN]; 	//���ջ������30���ֽ�

//485ģʽ����
#define RECEIVE       0     							//����
#define SEND          1     							//����
#define RS485_TX_EN		PHout(13)						//485ģʽ����.0,����;1,����.


void RS485_Init(u32 bound);
void RS485_Send_Data(u8 *buf, u8 len);
void RS485_Receive_Data(u8 *buf);

#endif	   
















