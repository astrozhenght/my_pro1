#ifndef __DMA_H
#define	__DMA_H	   
#include "sys.h"

//232收发DMA缓存
#define LEN_SEND_232   (u16)20
#define LEN_RECV_232   (u16)20

//485收发DMA缓存
#define LEN_SEND_485   (u16)20
#define LEN_RECV_485   (u16)20


extern vu32 Receive_byte_num;

extern vu8 Receive_232_flag;

extern u8 SendBuff_232[LEN_SEND_232];
extern u8 ReceBuff_232[LEN_RECV_232];

extern u8 SendBuff_485[LEN_SEND_485];
extern u8 ReceBuff_485[LEN_RECV_485];


void DMA2_232_Config(void);
void DMA1_485_Config(void);

void DMA2_232_Send(u16 size);
void DMA1_485_Send(u16 size);

#endif






























