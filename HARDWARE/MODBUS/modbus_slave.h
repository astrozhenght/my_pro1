#ifndef _MODBUS_SLAVE_H
#define _MODBUS_SLAVE_H
#include "sys.h"

extern u16   Present_Mode;    //Ĭ��Ϊģʽѡ��
extern u16   Status_Recover;  //��λָʾ�� 
extern u16   Button_Recover;  //��λ��ť 
extern u16   Button_Auto;     //�Զ�ģʽ�л���ť
extern u16   Button_Single;   //����ģʽ�л���ť
extern u16   Button_Manual;   //�ֶ�ģʽ�л���ť

extern u16   Button_Start;    //������ť 
extern u16   Button_Stop;     //ֹͣ��ť 
extern u16   Button_BackMain; //���������水ť 
extern u16   Status_Alarm;    //����ָʾ��
extern u16   Status_Pause;    //��ָͣʾ��
extern u16   Status_Return;   //����ָʾ��
extern u16   Status_Wait;     //�ȴ�ָʾ��
extern u16   Status_EndLeft;  //����ָʾ��
extern u16   Status_EndRight; //�Ҽ���ָʾ��
extern u16   Status_DirLeft;  //���˶�ָʾ��
extern u16   Status_DirRight; //���˶�ָʾ��

extern float Data_RTSpeed; 	//ʵʱ�ٶ�
extern float Data_Location;   //ʵʱλ��
extern float Data_Speed1;     //����1
extern float Data_Speed2;     //����2
extern float Data_Speed3;     //����3
extern float Data_Speed4;     //����4
extern float Data_BKSpeed;    //�����ٶ�

extern u16   Data_Stage;   	//��������
extern float Data_Speed; 		//��������

extern float Data_RunSpeed; 	//�����ٶ�
extern u16   Button_Left;     //���˶���ť 
extern u16   Button_Right;    //���˶���ť 
extern int   Data_Animation;  //����λ��

void Modbus_Parse(void);

u16 CRC16(u8 *addr, int num);  

void Modbus_03_Solve(void);
void Modbus_05_Solve(void);
void Modbus_06_Solve(void);
void Modbus_10_Solve(void);

void Modbus_RegMap(void);

#endif
