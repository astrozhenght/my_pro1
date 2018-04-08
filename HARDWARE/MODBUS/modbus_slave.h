#ifndef _MODBUS_SLAVE_H
#define _MODBUS_SLAVE_H
#include "sys.h"

extern u16     Present_Page;    			//Ĭ��Ϊһ����

extern u16     Page1_Status_Recover;  //��λָʾ�� 
extern u16     Page1_Button_Recover;  //��λ��ť 
extern u16     Page1_Button_Auto;     //�Զ�ģʽ�л���ť
extern u16     Page1_Button_Single;   //����ģʽ�л���ť
extern u16     Page1_Button_Manual;   //�ֶ�ģʽ�л���ť

extern u16     Page2_Button_Start;    //������ť 
extern u16     Page2_Button_Stop;     //ֹͣ��ť 
extern u16     Page2_Button_BackMain; //���������水ť 
extern u16     Page2_Status_Alarm;    //����ָʾ��
extern u16     Page2_Status_Pause;    //��ָͣʾ��
extern u16     Page2_Status_Return;   //����ָʾ��
extern u16     Page2_Status_Wait;     //�ȴ�ָʾ��
extern u16     Page2_Status_EndLeft;  //����ָʾ��
extern u16     Page2_Status_EndRight; //�Ҽ���ָʾ��
extern u16     Page2_Status_DirLeft;  //���˶�ָʾ��
extern u16     Page2_Status_DirRight; //���˶�ָʾ��

extern float   Page2_Data_RTSpeed; 	  //ʵʱ�ٶ�
extern float   Page2_Data_Location;   //ʵʱλ��
extern float   Page2_Data_Speed1;     //����1
extern float   Page2_Data_Speed2;     //����2
extern float   Page2_Data_Speed3;     //����3
extern float   Page2_Data_Speed4;     //����4
extern float   Page2_Data_BKSpeed;    //�����ٶ�

extern u16     Page3_Data_Stage;   		//��������
extern float   Page3_Data_Speed; 			//��������

extern float   Page4_Data_RunSpeed; 	//�����ٶ�
extern u16     Page4_Button_Left;     //���˶���ť 
extern u16     Page4_Button_Right;    //���˶���ť 
extern int     Page4_Data_Animation;  //����λ��



void Modbus_Parse(void);

u16 CRC16(u8 *addr, int num);  

void Modbus_03_Solve(void);
void Modbus_05_Solve(void);
void Modbus_06_Solve(void);
void Modbus_10_Solve(void);

void Modbus_RegMap(void);

#endif
