#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"

extern vu8   TIM5_Count;
extern u8    Motor_Dir;
extern float EXP_LOC;
	
extern vu8   Flag_Status_Motion;	
extern vu8   Flag_Init; 	//��ǰΪ�ϵ縴λ״̬

#define DOING       					0  //�ϵ縴λ״̬
#define DONE       						1  //�ϵ縴λ���״̬

//�����λ״̬
#define STATUS_REST       		0  //������ھ�ֹ״̬
#define STATUS_ONESTEP   			1  //��λ��һ��������
#define STATUS_TWOSTEP   			2  //��λ�ڶ���������
#define STATUS_THREESTEP   		3  //��λ������������
#define STATUS_AUTOMOD   			4  //�Զ�������
#define STATUS_SINGMOD   			5  //����������

#define PAGE_ONE        			1  //��һҳ
#define PAGE_TWO       			  2  //�ڶ�ҳ
#define PAGE_THREE       			3  //����ҳ
#define PAGE_FOUR        			4  //����ҳ

#define DIR_STOP			   			1  //ֹͣ
#define DIR_LEFT   						2  //�����˶�
#define DIR_RIGHT  						3  //�����˶�

#define CHG_NONE			   			0  //���ı�
#define CHG_DIR			   			  1  //�ı䷽��
#define CHG_SPD			   			  2  //�ı��ٶ�

void Mode_Control(void);
void Motor_Restore(void);

void FreqChg_Control(void);


#endif


