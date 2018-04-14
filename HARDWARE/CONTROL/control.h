#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"

#define DOING       			0  //�ϵ縴λ״̬
#define DONE       				1  //�ϵ縴λ���״̬

//ϵͳ״̬
#define STATUS_REST       		0  //��ֹ״̬
#define STATUS_ONESTEP   		1  //��λ��һ��״̬
#define STATUS_TWOSTEP   		2  //��λ�ڶ���״̬
#define STATUS_AUTOMOD   		3  //�Զ�״̬
#define STATUS_SINGMOD   		4  //����״̬

#define MODE_SELECT        		1  //ģʽѡ��
#define MODE_AUTO       		2  //�Զ�ģʽ
#define MODE_SINGLE       		3  //����ģʽ
#define MODE_MANUAL        		4  //�ֶ�ģʽ

#define DIR_STOP			   	1  //ֹͣ
#define DIR_LEFT   				2  //�����˶�
#define DIR_RIGHT  				3  //�����˶�

#define RECEIVE2			    1 
#define SEND2    				2  
#define WAITING    				3  

extern vu8   TIM5_Count;
extern float EXP_LOC;

extern vu8   Status_Motion;	
extern vu8   Flag_Init;
extern vu8   order_type;   //��Ƶ����������

extern float Last_Speed;   //��һ�ε��ٶ�ֵ������͵�ǰ���ٶȲ���Ⱦ�Ҫ�ı��ٶ�
extern vu8   Motor_Dir;    //�����ǰ����Ĭ��ֹͣ
extern vu8   Last_Dir;     //����ϴεķ���Ĭ��ֹͣ

void Mode_Control(void);
void Motor_Restore(void);

void FreqChg_Control(void);
void FreqRev_Deal(void);  //�������֮�����


#endif


