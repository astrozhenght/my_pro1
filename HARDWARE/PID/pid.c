#include "pid.h"
#include "control.h"
#include "modbus_slave.h"
#include "sys.h"
#include "timer.h"

//��Χ[0.0, 0.05]

float P,I,D;
float Err[2];
float SUM_Err = 0.0f;

void PID_Init(void)
{
	P = 6.0f;  	//��Ӧ�ٶ�
	I = 4.0f;   //��̬���
	D = 6.0f;   //������
}

/**
 *info:λ��ʽPID
 *����������λ�ã���ǰλ�ã�����ٶ�
 *����ֵ��PID�õ����ٶ�ֵ
**/
float PID_Calculate(float exp_distance, float now_distance, float limit)
{
	static u8 flag = 0;
	float output;
	Err[0] = exp_distance - now_distance;  //����ֵ-����ֵ����תʱ�������ֵ
	Err[1] = Err[0];

	//�����޷�
	if((Err[0]<0.1f) && (Err[0]>-0.1f))  //0.1mm���ڿ�ʼ����������0.05����
	{
		SUM_Err += Err[0];
		flag = 1;
	}
	else
	{
		flag = 0;
	}
	if(SUM_Err > 0.15f)  //�������0.6mm/s
	{
		SUM_Err = 0.15f;
	}
	else if(SUM_Err < -0.15f)
	{
		SUM_Err = -0.15f;
	}
	
	//�������������0.6mm/s
	output = P*Err[0] + flag*I*SUM_Err + D*(Err[0] - Err[1]);
	
	//����޷�
	if(output > limit)  //�������ֵ
		output = limit;
	else if(output < -limit) //С����Сֵ
		output = -limit;
	
	//�޶�����
	if(output < 0)
		output *= -1.0f; 
	return output;
}

/**
 *��Ҫ����λ�ú��ٶ�
 *λ�ã�position
 *�ٶȣ�speed
 *note��������TIM5�����ж��Ƿ����
 *���ȣ�+-0.03mm
**/
void PID_Control_SPD(float position, float speed)
{
	//pid������ֵ�����ٶ�
	Page4_Data_RunSpeed = PID_Calculate(position, Page2_Data_Location, speed);  //����40mm���ٶ����15mm/s
				
	if(Page2_Data_Location > (position+0.02f))
	{
		Motor_Dir = DIR_LEFT;  //�����ת
	}
	else if(Page2_Data_Location < (position-0.03f))
	{
		Motor_Dir = DIR_RIGHT; //�����ת
	}
	else //����Ŀ��㣬������ʱ���������жϺ��ж�״̬�Ƿ���ֹͣ
	{	
		if(Motor_Dir != DIR_STOP)  //����ֹͣ��ʱ�����һ�Σ�����
		{
			TIM5_Count = 2;	  //400ms��
			TIM5->CNT = 0;    //���ϼ�����������ֵ����
			TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5���жϼ�ʱ		
			Motor_Dir = DIR_STOP;  	//���ֹͣת��
		}			
	}
}

