#include "control.h"
#include "modbus_slave.h"
#include "modbus_master.h"
#include "led.h"
#include "exti.h"
#include "encoder.h"
#include "time.h"
#include "delay.h"
#include "pid.h"

long  Encoder_Pulse_NUM = 0; //����ʵʱλ��
long  Last_Pulse_NUM    = 0; //��ǰ��������Ϊ0 
vu8   Flag_Init = DOING; 	 //��ǰΪ�����ϵ縴λ
vu8   Flag_Status_Motion = STATUS_REST; //��ǰΪ��ֹ״̬
u8    Motor_Dir = DIR_STOP;  //���Ϊֹͣ״̬
vu8   TIM5_Count = 0; 		 //��ʱ��5�ļ�ʱ����
float Last_Speed   = 0.0f;   //��һ�ε�Ƶ��ֵ������͵�ǰ�Ĳ�һ����Ҫ����
u8    Last_Dir     = DIR_STOP;   //Ĭ��״̬��ֹ
volatile float Location_Left_Limit = 0.0f;   //������λ����������ޣ��ͼ�¼����λ��

//��¼�Զ�ģʽ�°�������ʱ���ĸ��ٶȣ��˶������и����ٶ���Ч
float Present_Data_Speed[6] = {0};  //1-5�洢���ݣ�0��������   

/**
 *���ã�ģʽ�л����߼����Ʋ���
**/
void Mode_Control(void)
{	
	static u8  Flag_Page4_Button_Left = 0; 	 //��ťδ���µı�־
	static u8  Flag_Page4_Button_Right = 0;  //�Ұ�ťδ���µı�־
	static u8  Flag_PLC_Page = PAGE_ONE;     //����PLCĬ�ϵ�ǰΪ��һҳ�棬ҳ�������PLC�д���
	
/******************* ģʽѡ�����--ҳ���л� 		****************************/
	if(Flag_PLC_Page == PAGE_ONE)
	{	
		if(Flag_Status_Motion == STATUS_REST)  //�ھ�ֹ������
		{
			if((Page2_Data_Location>=-0.0f) && (Page2_Data_Location<0.05f)) //����̨��ԭ��
			{				
				Page1_Button_Recover = 0;  //��ԭ�㰴�¸�λ����Ӧ
				if(Page1_Button_Auto == 1) //�����Զ�ģʽ����
				{
					Present_Page = 2; 			//�������л����ڶ�ҳ���л����Զ�����������0
					Flag_PLC_Page = 2;  		//PLC��¼��ǰҳ��
					Page1_Button_Auto = 0;
				}
				else if(Page1_Button_Single == 1)  //���µ���ģʽ����
				{
					Present_Page = 3;	        //�������л�������ҳ��֮���Զ�����������0
					Flag_PLC_Page = 3;  		//PLC��¼��ǰҳ��
					Page1_Button_Single = 0;				
				}
			}
			else    //����̨����ԭ��
			{				
				if(Page1_Button_Recover == 1)  		//��λ���£������λ
				{
					Motor_Restore();  	   //�����λ������ֻ�ڰ�ť���º�ִ�У�ִ��һ�Σ�����
					Page1_Button_Recover = 0;
				}
				
				Page1_Button_Auto = 0; 	//�����Զ�ģʽ��ť������Ӧ
				Page1_Button_Single = 0; //���µ���ģʽ��ť������Ӧ			
			}			
			//�ڲ���ԭ�㣬������Ӧ�ֶ�ģʽ
			if(Page1_Button_Manual == 1)  //�����ֶ�ģʽ
			{ 			
				Present_Page = 4;	          //�������л�������ҳ��֮���Զ�����������0
				Flag_PLC_Page = 4;  			  //PLC��¼��ǰҳ��
				Page1_Button_Manual = 0;
			}		
		}
		else  //�ڸ�λ�����У����°�������ť����Ӧ
		{
			Page1_Button_Auto = 0; 	//�����Զ�ģʽ��ť������Ӧ
			Page1_Button_Single = 0;//���µ���ģʽ��ť������Ӧ
			Page1_Button_Manual = 0;//�����ֶ�ģʽ��ť������Ӧ
			Page1_Button_Recover = 0; //���¸�λ��ť������Ӧ
		}
		
		if(Page2_Button_Stop == 1)
		{
			Motor_Dir = DIR_STOP; //���ֹͣת��	
			Page2_Status_Return = 0; //����״̬����0
			Flag_Status_Motion = STATUS_REST; //�˶�״̬Ϊֹͣ
			Page2_Button_Stop = 0;
		}
	}

/******************* �Զ�ģʽ����--�����Զ�ģʽ ****************************/
	if(Flag_PLC_Page == PAGE_TWO)
	{
		if(Page2_Button_Start == 1)  //����������ť
		{
			if(Flag_Status_Motion == STATUS_REST) //ֹͣ״̬
			{
				if((Page2_Data_Location>=-0.0f) && (Page2_Data_Location<0.05f)) //����̨��ԭ��
				{
					if((Page2_Data_Speed1>0) && (Page2_Data_Speed2>0) && \
						 (Page2_Data_Speed3>0) && (Page2_Data_Speed4>0) && \
						 (Page2_Data_BKSpeed>0) ) //����ٶȾ�����0
					{
						Present_Data_Speed[1] = Page2_Data_Speed1;  //��¼��ǰ�����һ���ٶ�				
						Present_Data_Speed[2] = Page2_Data_Speed2;  //��¼��ǰ����Ķ����ٶ�				
						Present_Data_Speed[3] = Page2_Data_Speed3;  //��¼��ǰ����������ٶ�				
						Present_Data_Speed[4] = Page2_Data_Speed4;  //��¼��ǰ������Ķ��ٶ�
						Present_Data_Speed[5] = Page2_Data_BKSpeed; //��¼��ǰ����ķ����ٶ�
						
						Page3_Data_Stage = 1;  //��ǰ����Ϊ��һ��						
						Flag_Status_Motion = STATUS_AUTOMOD;  //��־��ǰ�˶�״̬Ϊ�Զ��˶�״̬
					}
				}	
			}	
			Page2_Button_Start = 0;
		}
		else if(Page1_Button_Recover == 1)  //���¸�λ
		{
			if(Flag_Status_Motion == STATUS_REST) //ֹͣ״̬
			{			
				Motor_Restore(); 					//��λ
			}
			Page1_Button_Recover = 0;
		}
		else if(Page2_Button_Stop == 1) //����ֹͣ��ť
		{
			Motor_Dir = DIR_STOP; 	 //���ֹͣת��	
			Page2_Status_Return = 0; //���ص���
			Page2_Status_Wait = 0;   //�ȴ�ָʾ����
			Flag_Status_Motion = STATUS_REST; //��־��ǰ״̬Ϊ��ֹ
			Page3_Data_Stage = 0;    //��������
			Page2_Button_Stop = 0;  
		}	
	}

/******************* ����ģʽ����--���°�ť 		****************************/
	if(Flag_PLC_Page == PAGE_THREE)
	{
		if(Page2_Button_Start == 1) //����������ť
		{
			if(Flag_Status_Motion == STATUS_REST)  //��ǰ�˶�״̬Ϊ��ֹ
			{
				Page3_Data_Stage++;  //������һ
				if(Page3_Data_Stage == 1) //��һ�ΰ���
				{
					if((Page2_Data_Location>=-0.0f) && (Page2_Data_Location<0.05f)) //����̨��ԭ��
					{
						Flag_Status_Motion = STATUS_SINGMOD; //��ǰ�˶�ģʽΪ����ģʽ
					}
					else  //��һ������ԭ��
					{
						Page3_Data_Stage = 0;  //������0
					}
				}
				else if(Page3_Data_Stage <= 5) //�岽֮��
				{
					Flag_Status_Motion = STATUS_SINGMOD; //��ǰ�˶�ģʽΪ����ģʽ
				}	
			}	
			Page2_Button_Start = 0;			
		}		
		else if(Page1_Button_Recover == 1)  //���¸�λ
		{
			if(Flag_Status_Motion == STATUS_REST) //ֹͣ״̬
			{
				Motor_Restore(); //��λ�����л��ж�ԭ��
			  Page3_Data_Stage = 0; //��������
			}
			Page1_Button_Recover = 0;
		}
		else if(Page2_Button_Stop == 1)
		{
			Motor_Dir = DIR_STOP; //���ֹͣת��	
			Flag_Status_Motion = STATUS_REST; //�˶�״̬Ϊֹͣ
			Page3_Data_Stage = 0; //��������
			Page2_Status_Return = 0;
			Page2_Button_Stop = 0;
		}	
	}

/******************* �ֶ�ģʽ����--�ֶ����ư�ť ****************************/
	if(Flag_PLC_Page == PAGE_FOUR)
	{		
		if(Flag_Status_Motion == STATUS_REST)  //�ھ�ֹ������
		{		
			if(Page4_Button_Left && (Flag_Page4_Button_Left==0)) //��һ�ΰ��²ŷ�����
			{
				if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT)!=1) //��������
				{
					Flag_Page4_Button_Left = 1;
					Motor_Dir = DIR_LEFT; //�������ת��
				}			
			}
			else if((Page4_Button_Left==0) && Flag_Page4_Button_Left)
			{
				Flag_Page4_Button_Left = 0;   //�����ɿ���
				Motor_Dir = DIR_STOP; 	//���ֹͣת��
			}
			
			if(Page4_Button_Right && (Flag_Page4_Button_Right==0))
			{
				if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT)!=1) //�����Ҽ���
				{
					Flag_Page4_Button_Right = 1;
					Motor_Dir = DIR_RIGHT;  //�������ת��
				}	
			}
			else if((Page4_Button_Right==0) && Flag_Page4_Button_Right) //�����ɿ�
			{
				Flag_Page4_Button_Right = 0;  //�����ɿ���
				Motor_Dir = DIR_STOP;  //���ֹͣת��
			}
			
			if(Page1_Button_Recover == 1)  //���¸�λ
			{
				if((Page2_Data_Location>=-0.0f) && (Page2_Data_Location<0.05f)) //����̨��ԭ��
				{
					//δ����
				}
				else  //����̨����ԭ��
				{
					Page2_Status_Return = 1; //����״̬����1
					Motor_Restore(); 				 //��ʼ��λ��������PID��λ�ú��ٶȣ������˵�����λ״̬��������
				}
				Page1_Button_Recover = 0;
			}
		}	
		else  //���ھ�ֹ������
		{
			Page1_Button_Recover = 0; //����Ӧ
		}
		
		if(Page2_Button_Stop == 1) //������ֹͣ��ť
		{
			Motor_Dir = DIR_STOP; //���ֹͣת��	
			Page2_Status_Return = 0; //����״̬����0
			Flag_Status_Motion = STATUS_REST; //�˶�״̬Ϊֹͣ
			Page2_Button_Stop = 0;
		}
	}	
	
/******************* ������ҳ��--���ذ�ť���� 	****************************/
	if(Page2_Button_BackMain == 1)
	{
		Page2_Button_BackMain = 0;		
		Present_Page = 1;	      //�������л�����һҳ��֮���Զ�����������0
		Flag_PLC_Page = 1;  		//PLC������¼��ǰҳ��
		Motor_Dir = DIR_STOP;   //���·��أ����ֹͣ�˶�
		Flag_Status_Motion = STATUS_REST; //�˶�״̬Ϊֹͣ
		Page2_Status_Return = 0; //����״̬����0
		Page3_Data_Stage = 0;   //��������
	}	

/******************* ״ָ̬ʾ�ƴ���  						****************************/	
	if(Page2_Data_RTSpeed > 0)
	{
		Page2_Status_DirLeft = 0;
		Page2_Status_DirRight = 1;		
	}
	else if(Page2_Data_RTSpeed == 0)
	{
		Page2_Status_DirLeft = 0;
		Page2_Status_DirRight = 0;		
	}
	else if(Page2_Data_RTSpeed < 0)
	{
		Page2_Status_DirLeft = 1;			
		Page2_Status_DirRight = 0;		
	}
	//���Ҽ���״̬
	if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == 1)
	{
		Page2_Status_EndLeft = 1;
	}
	else
	{
		Page2_Status_EndLeft = 0;
	}
	if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT) == 1)
	{
		Page2_Status_EndRight = 1;
	}
	else 
	{
		Page2_Status_EndRight = 0;	
	}
	if((Page2_Data_Location>=-0.0f) && (Page2_Data_Location<0.05f)) //����̨��ԭ��
	{
		Page1_Status_Recover = 1;
	}
	else 
	{
		Page1_Status_Recover = 0;	
	}
}

/**
 *���ã����Ʊ�Ƶ����ָ���
 *���̣������ж��ٶȺͷ�����û�иı䣬�ı��˾Ϳ�����ʱ����30ms֮����ָ��
**/
void FreqChg_Control(void)
{
	if((Motor_Dir!=Last_Dir) || (Page4_Data_RunSpeed!=Last_Speed)) //ֹͣ���Լ�ʱ��Ӧ
	{
		TIM_Cmd(TIM2, ENABLE); 	//ʹ�ܶ�ʱ��2
	}
}

/**
 *���ã������λ��ԭ��ĵ�һ���˶�
 *���̣���λ״̬�����һ�����޸�Ƶ��Ϊ50
**/
void Motor_Restore(void)
{	
	Page2_Status_Return = 1;  //���ص���
	
	if(Flag_Init == DOING)  //��ʾ�����ϵ縴λ
	{
		Page4_Data_RunSpeed = 15.0f;  //�ٶ�15mm/s
		
		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //λ��������
		{
			Motor_Dir = DIR_RIGHT; 		//�������ת��
			Flag_Status_Motion = STATUS_TWOSTEP; //��ʾ��ǰ���ڸ�λ�ĵڶ���
			Location_Left_Limit = 1.0f;  //��¼���޵�ǰλ��
		}
		else   //�������޵�����λ��
		{
			Motor_Dir = DIR_LEFT; 				//�������ת��
			Flag_Status_Motion = STATUS_ONESTEP; //��ʾ��ǰ���ڸ�λ�ĵ�һ��
		}		
	}
	else if(Flag_Init == DONE)  //��ʾ�ϵ縴λ�����
	{
		if((Page2_Data_Location>=-0.0f) && (Page2_Data_Location<0.05f)) //����̨��ԭ��
		{
			Page2_Status_Return = 0;  //���ص���
		}
		else   //����̨����ԭ��
		{
			Flag_Status_Motion = STATUS_THREESTEP; //��ʾ��ǰ���ڸ�λ�ĵ�����			
		}
	}
}

/**
 *���ã������жϴ�����
 *note�������ش���
**/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line7))  //����
	{
		if(Flag_Status_Motion == STATUS_ONESTEP) //��ǰΪ��λ״̬��һ��
		{
			Location_Left_Limit = Page2_Data_Location;  //��¼���޵�ǰλ��			
			Flag_Status_Motion = STATUS_TWOSTEP; 				//��λ�����˵ڶ���
			Motor_Dir = DIR_RIGHT;   	//��λ�������˶�
			Page4_Data_RunSpeed = 15.0f;
		}
		else  //����״����������
		{
			Motor_Dir = DIR_STOP; //���ֹͣת��				
			Flag_Status_Motion = STATUS_REST;
			Page3_Data_Stage = 0;  //��������
		}
		EXTI_ClearFlag(EXTI_Line7);		
	}
	
	if(EXTI_GetFlagStatus(EXTI_Line6))  //ԭ��
	{	
		if(Flag_Status_Motion == STATUS_ONESTEP)
		{		
			//��λ��ԭ��ʱ�䶯̬����20mm/�ٶ�/0.2s = TIM5_Count
			TIM5_Count = (u8)(15.0f/Page4_Data_RunSpeed/0.2f); //��ʱ��5������x���жϺ�Ż�ر�					
			TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5���жϼ�ʱ			
		}
		else if(Flag_Status_Motion == STATUS_TWOSTEP)  //û�ڸ�λ״̬������������δ����
		{
			TIM5_Count = 9; //��ʱ��5������1���жϺ�Ż�ر�
			TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5���жϼ�ʱ
			Page4_Data_RunSpeed = 2.0f; 	//��������һС�ξ���
		}	
		EXTI_ClearFlag(EXTI_Line6);
	}
	
	if(EXTI_GetFlagStatus(EXTI_Line5))  //�Ҽ���
	{
		Motor_Dir = DIR_STOP; //���ֹͣת��	
		Flag_Status_Motion = STATUS_REST;	 //��ǰΪ��ֹ״̬	
		EXTI_ClearFlag(EXTI_Line5);
	}
}

void TIM2_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET) 	//����ж�
	{
		if(Last_Dir != Motor_Dir)  //����ı�
		{
			Last_Dir = Motor_Dir;
			switch(Last_Dir)
			{
				case DIR_STOP:
					Motor_Stop();  //���ֹͣת��	
					if(Flag_Init == DOING)  //�����ϵ縴λ
					{
						TIM5_Count = 2; 				//200ms*2֮������жϣ�λ������
						TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5���жϼ�ʱ
					}						
					break;
				case DIR_LEFT:
					Motor_Left();  //�����ת
					break;
				case DIR_RIGHT:
					Motor_Right(); //�����ת
					break;
			}
		}
		else if(Last_Speed != Page4_Data_RunSpeed) //�ٶȸı�
		{	
			Last_Speed = Page4_Data_RunSpeed;
			//������Ϊ�ٶȺ�Ƶ�ʳ����Թ�ϵ������ϵ�����Ϊ4.13
			FREQ_Change_Freq(Last_Speed * 4.13f); //���ı�Ƶ����Ƶ��	
		}	
		TIM_Cmd(TIM2, DISABLE); //ʧ�ܶ�ʱ��2
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 	//����жϱ�־λ
	}
}

void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET) 	//����ж�
	{	
		Increment = TIM4->CNT;
		TIM4->CNT = 0;
		
		//����õ��ٶ�ֵ��Increment*2.5mm/1024����/20ms
		if(Increment == 0 || Increment < 512)
		{
			Page2_Data_RTSpeed = -1.0f * Increment * 2.5f / 2048.0f / 0.02f;  //��ת
			Encoder_Pulse_NUM -= Increment;
		}			
		else if(Increment > 1024)		
		{
			Page2_Data_RTSpeed = (Parameter - Increment) * 2.5f / 2048.0f / 0.02f;  //��ת
			Encoder_Pulse_NUM += (Parameter - Increment);			
		}
		//λ�ã�������*2.5mm/1024 ��λmm
		Page2_Data_Location = Encoder_Pulse_NUM * 2.5 / 2048;		
		Page4_Data_Animation = (int)Page2_Data_Location; //����λ��		
		if(Flag_Status_Motion == STATUS_AUTOMOD) //�Զ�ģʽ�˶�״̬��
		{
			if(Page3_Data_Stage <= 3) //ǰ�Ķ�������
			{			
				if(Encoder_Pulse_NUM >= 32768 * Page3_Data_Stage)  //�˶�������40mm������
				{
					Page3_Data_Stage++;  	 //������1�����ӵ�4						
				}
			}		
			if(Page3_Data_Stage <= 4)
			{
				PID_Control_SPD(160.03f, Present_Data_Speed[Page3_Data_Stage]); //���������ٶȣ�����������ж�			
			}		
			else if(Page3_Data_Stage == 5)  //��ԭ��
			{
				PID_Control_SPD(0.03f, Present_Data_Speed[Page3_Data_Stage]); //���������ٶȣ�����������ж�	
			}				
		}				
		else if(Flag_Status_Motion == STATUS_TWOSTEP) //�ڸ�λ�ĵڶ���״̬��
		{
			if(Location_Left_Limit != 0)  //��������˵����������
			{
				if((Page2_Data_Location-Location_Left_Limit) > 70.0f) 
				{
					Page4_Data_RunSpeed = 2.0f;   //��������	
					Location_Left_Limit = 0.0f;   //��������޺�λ�ù���
				}
			}
		}
		else if(Flag_Status_Motion == STATUS_THREESTEP) //�����˸�λ��ť
		{
			PID_Control_SPD(0.03f, 15.0f); //���������ٶ�
		}
		else if(Flag_Status_Motion == STATUS_SINGMOD)
		{
			if(Page3_Data_Stage <= 4)  //����С�ڵ���4
			{
				PID_Control_SPD(Page3_Data_Stage*40.0f+0.03f, Page3_Data_Speed);				
			}
			else if(Page3_Data_Stage == 5)
			{
				PID_Control_SPD(0.03f, Page2_Data_BKSpeed);								
			}
		}
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 	    //����жϱ�־λ
}
	
void TIM5_IRQHandler(void)
{		
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) 	//����ж�
	{
		TIM5_Count--;
		if(TIM5_Count == 0)
		{				
			TIM_Cmd(TIM5, DISABLE);   //ʧ�ܶ�ʱ��5			
			if((Flag_Init==DOING) && (Flag_Status_Motion==STATUS_REST))  	  //�ϵ縴λ
 			{
				Encoder_Pulse_NUM = 0;  	//λ�ù���
				Flag_Init = DONE;    			//��־�ϵ縴λ���
				//��ť���
				Page1_Button_Recover = 0; //��λ��ť 
				Page2_Button_Stop = 0;    //ֹͣ��ť
				Page1_Button_Auto = 0;    //�Զ�ģʽ�л���ť
				Page1_Button_Single = 0;  //����ģʽ�л���ť
			  Page1_Button_Manual = 0;  //�ֶ�ģʽ�л���ť
			}					
			if(Flag_Status_Motion == STATUS_ONESTEP) //��ǰ���ڵ�һ��
			{
				Motor_Dir = DIR_RIGHT; 		//�������ת��					
				Page4_Data_RunSpeed = 2.0f;			
				Flag_Status_Motion = STATUS_TWOSTEP; //����ڶ�����λ
			}
			else if(Flag_Status_Motion == STATUS_TWOSTEP) //�ﵽ���ĵ�
			{
				Motor_Dir = DIR_STOP; 		//���ֹͣת��
				Flag_Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ				
				Page3_Data_Stage = 0; 	//�Զ�ģʽ�Ķ�����Ϊ0��
				Page2_Status_Return = 0; //����״̬����0		
			}		
			if(Motor_Dir == DIR_STOP) //����������200ms����Ȼ�Ǿ�ֹ״̬
			{
				if(Flag_Status_Motion == STATUS_SINGMOD)
				{
					Flag_Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ
					if(Page3_Data_Stage == 5)
					{
						Page3_Data_Stage = 0;
					}
					Motor_Dir = DIR_STOP;
				}
				else if(Flag_Status_Motion == STATUS_THREESTEP)
				{
					Flag_Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ
					Page3_Data_Stage = 0; //�Զ�ģʽ�Ķ�����Ϊ0��
					Page2_Status_Return = 0; //����״̬����
				}
				else if(Flag_Status_Motion == STATUS_AUTOMOD)
				{
					if((Page3_Data_Stage==4) && (Page2_Status_Wait==0)) //˵����Ҫ��ʱ3s
					{
						Page2_Status_Wait = 1;	//�ȴ�ָʾ����
						TIM_Cmd(TIM5, ENABLE);  //ʹ�ܶ�ʱ��5
						TIM5_Count = 15;   			//3s��ʱ
					}
					else if((Page3_Data_Stage==4) && (Page2_Status_Wait==1))
					{
						Page2_Status_Wait = 0;	//�ȴ�ָʾ����
						Page2_Status_Return = 1;//����״̬����
						Page3_Data_Stage++; 		//�����						
					}			
					else if(Page3_Data_Stage == 5)  //��ԭ�������
					{
						Flag_Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ	
						Page3_Data_Stage = 0; //�Զ�ģʽ�Ķ�����Ϊ0��
						Page2_Status_Return = 0; //����״̬����
					}
				}
			}			
		}		
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 	 //����жϱ�־λ
}

