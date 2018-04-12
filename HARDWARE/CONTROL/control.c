#include "control.h"
#include "modbus_slave.h"
#include "modbus_master.h"
#include "led.h"
#include "exti.h"
#include "encoder.h"
#include "time.h"
#include "delay.h"
#include "pid.h"
#include "dma.h"
#include "string.h"

long  Encoder_Pulse_NUM = 0; //��¼�������������
vu8   Flag_Init = DOING; 	 //�ϵ縴λ��־λ��doing��־�����ϵ縴λ��done��ʾ�ϵ縴λ���
vu8   Status_Motion = STATUS_REST; //��ǰΪ��ֹ״̬
vu8   TIM5_Count = 0; 		 //��ʱ��5�ļ�ʱ����
float Last_Speed = 0.0f;     //��һ�ε��ٶ�ֵ������͵�ǰ���ٶȲ���Ⱦ�Ҫ�ı��ٶ�
u8    Motor_Dir = DIR_STOP;  //�����ǰ����Ĭ��ֹͣ
u8    Last_Dir = DIR_STOP;   //����ϴεķ���Ĭ��ֹͣ
float Location_Left_Limit = 0.0f;   //�ϵ縴λ����������ޣ��ͼ�¼����λ��
float Present_Data_Speed[5] = {0};  //��¼�Զ�ģʽ�°�������ʱ������ٶȣ��˶������и����ٶ���Ч

vu8   order_type = ORDER_NONE;   //��Ƶ���������ͣ�Ĭ��������
vu8   communicate = SEND2;   	 //modbus����Ĭ���Ƿ���
/**
 *���ã�ģʽ�л����߼����Ʋ���
**/
void Mode_Control(void)
{	
	static u8 Flag_Button_Left = 0; 	//��ťδ���µı�־
	static u8 Flag_Button_Right = 0;    //�Ұ�ťδ���µı�־
	static u8 Flag_Mode = MODE_SELECT;  //����Ϊģʽѡ��
	
	if(Flag_Mode == MODE_SELECT)  //ģʽѡ��
	{	
		if(Status_Motion == STATUS_REST)  //�ھ�ֹ������
		{
			if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //����̨��ԭ��
			{				
				Button_Recover = 0;  //��ԭ�㰴�¸�λ����Ӧ
				if(Button_Auto == 1) //�����Զ�ģʽ����
				{
					Present_Mode = MODE_AUTO; 	//�����Զ�ģʽ
					Flag_Mode = MODE_AUTO;  	//��¼�Զ�ģʽ��־
					Button_Auto = 0;
				}
				else if(Button_Single == 1)  //���µ���ģʽ����
				{
					Present_Mode = MODE_SINGLE;	 //���뵥��ģʽ
					Flag_Mode = MODE_SINGLE;  	 //��¼����ģʽ��־
					Button_Single = 0;				
				}
			}
			else    //����̨����ԭ��
			{				
				if(Button_Recover == 1)  //��λ��ť���£������λ
				{
					Motor_Restore();  	 //�����λ����
					Button_Recover = 0;
				}			
				Button_Auto = 0;   //�����Զ�ģʽ��ť������Ӧ
				Button_Single = 0; //���µ���ģʽ��ť������Ӧ			
			}			
			//�ڲ���ԭ�㣬������Ӧ�ֶ�ģʽ
			if(Button_Manual == 1)  //�����ֶ�ģʽ
			{ 			
				Present_Mode = 4;	//�����ֶ�ģʽ
				Flag_Mode = 4;  	//��¼�ֶ�ģʽ��־
				Button_Manual = 0;
			}		
		}
		else  //�ڸ�λ�����У����°�������ť����Ӧ
		{
			Button_Auto = 0; //�����Զ�ģʽ��ť������Ӧ
			Button_Single = 0;//���µ���ģʽ��ť������Ӧ
			Button_Manual = 0;//�����ֶ�ģʽ��ť������Ӧ
			Button_Recover = 0; //���¸�λ��ť������Ӧ
		}		
		if(Button_Stop == 1) //�����˳���ť
		{
			Motor_Dir = DIR_STOP; //��������Ϊֹͣ	
			Status_Return = 0; 	  //����״̬����0
			Status_Motion = STATUS_REST; //�˶�״̬Ϊֹͣ
			Button_Stop = 0;
		}
	}
	if(Flag_Mode == MODE_AUTO)  //�Զ�ģʽ
	{
		if(Button_Start == 1)  //����������ť
		{
			if(Status_Motion == STATUS_REST) //ֹͣ״̬
			{
				if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //����̨��ԭ��
				{
					if((Data_Speed1>0) && (Data_Speed2>0) && \
					   (Data_Speed3>0) && (Data_Speed4>0) && \
					   (Data_BKSpeed>0) ) //����ٶȾ�����0
					{
						Present_Data_Speed[0] = Data_Speed1;  //��¼��ǰ�����һ���ٶȣ��˶������и����ٶ���Ч				
						Present_Data_Speed[1] = Data_Speed2;  //��¼��ǰ����Ķ����ٶȣ��˶������и����ٶ���Ч				
						Present_Data_Speed[2] = Data_Speed3;  //��¼��ǰ����������ٶȣ��˶������и����ٶ���Ч				
						Present_Data_Speed[3] = Data_Speed4;  //��¼��ǰ������Ķ��ٶȣ��˶������и����ٶ���Ч
						Present_Data_Speed[4] = Data_BKSpeed; //��¼��ǰ����ķ����ٶȣ��˶������и����ٶ���Ч
						Data_Stage = 1;  //��ǰ����Ϊ��һ��	
						Motor_Dir = DIR_RIGHT; //��������˶�
						Data_RunSpeed = Present_Data_Speed[0];//��ʼ�ٶ�
						Status_Motion = STATUS_AUTOMOD;  //��־��ǰ�˶�״̬Ϊ�Զ��˶�״̬
					}
				}	
			}	
			Button_Start = 0;
		}
		else if(Button_Recover == 1)  //���¸�λ
		{
			if(Status_Motion == STATUS_REST) //ֹͣ״̬
			{			
				Motor_Restore(); 					//��λ
			}
			Button_Recover = 0;
		}
		else if(Button_Stop == 1) //����ֹͣ��ť
		{
			Motor_Dir = DIR_STOP; //��������Ϊֹͣ	
			Status_Return = 0; //���ص���
			Status_Wait = 0;   //�ȴ�ָʾ����
			Status_Motion = STATUS_REST; //��־��ǰ״̬Ϊ��ֹ
			Data_Stage = 0;    //��������
			Button_Stop = 0;  
		}	
	}
	if(Flag_Mode == MODE_SINGLE)  //����ģʽ
	{
		if(Button_Start == 1) //����������ť
		{
			if(Status_Motion == STATUS_REST)  //��ǰ״̬Ϊ��ֹ״̬
			{
				Data_Stage++;  //������һ
				if(Data_Stage == 1) //����1˵����һ�ΰ���������ť
				{
					if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //����̨��ԭ��
					{
						Motor_Dir = DIR_RIGHT; //�������ת��
						Data_RunSpeed = Data_Speed;
						Status_Motion = STATUS_SINGMOD; //��ǰ״̬Ϊ����״̬
					}
					else  //��һ������ԭ��
					{
						Data_Stage = 0;  //������0
					}
				}
				else if(Data_Stage <= 4) //�Ķ�֮��
				{
					Motor_Dir = DIR_RIGHT; //�������ת��
					Data_RunSpeed = Data_Speed;
					Status_Motion = STATUS_SINGMOD; //��ǰ״̬Ϊ����״̬
				}	
				else if(Data_Stage == 5)
				{
					Status_Return = 1;
					Motor_Dir = DIR_LEFT; //�������ת��
					Data_RunSpeed = Data_BKSpeed;   //���÷����ٶ�
					Status_Motion = STATUS_ONESTEP; //��ǰ״̬Ϊ����״̬
				}
			}	
			Button_Start = 0;			
		}		
		else if(Button_Recover == 1)  //���¸�λ
		{
			if(Status_Motion == STATUS_REST) //��ǰ״̬Ϊֹͣ״̬
			{
				Motor_Restore(); //��λ�����л��ж�ԭ��
			    Data_Stage = 0;  //��������
			}
			Button_Recover = 0;
		}
		else if(Button_Stop == 1)
		{
			Motor_Dir = DIR_STOP; //��������Ϊֹͣ
			Status_Motion = STATUS_REST; //��ǰ״̬Ϊ��ֹ״̬
			Data_Stage = 0; 	//��������
			Status_Return = 0;  //���ص���
			Button_Stop = 0;
		}	
	}
	if(Flag_Mode == MODE_MANUAL)  //�ֶ�ģʽ
	{		
		if(Status_Motion == STATUS_REST)  //��ǰ״̬Ϊ��ֹ״̬
		{		
			if(Button_Left && (Flag_Button_Left==0)) //�������˶���ť
			{
				if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT)!=1) //��������
				{
					Flag_Button_Left = 1;  //�������±�־λ��1
					Motor_Dir = DIR_LEFT;  //��������Ϊ��ת
				}			
			}
			else if((Button_Left==0) && Flag_Button_Left) //�����ɿ�
			{
				Flag_Button_Left = 0;   //�����ɿ���־λ��0
				Motor_Dir = DIR_STOP; 	//��������Ϊֹͣ
			}
			
			if(Button_Right && (Flag_Button_Right==0)) //�������˶���ť
			{
				if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT)!=1) //�����Ҽ���
				{
					Flag_Button_Right = 1;  //�������±�־λ��1
					Motor_Dir = DIR_RIGHT;  //��������Ϊ��ת
				}	
			}
			else if((Button_Right==0) && Flag_Button_Right) //�����ɿ�
			{
				Flag_Button_Right = 0;  //�����ɿ���־λ��0
				Motor_Dir = DIR_STOP;   //��������Ϊֹͣ
			}
			
			if(Button_Recover == 1)  //���¸�λ
			{
				if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //����̨��ԭ��
				{
					//δ����
				}
				else  //����̨����ԭ��
				{
					Status_Return = 1;  //����״̬����1
					Motor_Restore(); 	//��ʼ��λ
				}
				Button_Recover = 0;
			}
		}	
		else  //��ǰ״̬�Ǿ�ֹ״̬
		{
			Button_Recover = 0; //��λ��ť��������Ӧ
		}
		
		if(Button_Stop == 1) //������ֹͣ��ť
		{
			Motor_Dir = DIR_STOP; //��������Ϊֹͣ
			Status_Return = 0; 	  //����״̬����0
			Status_Motion = STATUS_REST; //�˶�״̬Ϊ��ֹ״̬
			Button_Stop = 0;
		}
	}	
	
	if(Button_BackMain == 1)   //�˳���ť����
	{
		Button_BackMain = 0;		
		Present_Mode = MODE_SELECT;	  //����ģʽѡ��
		Flag_Mode = MODE_SELECT;  	  //��¼ģʽѡ���־
		Motor_Dir = DIR_STOP;   	  //��������Ϊֹͣ
		Status_Motion = STATUS_REST;  //�˶�״̬Ϊ��ֹ״̬
		Status_Return = 0; 	//����״̬����
		Data_Stage = 0;   	//��������
	}	
	
	//״ָ̬ʾ�ƴ���	
	if(Data_RTSpeed > 0)
	{
		Status_DirLeft = 0;  //�������
		Status_DirRight = 1; //�ҷ������		
	}
	else if(Data_RTSpeed == 0)
	{
		Status_DirLeft = 0;  //�������
		Status_DirRight = 0; //�ҷ������		
	}
	else if(Data_RTSpeed < 0)
	{
		Status_DirLeft = 1;	 //�������		
		Status_DirRight = 0; //�ҷ������		
	}
	//���Ҽ���״̬
	if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == 1)
	{
		Status_EndLeft = 1; //���޵���
	}
	else
	{
		Status_EndLeft = 0; //���޵���
	}
	if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT) == 1)
	{
		Status_EndRight = 1; //�Ҽ��޵���
	}
	else 
	{
		Status_EndRight = 0; //�Ҽ��޵���	
	}
	if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //����̨��ԭ��
	{
		Status_Recover = 1;
	}
	else 
	{
		Status_Recover = 0;	
	}
}

/**
 *���ã����Ʊ�Ƶ����ָ���
 *���̣������ж��ٶȺͷ�����û�иı䣬�ı��˾Ϳ�����ʱ����30ms֮����ָ��
**/
void FreqChg_Control(void)
{
	//����Ƶ����ָ��
	if(communicate == SEND2)
	{
		if(Last_Dir != Motor_Dir)  //����ı�
		{
			__disable_irq() ; //�ر����ж�
			switch(Motor_Dir)
			{
				case DIR_STOP:
					Motor_Stop();  //���ֹͣת��	
					if(Flag_Init == DOING)  //�����ϵ縴λ
					{
						TIM5_Count = 2; 	//200ms*2֮������жϣ�λ������
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
			communicate = WAITING; //�ȴ�30ms
			TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
			__enable_irq() ; //�����ж�
		}
		else if(Last_Speed != Data_RunSpeed) //�ٶȸı�
		{	
			__disable_irq() ; //�ر����ж� 
			//������Ϊ�ٶȺ�Ƶ�ʳ����Թ�ϵ������ϵ�����Ϊ4.13
			FREQ_Change_Freq(Data_RunSpeed * 4.13f); //���ı�Ƶ����Ƶ��
			communicate = WAITING; //�ȴ�30ms			
			TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
			__enable_irq() ; //�����ж�
		}	
	}
}

/**
 *���ã������λ��ԭ��ĵ�һ���˶�
 *���̣���λ״̬�����һ�����޸�Ƶ��Ϊ50
**/
//void Motor_Restore(void)
//{	
//	Status_Return = 1;  //���ص���
//	
//	if(Flag_Init == DOING)  //��ʾ�����ϵ縴λ
//	{
//		Data_RunSpeed = 15.0f;  //�ٶ�15mm/s
//		
//		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //λ��������
//		{
//			Motor_Dir = DIR_RIGHT; 			//��������Ϊ��ת
//			Status_Motion = STATUS_TWOSTEP; //�˶�״̬Ϊ��λ�ڶ���״̬
//			Location_Left_Limit = 1.0f;  	//��¼���޵�ǰλ��
//		}
//		else   //�������޵�����λ��
//		{
//			Motor_Dir = DIR_LEFT; 		//��������Ϊ��ת
//			Status_Motion = STATUS_ONESTEP; //�˶�״̬Ϊ��λ��һ��״̬
//		}		
//	}
//	else if(Flag_Init == DONE)  //��ʾ�ϵ縴λ�����
//	{
//		if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //����̨��ԭ�� [0.0mm,0.05mm)
//		{
//			Status_Return = 0;  //���ص���
//		}
//		else   //����̨����ԭ��
//		{
//			Status_Motion = STATUS_THREESTEP; //�˶�״̬Ϊ��λ������״̬			
//		}
//	}
//}

void Motor_Restore(void)
{	
	Status_Return = 1;  	//���ص���	
	if(Flag_Init == DOING)  //��ʾ�����ϵ縴λ
	{
		Data_RunSpeed = 15.0f;  //�ٶ�15mm/s	
		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //λ��������
		{
			Motor_Dir = DIR_RIGHT; 			//��������Ϊ��ת
			Status_Motion = STATUS_TWOSTEP; //�˶�״̬Ϊ��λ�ڶ���״̬
			Location_Left_Limit = 1.0f;  	//��¼���޵�ǰλ��
		}
		else   //�������޵�����λ��
		{
			Motor_Dir = DIR_LEFT; 		//��������Ϊ��ת
			Status_Motion = STATUS_ONESTEP; //�˶�״̬Ϊ��λ��һ��״̬
		}	
	}	
	else if(Flag_Init == DONE)  //��ʾ�ϵ縴λ�����
	{
		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //λ��������
		{
			Data_RunSpeed = 15.0f;  //�ٶ�15mm/s
			Motor_Dir = DIR_RIGHT; 	//��������Ϊ��ת
			Status_Motion = STATUS_TWOSTEP; //�˶�״̬Ϊ��λ�ڶ���״̬
		}
		else  //�������޵�����λ��
		{
			if(Data_Location <= -10.0f)  //λ��С��-10mm
			{
				Motor_Dir = DIR_RIGHT; 			//��������Ϊ��ת
				Data_RunSpeed = 15.0f;  		//�ٶ�15mm/s
				Status_Motion = STATUS_TWOSTEP; //�˶�״̬Ϊ��λ�ڶ���״̬
			}
			else if(Data_Location <= -4.0f)
			{
				Motor_Dir = DIR_RIGHT; 			//��������Ϊ��ת
				Data_RunSpeed = 2.0f;  			//�ٶ�2mm/s	
				Status_Motion = STATUS_TWOSTEP; //�˶�״̬Ϊ��λ�ڶ���״̬
			}
			else if(((Data_Location<-0.1f) && (Data_Location>-4.0f))) //��ξ��붨ʱ���ƣ����ػ��⵽������
			{
				Motor_Dir = DIR_LEFT; 			//��������Ϊ��ת
				Data_RunSpeed = 15.0f;  		//�ٶ�15mm/s	
				Status_Motion = STATUS_ONESTEP; //�˶�״̬Ϊ��λ��һ��״̬
				TIM5_Count = 5; //��ʱ��5������5���жϺ�Ż�ر�					
				TIM_Cmd(TIM5, ENABLE); 		//ʹ�ܶ�ʱ��5			
			}
			else if((Data_Location>0.1f) && (Data_Location<5.0f))  //��ξ��붨ʱ���ƣ����ػ��⵽������
			{
				Motor_Dir = DIR_LEFT; 			//��������Ϊ��ת
				Data_RunSpeed = 15.0f;  		//�ٶ�15mm/s			
				Status_Motion = STATUS_ONESTEP; //�˶�״̬Ϊ��λ��һ��״̬
				TIM5_Count = 6; //��ʱ��5������x���жϺ�Ż�ر�					
				TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5
			}
			else if(Data_Location >= 5.0f)
			{
				Motor_Dir = DIR_LEFT; 			//��������Ϊ��ת
				Data_RunSpeed = 15.0f;  		//�ٶ�15mm/s
				Status_Motion = STATUS_ONESTEP; //�˶�״̬Ϊ��λ��һ��״̬
			}
			else  //��ԭ��λ��
			{
				Status_Return = 0;  //���ص���
			}
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
		if(Status_Motion == STATUS_ONESTEP) //�˶�״̬Ϊ��λ��һ��״̬
		{
			Location_Left_Limit = Data_Location; //��¼���޵�ǰλ��			
			Status_Motion = STATUS_TWOSTEP; //�˶�״̬Ϊ��λ�ڶ���״̬
			Motor_Dir = DIR_RIGHT;   	//��������Ϊ��ת
			Data_RunSpeed = 15.0f;
		}
		else  //����״����������
		{
			Motor_Dir = DIR_STOP; //��������Ϊֹͣ				
			Status_Motion = STATUS_REST;
			Data_Stage = 0;  //��������
		}
		EXTI_ClearFlag(EXTI_Line7);		
	}
	if(EXTI_GetFlagStatus(EXTI_Line6))  //ԭ��
	{	
		if(Status_Motion == STATUS_ONESTEP) //�˶�״̬Ϊ��λ��һ��״̬
		{		
			//��λ��ԭ��ʱ�䶯̬����20mm/�ٶ�/0.2s = TIM5_Count
			TIM5_Count = (u8)(15.0f/Data_RunSpeed/0.2f); //��ʱ��5������x���жϺ�Ż�ر�					
			TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5			
		}
		else if(Status_Motion == STATUS_TWOSTEP)  //�˶�״̬Ϊ��λ�ڶ���״̬
		{
			TIM5_Count = 9; //��ʱ��5������1���жϺ�Ż�ر�
			TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5���жϼ�ʱ
			Data_RunSpeed = 2.0f; 	//��������һС�ξ���
		}	
		EXTI_ClearFlag(EXTI_Line6);
	}	
	if(EXTI_GetFlagStatus(EXTI_Line5)) //�Ҽ���
	{
		Motor_Dir = DIR_STOP; 	//��������Ϊֹͣ	
		Status_Motion = STATUS_REST; //��ǰΪ��ֹ״̬	
		EXTI_ClearFlag(EXTI_Line5);
	}
}

void TIM2_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET) 	//����ж�
	{	
		if(Last_Dir != Motor_Dir) 
		{
			Last_Dir = Motor_Dir; //�����ط���ͨ�����
		}
		else if(Last_Speed != Data_RunSpeed)
		{
			Last_Speed = Data_RunSpeed;
		}
		communicate = SEND2;	 //���Է���ָ��
//		communicate = RECEIVE2; //ת��Ϊ����״̬
		TIM_Cmd(TIM2, DISABLE); //ʧ�ܶ�ʱ��2
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 	//����жϱ�־λ
	}
}

//�������֮�����
void FreqRev_Deal(void)
{
	u8 i;
	if(communicate == RECEIVE2) //����״̬
	{
		for(i = 0; i < 3; i++)
		{
			if(Receive_485_flag == 1)  //485���յ�����
			{
				Modbus2_Parse(); //�����Ƶ�����ص�����
				//�������
				memset(SendBuff_485, 0, LEN_SEND_485);
				memset(ReceBuff_485, 0, LEN_RECV_485);
				break; //����ѭ��
			}
			else if(Receive_485_flag == 0)//485δ���յ��ظ����ݣ��ط�
			{
				if(Last_Dir != Motor_Dir)  //����ı�
				{
					switch(Motor_Dir)
					{
						case DIR_STOP:
							Motor_Stop();  //���ֹͣת��						
							break;
						case DIR_LEFT:
							Motor_Left();  //�����ת
							break;
						case DIR_RIGHT:
							Motor_Right(); //�����ת
							break;
					}
				}
				else if(Last_Speed != Data_RunSpeed) //�ٶȸı�
				{	
					FREQ_Change_Freq(Data_RunSpeed * 4.13f); //���ı�Ƶ����Ƶ��	
				}
				delay_ms(25); //�ط�֮����ʱ25ms				
			}
		}
		if(i == 3) //�������ط�
		{
			if(Receive_485_flag == 1)  //485���յ�����
			{
				Modbus2_Parse(); //�����Ƶ�����ص�����
				//�������
				memset(SendBuff_485, 0, LEN_SEND_485);
				memset(ReceBuff_485, 0, LEN_RECV_485);
			}
			else 
			{
				//����������
				Data_Error = 5;   //��·����
				Status_Alarm = 0; //ͨ�ű���
			}	
		}
		
		//������ݣ�������
		if(Last_Dir != Motor_Dir) 
		{
			Last_Dir = Motor_Dir; //�����ط���ͨ�����
		}
		else if(Last_Speed != Data_RunSpeed)
		{
			Last_Speed = Data_RunSpeed;
		}
		communicate = SEND2;	 //���Է���ָ��			
		Receive_485_flag = 0;	 //������ձ�־λ	
	}
}

void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET) 	//����ж�
	{	
		Increment = TIM4->CNT; //��ȡ������������
		TIM4->CNT = 0;  //TIM4��������ֵ��0
		
		//����õ��ٶ�ֵ��Increment*2.5mm/1024����/20ms
		if(Increment < 1024)
		{
			Data_RTSpeed = -1.0f * Increment * 2.5f / 2048.0f / 0.02f;  //��ת
			Encoder_Pulse_NUM -= Increment;
		}			
		else if(Increment > 1024)		
		{
			Data_RTSpeed = (Parameter - Increment) * 2.5f / 2048.0f / 0.02f;  //��ת
			Encoder_Pulse_NUM += (Parameter - Increment);			
		}
		//λ�ã�������*2.5mm/1024 ��λmm
		Data_Location = Encoder_Pulse_NUM * 2.5 / 2048;		
		Data_Animation = (int)Data_Location; //����λ��		
		if(Status_Motion == STATUS_AUTOMOD) //�Զ�ģʽ�˶�״̬��
		{
			if(Data_Stage <= 3) //ǰ�Ķ�������
			{			
				if(Encoder_Pulse_NUM >= 32768 * Data_Stage)  //�˶�������40mm������
				{
					Data_Stage++;  	 //������1�����ӵ�4
					Data_RunSpeed = Present_Data_Speed[Data_Stage-1]; //����Ϊ1ʱȡPresent_Data_Speed[0]��Ϊ��ǰ����					
				}
			}		
			if(Data_Stage == 4)  //���Ķ�������
			{
				if(Encoder_Pulse_NUM >= (32768 * 3 + 32700))  //����ָ��λ��
				{
					Data_Stage++; 
					Status_Wait = 1;		//�ȴ�ָʾ����
					Motor_Dir = DIR_STOP; 	//���ֹͣת��			
					TIM5_Count = 15; 		//��ʱ��5������15���жϣ���3s��
					TIM_Cmd(TIM5, ENABLE); 	//ʹ�ܶ�ʱ��5
				}
				else if(Encoder_Pulse_NUM >= (32768 * 3 + 26624))  //�˶���160mmǰ��Ȧ������
				{
					Data_RunSpeed = 2.0f; //����ǰ��
				}	
//				PID_Control_SPD(160.03f, Present_Data_Speed[Data_Stage-1]); //PID����			
			}		
//			else if(Data_Stage == 5)  //��ԭ��
//			{
//				PID_Control_SPD(0.03f, Present_Data_Speed[Data_Stage-1]); //PID����	
//			}				
		}				
		else if(Status_Motion == STATUS_TWOSTEP) //�ڸ�λ�ĵڶ���״̬��
		{
			if(Flag_Init == DOING)  //��ǰΪ�ϵ縴λ״̬
			{
				if(Location_Left_Limit != 0)  //��������˵����������
				{
					if((Data_Location-Location_Left_Limit) > 70.0f) 
					{
						Data_RunSpeed = 2.0f;   //��������	
						Location_Left_Limit = 0.0f;   //��������޺�λ�ù���
					}
				}
			}
			else if(Flag_Init == DONE) //������λ��ɣ����Եõ���ȷ����
			{
				if((Data_Location>-17.0f) && (Data_Location<-0.4f)) //��λλ����-17mm�󣬽��ٵ�2mm/s
				{
					Data_RunSpeed = 2.0f; 	//��������һС�ξ���			
				}
			}
		}
//		else if(Status_Motion == STATUS_THREESTEP) //�����˸�λ��ť
//		{
//			PID_Control_SPD(0.03f, 15.0f); //���������ٶ�
//		}
		else if(Status_Motion == STATUS_SINGMOD)
		{
			if(Data_Stage <= 4)  //����С�ڵ���4
			{
				if(Encoder_Pulse_NUM >= (32768 * (Data_Stage-1) + 32700))  //����ָ��λ��
				{
					Motor_Dir = DIR_STOP; 	//���ֹͣת��
					Status_Motion = STATUS_REST;					
				}
				else if(Encoder_Pulse_NUM >= (32768 * (Data_Stage-1) + 26624))  //�˶���160mmǰ��Ȧ������
				{
					Data_RunSpeed = 2.0f; //����ǰ��
				}
//				PID_Control_SPD(Data_Stage*40.0f+0.03f, Data_Speed); //PID����				
			}
//			else if(Data_Stage == 5)
//			{
//				Motor_Dir = DIR_LEFT; 			//�������ת��
//				Status_Motion = STATUS_ONESTEP; //��ԭ��תΪ��λ����
//				Data_RunSpeed = Data_BKSpeed;   //���÷����ٶ�
////				PID_Control_SPD(0.03f, Data_BKSpeed); //PID����								
//			}
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
			if((Flag_Init==DOING) && (Status_Motion==STATUS_REST))  	  //�ϵ縴λ
 			{
				Encoder_Pulse_NUM = 0; //λ�ù���
				Flag_Init = DONE;   //��־�ϵ縴λ���
				//��ť���
				Button_Recover = 0; //��λ��ť 
				Button_Stop = 0;    //ֹͣ��ť
				Button_Auto = 0;    //�Զ�ģʽ�л���ť
				Button_Single = 0;  //����ģʽ�л���ť
			    Button_Manual = 0;  //�ֶ�ģʽ�л���ť
			}					
			if(Status_Motion == STATUS_ONESTEP) //��ǰ���ڵ�һ��
			{
				Motor_Dir = DIR_RIGHT; 		//��������Ϊ��ת				
				Data_RunSpeed = 2.0f;			
				Status_Motion = STATUS_TWOSTEP; //����ڶ�����λ
			}
			else if(Status_Motion == STATUS_TWOSTEP) //�ﵽ���ĵ�
			{
				Motor_Dir = DIR_STOP; 		//��������Ϊֹͣ
				Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ				
				Data_Stage = 0;    //�Զ�ģʽ�Ķ�����Ϊ0��
				Status_Return = 0; //����״̬����0				
			}	
			else if(Status_Motion == STATUS_AUTOMOD)
			{
				Status_Wait = 0;
				Status_Return = 1; //����״̬����1
				Motor_Dir = DIR_LEFT; //�������ת��
				Status_Motion = STATUS_ONESTEP; //��ԭ��תΪ��λ��������			
				Data_RunSpeed = Present_Data_Speed[Data_Stage-1]; //���÷����ٶ�
			}				
//			if(Motor_Dir == DIR_STOP) //����������200ms����Ȼ�Ǿ�ֹ״̬
//			{
//				if(Status_Motion == STATUS_SINGMOD)
//				{
//					Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ
//					if(Data_Stage == 5)
//					{
//						Data_Stage = 0;
//					}
//					Motor_Dir = DIR_STOP;
//				}
////				else if(Status_Motion == STATUS_THREESTEP)
////				{
////					Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ
////					Data_Stage = 0; //�Զ�ģʽ�Ķ�����Ϊ0��
////					Status_Return = 0; //����״̬����
////				}
//				else if(Status_Motion == STATUS_AUTOMOD)
//				{
//					if((Data_Stage==4) && (Status_Wait==0)) //˵����Ҫ��ʱ3s
//					{
//						Status_Wait = 1;	//�ȴ�ָʾ����
//						TIM_Cmd(TIM5, ENABLE);  //ʹ�ܶ�ʱ��5
//						TIM5_Count = 15;   		//3s��ʱ
//					}
//					else if((Data_Stage==4) && (Status_Wait==1))
//					{
//						Status_Wait = 0;	//�ȴ�ָʾ����
//						Status_Return = 1;	//����״̬����
//						Data_Stage++; 		//�����	
//						
//						Motor_Dir = DIR_LEFT; 			//�������ת��
//						Status_Motion = STATUS_ONESTEP; //��ԭ��תΪ��λ����
//						Data_RunSpeed = Present_Data_Speed[Data_Stage-1]; //���÷����ٶ�[4]
//					}			
////					else if(Data_Stage == 5)  //��ԭ�������
////					{
////						Status_Motion = STATUS_REST; //�˶�״̬��Ϊֹͣ	
////						Data_Stage = 0; 	//�Զ�ģʽ�Ķ�����Ϊ0��
////						Status_Return = 0;  //����״̬����
////					}
//				}
//			}			
		}		
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 	 //����жϱ�־λ
}

