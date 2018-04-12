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

long  Encoder_Pulse_NUM = 0; //记录编码器脉冲个数
vu8   Flag_Init = DOING; 	 //上电复位标志位，doing标志正在上电复位，done表示上电复位完成
vu8   Status_Motion = STATUS_REST; //当前为静止状态
vu8   TIM5_Count = 0; 		 //定时器5的计时次数
float Last_Speed = 0.0f;     //上一次的速度值，如果和当前的速度不相等就要改变速度
u8    Motor_Dir = DIR_STOP;  //电机当前方向，默认停止
u8    Last_Dir = DIR_STOP;   //电机上次的方向，默认停止
float Location_Left_Limit = 0.0f;   //上电复位如果到达左极限，就记录左极限位置
float Present_Data_Speed[5] = {0};  //记录自动模式下按下启动时的五个速度，运动过程中更改速度无效

vu8   order_type = ORDER_NONE;   //变频器命令类型，默认无命令
vu8   communicate = SEND2;   	 //modbus主机默认是发送
/**
 *作用：模式切换，逻辑控制部分
**/
void Mode_Control(void)
{	
	static u8 Flag_Button_Left = 0; 	//左按钮未按下的标志
	static u8 Flag_Button_Right = 0;    //右按钮未按下的标志
	static u8 Flag_Mode = MODE_SELECT;  //开机为模式选择
	
	if(Flag_Mode == MODE_SELECT)  //模式选择
	{	
		if(Status_Motion == STATUS_REST)  //在静止过程中
		{
			if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //工作台在原点
			{				
				Button_Recover = 0;  //在原点按下复位无响应
				if(Button_Auto == 1) //按下自动模式按键
				{
					Present_Mode = MODE_AUTO; 	//进入自动模式
					Flag_Mode = MODE_AUTO;  	//记录自动模式标志
					Button_Auto = 0;
				}
				else if(Button_Single == 1)  //按下单步模式按键
				{
					Present_Mode = MODE_SINGLE;	 //进入单步模式
					Flag_Mode = MODE_SINGLE;  	 //记录单步模式标志
					Button_Single = 0;				
				}
			}
			else    //工作台不在原点
			{				
				if(Button_Recover == 1)  //复位按钮按下，电机复位
				{
					Motor_Restore();  	 //电机复位函数
					Button_Recover = 0;
				}			
				Button_Auto = 0;   //按下自动模式按钮，无响应
				Button_Single = 0; //按下单步模式按钮，无响应			
			}			
			//在不在原点，都会响应手动模式
			if(Button_Manual == 1)  //按下手动模式
			{ 			
				Present_Mode = 4;	//进入手动模式
				Flag_Mode = 4;  	//记录手动模式标志
				Button_Manual = 0;
			}		
		}
		else  //在复位过程中，按下按键，按钮不响应
		{
			Button_Auto = 0; //按下自动模式按钮，无响应
			Button_Single = 0;//按下单步模式按钮，无响应
			Button_Manual = 0;//按下手动模式按钮，无响应
			Button_Recover = 0; //按下复位按钮，无响应
		}		
		if(Button_Stop == 1) //按下退出按钮
		{
			Motor_Dir = DIR_STOP; //电机方向改为停止	
			Status_Return = 0; 	  //返回状态灯置0
			Status_Motion = STATUS_REST; //运动状态为停止
			Button_Stop = 0;
		}
	}
	if(Flag_Mode == MODE_AUTO)  //自动模式
	{
		if(Button_Start == 1)  //按下启动按钮
		{
			if(Status_Motion == STATUS_REST) //停止状态
			{
				if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //工作台在原点
				{
					if((Data_Speed1>0) && (Data_Speed2>0) && \
					   (Data_Speed3>0) && (Data_Speed4>0) && \
					   (Data_BKSpeed>0) ) //五段速度均大于0
					{
						Present_Data_Speed[0] = Data_Speed1;  //记录当前输入的一段速度，运动过程中更改速度无效				
						Present_Data_Speed[1] = Data_Speed2;  //记录当前输入的二段速度，运动过程中更改速度无效				
						Present_Data_Speed[2] = Data_Speed3;  //记录当前输入的三段速度，运动过程中更改速度无效				
						Present_Data_Speed[3] = Data_Speed4;  //记录当前输入的四段速度，运动过程中更改速度无效
						Present_Data_Speed[4] = Data_BKSpeed; //记录当前输入的返回速度，运动过程中更改速度无效
						Data_Stage = 1;  //当前段数为第一段	
						Motor_Dir = DIR_RIGHT; //电机向右运动
						Data_RunSpeed = Present_Data_Speed[0];//初始速度
						Status_Motion = STATUS_AUTOMOD;  //标志当前运动状态为自动运动状态
					}
				}	
			}	
			Button_Start = 0;
		}
		else if(Button_Recover == 1)  //按下复位
		{
			if(Status_Motion == STATUS_REST) //停止状态
			{			
				Motor_Restore(); 					//复位
			}
			Button_Recover = 0;
		}
		else if(Button_Stop == 1) //按下停止按钮
		{
			Motor_Dir = DIR_STOP; //电机方向改为停止	
			Status_Return = 0; //返回灯灭
			Status_Wait = 0;   //等待指示灯灭
			Status_Motion = STATUS_REST; //标志当前状态为静止
			Data_Stage = 0;    //段数归零
			Button_Stop = 0;  
		}	
	}
	if(Flag_Mode == MODE_SINGLE)  //单步模式
	{
		if(Button_Start == 1) //按下启动按钮
		{
			if(Status_Motion == STATUS_REST)  //当前状态为静止状态
			{
				Data_Stage++;  //段数加一
				if(Data_Stage == 1) //等于1说明第一次按下启动按钮
				{
					if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //工作台在原点
					{
						Motor_Dir = DIR_RIGHT; //电机向右转动
						Data_RunSpeed = Data_Speed;
						Status_Motion = STATUS_SINGMOD; //当前状态为单步状态
					}
					else  //第一步不在原点
					{
						Data_Stage = 0;  //段数归0
					}
				}
				else if(Data_Stage <= 4) //四段之内
				{
					Motor_Dir = DIR_RIGHT; //电机向右转动
					Data_RunSpeed = Data_Speed;
					Status_Motion = STATUS_SINGMOD; //当前状态为单步状态
				}	
				else if(Data_Stage == 5)
				{
					Status_Return = 1;
					Motor_Dir = DIR_LEFT; //电机向左转动
					Data_RunSpeed = Data_BKSpeed;   //设置返回速度
					Status_Motion = STATUS_ONESTEP; //当前状态为单步状态
				}
			}	
			Button_Start = 0;			
		}		
		else if(Button_Recover == 1)  //按下复位
		{
			if(Status_Motion == STATUS_REST) //当前状态为停止状态
			{
				Motor_Restore(); //复位函数中会判断原点
			    Data_Stage = 0;  //段数归零
			}
			Button_Recover = 0;
		}
		else if(Button_Stop == 1)
		{
			Motor_Dir = DIR_STOP; //电机方向改为停止
			Status_Motion = STATUS_REST; //当前状态为静止状态
			Data_Stage = 0; 	//段数归零
			Status_Return = 0;  //返回灯灭
			Button_Stop = 0;
		}	
	}
	if(Flag_Mode == MODE_MANUAL)  //手动模式
	{		
		if(Status_Motion == STATUS_REST)  //当前状态为静止状态
		{		
			if(Button_Left && (Flag_Button_Left==0)) //按下左运动按钮
			{
				if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT)!=1) //不在左极限
				{
					Flag_Button_Left = 1;  //按键按下标志位置1
					Motor_Dir = DIR_LEFT;  //电机方向改为左转
				}			
			}
			else if((Button_Left==0) && Flag_Button_Left) //按键松开
			{
				Flag_Button_Left = 0;   //按键松开标志位置0
				Motor_Dir = DIR_STOP; 	//电机方向改为停止
			}
			
			if(Button_Right && (Flag_Button_Right==0)) //按下右运动按钮
			{
				if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT)!=1) //不在右极限
				{
					Flag_Button_Right = 1;  //按键按下标志位置1
					Motor_Dir = DIR_RIGHT;  //电机方向改为右转
				}	
			}
			else if((Button_Right==0) && Flag_Button_Right) //按键松开
			{
				Flag_Button_Right = 0;  //按键松开标志位置0
				Motor_Dir = DIR_STOP;   //电机方向改为停止
			}
			
			if(Button_Recover == 1)  //按下复位
			{
				if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //工作台在原点
				{
					//未处理
				}
				else  //工作台不在原点
				{
					Status_Return = 1;  //返回状态灯置1
					Motor_Restore(); 	//开始复位
				}
				Button_Recover = 0;
			}
		}	
		else  //当前状态非静止状态
		{
			Button_Recover = 0; //复位按钮按下无响应
		}
		
		if(Button_Stop == 1) //按下了停止按钮
		{
			Motor_Dir = DIR_STOP; //电机方向改为停止
			Status_Return = 0; 	  //返回状态灯置0
			Status_Motion = STATUS_REST; //运动状态为静止状态
			Button_Stop = 0;
		}
	}	
	
	if(Button_BackMain == 1)   //退出按钮按下
	{
		Button_BackMain = 0;		
		Present_Mode = MODE_SELECT;	  //进入模式选择
		Flag_Mode = MODE_SELECT;  	  //记录模式选择标志
		Motor_Dir = DIR_STOP;   	  //电机方向改为停止
		Status_Motion = STATUS_REST;  //运动状态为静止状态
		Status_Return = 0; 	//返回状态灯灭
		Data_Stage = 0;   	//段数归零
	}	
	
	//状态指示灯处理	
	if(Data_RTSpeed > 0)
	{
		Status_DirLeft = 0;  //左方向灯灭
		Status_DirRight = 1; //右方向灯亮		
	}
	else if(Data_RTSpeed == 0)
	{
		Status_DirLeft = 0;  //左方向灯灭
		Status_DirRight = 0; //右方向灯灭		
	}
	else if(Data_RTSpeed < 0)
	{
		Status_DirLeft = 1;	 //左方向灯亮		
		Status_DirRight = 0; //右方向灯灭		
	}
	//左右极限状态
	if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == 1)
	{
		Status_EndLeft = 1; //左极限灯亮
	}
	else
	{
		Status_EndLeft = 0; //左极限灯灭
	}
	if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT) == 1)
	{
		Status_EndRight = 1; //右极限灯亮
	}
	else 
	{
		Status_EndRight = 0; //右极限灯灭	
	}
	if((Data_Location>=-0.1f) && (Data_Location<=0.1f)) //工作台在原点
	{
		Status_Recover = 1;
	}
	else 
	{
		Status_Recover = 0;	
	}
}

/**
 *作用：控制变频器的指令发送
 *流程：首先判断速度和方向有没有改变，改变了就开启定时器，30ms之后发送指令
**/
void FreqChg_Control(void)
{
	//给变频器发指令
	if(communicate == SEND2)
	{
		if(Last_Dir != Motor_Dir)  //方向改变
		{
			__disable_irq() ; //关闭总中断
			switch(Motor_Dir)
			{
				case DIR_STOP:
					Motor_Stop();  //电机停止转动	
					if(Flag_Init == DOING)  //正在上电复位
					{
						TIM5_Count = 2; 	//200ms*2之后进入中断，位置清零
						TIM_Cmd(TIM5, ENABLE); 	//使能定时器5，中断计时
					}						
					break;
				case DIR_LEFT:
					Motor_Left();  //电机左转
					break;
				case DIR_RIGHT:
					Motor_Right(); //电机右转
					break;
			}
			communicate = WAITING; //等待30ms
			TIM_Cmd(TIM2, ENABLE); //使能定时器2
			__enable_irq() ; //打开总中断
		}
		else if(Last_Speed != Data_RunSpeed) //速度改变
		{	
			__disable_irq() ; //关闭总中断 
			//近似认为速度和频率呈线性关系，比例系数求得为4.13
			FREQ_Change_Freq(Data_RunSpeed * 4.13f); //更改变频器的频率
			communicate = WAITING; //等待30ms			
			TIM_Cmd(TIM2, ENABLE); //使能定时器2
			__enable_irq() ; //打开总中断
		}	
	}
}

/**
 *作用：电机复位回原点的第一步运动
 *流程：复位状态进入第一步，修改频率为50
**/
//void Motor_Restore(void)
//{	
//	Status_Return = 1;  //返回灯亮
//	
//	if(Flag_Init == DOING)  //表示正在上电复位
//	{
//		Data_RunSpeed = 15.0f;  //速度15mm/s
//		
//		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //位置在左极限
//		{
//			Motor_Dir = DIR_RIGHT; 			//电机方向改为右转
//			Status_Motion = STATUS_TWOSTEP; //运动状态为复位第二步状态
//			Location_Left_Limit = 1.0f;  	//记录左极限当前位置
//		}
//		else   //除了左极限的其他位置
//		{
//			Motor_Dir = DIR_LEFT; 		//电机方向改为左转
//			Status_Motion = STATUS_ONESTEP; //运动状态为复位第一步状态
//		}		
//	}
//	else if(Flag_Init == DONE)  //表示上电复位完成了
//	{
//		if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //工作台在原点 [0.0mm,0.05mm)
//		{
//			Status_Return = 0;  //返回灯灭
//		}
//		else   //工作台不在原点
//		{
//			Status_Motion = STATUS_THREESTEP; //运动状态为复位第三步状态			
//		}
//	}
//}

void Motor_Restore(void)
{	
	Status_Return = 1;  	//返回灯亮	
	if(Flag_Init == DOING)  //表示正在上电复位
	{
		Data_RunSpeed = 15.0f;  //速度15mm/s	
		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //位置在左极限
		{
			Motor_Dir = DIR_RIGHT; 			//电机方向改为右转
			Status_Motion = STATUS_TWOSTEP; //运动状态为复位第二步状态
			Location_Left_Limit = 1.0f;  	//记录左极限当前位置
		}
		else   //除了左极限的其他位置
		{
			Motor_Dir = DIR_LEFT; 		//电机方向改为左转
			Status_Motion = STATUS_ONESTEP; //运动状态为复位第一步状态
		}	
	}	
	else if(Flag_Init == DONE)  //表示上电复位完成了
	{
		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //位置在左极限
		{
			Data_RunSpeed = 15.0f;  //速度15mm/s
			Motor_Dir = DIR_RIGHT; 	//电机方向改为右转
			Status_Motion = STATUS_TWOSTEP; //运动状态为复位第二步状态
		}
		else  //除了左极限的其他位置
		{
			if(Data_Location <= -10.0f)  //位置小于-10mm
			{
				Motor_Dir = DIR_RIGHT; 			//电机方向改为右转
				Data_RunSpeed = 15.0f;  		//速度15mm/s
				Status_Motion = STATUS_TWOSTEP; //运动状态为复位第二步状态
			}
			else if(Data_Location <= -4.0f)
			{
				Motor_Dir = DIR_RIGHT; 			//电机方向改为右转
				Data_RunSpeed = 2.0f;  			//速度2mm/s	
				Status_Motion = STATUS_TWOSTEP; //运动状态为复位第二步状态
			}
			else if(((Data_Location<-0.1f) && (Data_Location>-4.0f))) //这段距离定时左移，返回会检测到上升沿
			{
				Motor_Dir = DIR_LEFT; 			//电机方向改为左转
				Data_RunSpeed = 15.0f;  		//速度15mm/s	
				Status_Motion = STATUS_ONESTEP; //运动状态为复位第一步状态
				TIM5_Count = 5; //定时器5将进入5次中断后才会关闭					
				TIM_Cmd(TIM5, ENABLE); 		//使能定时器5			
			}
			else if((Data_Location>0.1f) && (Data_Location<5.0f))  //这段距离定时左移，返回会检测到上升沿
			{
				Motor_Dir = DIR_LEFT; 			//电机方向改为左转
				Data_RunSpeed = 15.0f;  		//速度15mm/s			
				Status_Motion = STATUS_ONESTEP; //运动状态为复位第一步状态
				TIM5_Count = 6; //定时器5将进入x次中断后才会关闭					
				TIM_Cmd(TIM5, ENABLE); 	//使能定时器5
			}
			else if(Data_Location >= 5.0f)
			{
				Motor_Dir = DIR_LEFT; 			//电机方向改为左转
				Data_RunSpeed = 15.0f;  		//速度15mm/s
				Status_Motion = STATUS_ONESTEP; //运动状态为复位第一步状态
			}
			else  //在原点位置
			{
				Status_Return = 0;  //返回灯灭
			}
		}		
	}
}

/**
 *作用：极限中断处理函数
 *note：上升沿触发
**/
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line7))  //左极限
	{
		if(Status_Motion == STATUS_ONESTEP) //运动状态为复位第一步状态
		{
			Location_Left_Limit = Data_Location; //记录左极限当前位置			
			Status_Motion = STATUS_TWOSTEP; //运动状态为复位第二步状态
			Motor_Dir = DIR_RIGHT;   	//电机方向改为右转
			Data_RunSpeed = 15.0f;
		}
		else  //其他状况碰到左极限
		{
			Motor_Dir = DIR_STOP; //电机方向改为停止				
			Status_Motion = STATUS_REST;
			Data_Stage = 0;  //段数归零
		}
		EXTI_ClearFlag(EXTI_Line7);		
	}
	if(EXTI_GetFlagStatus(EXTI_Line6))  //原点
	{	
		if(Status_Motion == STATUS_ONESTEP) //运动状态为复位第一步状态
		{		
			//复位回原点时间动态处理，20mm/速度/0.2s = TIM5_Count
			TIM5_Count = (u8)(15.0f/Data_RunSpeed/0.2f); //定时器5将进入x次中断后才会关闭					
			TIM_Cmd(TIM5, ENABLE); 	//使能定时器5			
		}
		else if(Status_Motion == STATUS_TWOSTEP)  //运动状态为复位第二步状态
		{
			TIM5_Count = 9; //定时器5将进入1次中断后才会关闭
			TIM_Cmd(TIM5, ENABLE); 	//使能定时器5，中断计时
			Data_RunSpeed = 2.0f; 	//低速运行一小段距离
		}	
		EXTI_ClearFlag(EXTI_Line6);
	}	
	if(EXTI_GetFlagStatus(EXTI_Line5)) //右极限
	{
		Motor_Dir = DIR_STOP; 	//电机方向改为停止	
		Status_Motion = STATUS_REST; //当前为静止状态	
		EXTI_ClearFlag(EXTI_Line5);
	}
}

void TIM2_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET) 	//溢出中断
	{	
		if(Last_Dir != Motor_Dir) 
		{
			Last_Dir = Motor_Dir; //不会重发，通信完成
		}
		else if(Last_Speed != Data_RunSpeed)
		{
			Last_Speed = Data_RunSpeed;
		}
		communicate = SEND2;	 //可以发送指令
//		communicate = RECEIVE2; //转换为接收状态
		TIM_Cmd(TIM2, DISABLE); //失能定时器2
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 	//清除中断标志位
	}
}

//发送完成之后进入
void FreqRev_Deal(void)
{
	u8 i;
	if(communicate == RECEIVE2) //接收状态
	{
		for(i = 0; i < 3; i++)
		{
			if(Receive_485_flag == 1)  //485接收到数据
			{
				Modbus2_Parse(); //处理变频器返回的数据
				//清空数据
				memset(SendBuff_485, 0, LEN_SEND_485);
				memset(ReceBuff_485, 0, LEN_RECV_485);
				break; //跳出循环
			}
			else if(Receive_485_flag == 0)//485未接收到回复数据，重发
			{
				if(Last_Dir != Motor_Dir)  //方向改变
				{
					switch(Motor_Dir)
					{
						case DIR_STOP:
							Motor_Stop();  //电机停止转动						
							break;
						case DIR_LEFT:
							Motor_Left();  //电机左转
							break;
						case DIR_RIGHT:
							Motor_Right(); //电机右转
							break;
					}
				}
				else if(Last_Speed != Data_RunSpeed) //速度改变
				{	
					FREQ_Change_Freq(Data_RunSpeed * 4.13f); //更改变频器的频率	
				}
				delay_ms(25); //重发之后延时25ms				
			}
		}
		if(i == 3) //第三次重发
		{
			if(Receive_485_flag == 1)  //485接收到数据
			{
				Modbus2_Parse(); //处理变频器返回的数据
				//清空数据
				memset(SendBuff_485, 0, LEN_SEND_485);
				memset(ReceBuff_485, 0, LEN_RECV_485);
			}
			else 
			{
				//报错！！！！
				Data_Error = 5;   //回路不畅
				Status_Alarm = 0; //通信报错
			}	
		}
		
		//清除数据！！！！
		if(Last_Dir != Motor_Dir) 
		{
			Last_Dir = Motor_Dir; //不会重发，通信完成
		}
		else if(Last_Speed != Data_RunSpeed)
		{
			Last_Speed = Data_RunSpeed;
		}
		communicate = SEND2;	 //可以发送指令			
		Receive_485_flag = 0;	 //清除接收标志位	
	}
}

void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET) 	//溢出中断
	{	
		Increment = TIM4->CNT; //获取编码器脉冲数
		TIM4->CNT = 0;  //TIM4计数器的值归0
		
		//换算得到速度值：Increment*2.5mm/1024脉冲/20ms
		if(Increment < 1024)
		{
			Data_RTSpeed = -1.0f * Increment * 2.5f / 2048.0f / 0.02f;  //反转
			Encoder_Pulse_NUM -= Increment;
		}			
		else if(Increment > 1024)		
		{
			Data_RTSpeed = (Parameter - Increment) * 2.5f / 2048.0f / 0.02f;  //正转
			Encoder_Pulse_NUM += (Parameter - Increment);			
		}
		//位置：脉冲数*2.5mm/1024 单位mm
		Data_Location = Encoder_Pulse_NUM * 2.5 / 2048;		
		Data_Animation = (int)Data_Location; //动画位置		
		if(Status_Motion == STATUS_AUTOMOD) //自动模式运动状态下
		{
			if(Data_Stage <= 3) //前四段运行中
			{			
				if(Encoder_Pulse_NUM >= 32768 * Data_Stage)  //运动超过了40mm，换速
				{
					Data_Stage++;  	 //段数加1，最多加到4
					Data_RunSpeed = Present_Data_Speed[Data_Stage-1]; //段数为1时取Present_Data_Speed[0]作为当前段速					
				}
			}		
			if(Data_Stage == 4)  //第四段运行中
			{
				if(Encoder_Pulse_NUM >= (32768 * 3 + 32700))  //超过指定位置
				{
					Data_Stage++; 
					Status_Wait = 1;		//等待指示灯亮
					Motor_Dir = DIR_STOP; 	//电机停止转动			
					TIM5_Count = 15; 		//定时器5将进入15次中断，即3s钟
					TIM_Cmd(TIM5, ENABLE); 	//使能定时器5
				}
				else if(Encoder_Pulse_NUM >= (32768 * 3 + 26624))  //运动到160mm前三圈，换速
				{
					Data_RunSpeed = 2.0f; //低速前进
				}	
//				PID_Control_SPD(160.03f, Present_Data_Speed[Data_Stage-1]); //PID控制			
			}		
//			else if(Data_Stage == 5)  //回原点
//			{
//				PID_Control_SPD(0.03f, Present_Data_Speed[Data_Stage-1]); //PID控制	
//			}				
		}				
		else if(Status_Motion == STATUS_TWOSTEP) //在复位的第二步状态中
		{
			if(Flag_Init == DOING)  //当前为上电复位状态
			{
				if(Location_Left_Limit != 0)  //不等于零说明到过左极限
				{
					if((Data_Location-Location_Left_Limit) > 70.0f) 
					{
						Data_RunSpeed = 2.0f;   //减速运行	
						Location_Left_Limit = 0.0f;   //处理过极限后，位置归零
					}
				}
			}
			else if(Flag_Init == DONE) //开机复位完成，可以得到正确距离
			{
				if((Data_Location>-17.0f) && (Data_Location<-0.4f)) //复位位置在-17mm后，降速到2mm/s
				{
					Data_RunSpeed = 2.0f; 	//低速运行一小段距离			
				}
			}
		}
//		else if(Status_Motion == STATUS_THREESTEP) //按下了复位按钮
//		{
//			PID_Control_SPD(0.03f, 15.0f); //更改运行速度
//		}
		else if(Status_Motion == STATUS_SINGMOD)
		{
			if(Data_Stage <= 4)  //段数小于等于4
			{
				if(Encoder_Pulse_NUM >= (32768 * (Data_Stage-1) + 32700))  //超过指定位置
				{
					Motor_Dir = DIR_STOP; 	//电机停止转动
					Status_Motion = STATUS_REST;					
				}
				else if(Encoder_Pulse_NUM >= (32768 * (Data_Stage-1) + 26624))  //运动到160mm前三圈，换速
				{
					Data_RunSpeed = 2.0f; //低速前进
				}
//				PID_Control_SPD(Data_Stage*40.0f+0.03f, Data_Speed); //PID控制				
			}
//			else if(Data_Stage == 5)
//			{
//				Motor_Dir = DIR_LEFT; 			//电机向左转动
//				Status_Motion = STATUS_ONESTEP; //回原点转为复位操作
//				Data_RunSpeed = Data_BKSpeed;   //设置返回速度
////				PID_Control_SPD(0.03f, Data_BKSpeed); //PID控制								
//			}
		}
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); 	    //清除中断标志位
}
	
void TIM5_IRQHandler(void)
{		
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET) 	//溢出中断
	{
		TIM5_Count--;
		if(TIM5_Count == 0)
		{				
			TIM_Cmd(TIM5, DISABLE);   //失能定时器5			
			if((Flag_Init==DOING) && (Status_Motion==STATUS_REST))  	  //上电复位
 			{
				Encoder_Pulse_NUM = 0; //位置归零
				Flag_Init = DONE;   //标志上电复位完成
				//按钮清空
				Button_Recover = 0; //复位按钮 
				Button_Stop = 0;    //停止按钮
				Button_Auto = 0;    //自动模式切换按钮
				Button_Single = 0;  //单步模式切换按钮
			    Button_Manual = 0;  //手动模式切换按钮
			}					
			if(Status_Motion == STATUS_ONESTEP) //当前处于第一步
			{
				Motor_Dir = DIR_RIGHT; 		//电机方向改为右转				
				Data_RunSpeed = 2.0f;			
				Status_Motion = STATUS_TWOSTEP; //进入第二步复位
			}
			else if(Status_Motion == STATUS_TWOSTEP) //达到中心点
			{
				Motor_Dir = DIR_STOP; 		//电机方向改为停止
				Status_Motion = STATUS_REST; //运动状态改为停止				
				Data_Stage = 0;    //自动模式的段数改为0段
				Status_Return = 0; //返回状态灯置0				
			}	
			else if(Status_Motion == STATUS_AUTOMOD)
			{
				Status_Wait = 0;
				Status_Return = 1; //返回状态灯置1
				Motor_Dir = DIR_LEFT; //电机向左转动
				Status_Motion = STATUS_ONESTEP; //回原点转为复位操作！！			
				Data_RunSpeed = Present_Data_Speed[Data_Stage-1]; //设置返回速度
			}				
//			if(Motor_Dir == DIR_STOP) //消除抖动，200ms后依然是静止状态
//			{
//				if(Status_Motion == STATUS_SINGMOD)
//				{
//					Status_Motion = STATUS_REST; //运动状态改为停止
//					if(Data_Stage == 5)
//					{
//						Data_Stage = 0;
//					}
//					Motor_Dir = DIR_STOP;
//				}
////				else if(Status_Motion == STATUS_THREESTEP)
////				{
////					Status_Motion = STATUS_REST; //运动状态改为停止
////					Data_Stage = 0; //自动模式的段数改为0段
////					Status_Return = 0; //返回状态灯灭
////				}
//				else if(Status_Motion == STATUS_AUTOMOD)
//				{
//					if((Data_Stage==4) && (Status_Wait==0)) //说明需要定时3s
//					{
//						Status_Wait = 1;	//等待指示灯亮
//						TIM_Cmd(TIM5, ENABLE);  //使能定时器5
//						TIM5_Count = 15;   		//3s定时
//					}
//					else if((Data_Stage==4) && (Status_Wait==1))
//					{
//						Status_Wait = 0;	//等待指示灯灭
//						Status_Return = 1;	//返回状态灯亮
//						Data_Stage++; 		//第五段	
//						
//						Motor_Dir = DIR_LEFT; 			//电机向左转动
//						Status_Motion = STATUS_ONESTEP; //回原点转为复位操作
//						Data_RunSpeed = Present_Data_Speed[Data_Stage-1]; //设置返回速度[4]
//					}			
////					else if(Data_Stage == 5)  //回原点结束了
////					{
////						Status_Motion = STATUS_REST; //运动状态改为停止	
////						Data_Stage = 0; 	//自动模式的段数改为0段
////						Status_Return = 0;  //返回状态灯灭
////					}
//				}
//			}			
		}		
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 	 //清除中断标志位
}

