#include "control.h"
#include "modbus_slave.h"
#include "modbus_master.h"
#include "led.h"
#include "exti.h"
#include "encoder.h"
#include "time.h"
#include "delay.h"
#include "pid.h"

long  Encoder_Pulse_NUM = 0; //计算实时位置
long  Last_Pulse_NUM    = 0; //当前脉冲数记为0 
vu8   Flag_Init = DOING; 	 //当前为正在上电复位
vu8   Flag_Status_Motion = STATUS_REST; //当前为静止状态
vu8   TIM5_Count = 0; 		 //定时器5的计时次数

float Last_Speed = 0.0f;     //上一次的频率值，如果和当前的不一样就要更改

u8    Motor_Dir = DIR_STOP;  //电机为停止状态
u8    Last_Dir = DIR_STOP;   //默认状态静止

volatile float Location_Left_Limit = 0.0f;   //开机复位如果到达左极限，就记录左极限位置

//记录自动模式下按下启动时的四个速度，运动过程中更改速度无效
float Present_Data_Speed[6] = {0};  //1-5存储数据，0不存数据   

/**
 *作用：模式切换，逻辑控制部分
**/
void Mode_Control(void)
{	
	static u8  Flag_Button_Left = 0; 	 //左按钮未按下的标志
	static u8  Flag_Button_Right = 0;    //右按钮未按下的标志
	static u8  Flag_Mode = MODE_SELECT;     //开机PLC默认当前为第一页面，页面变量，PLC中储存
	
	if(Flag_Mode == MODE_SELECT)  //当前处于模式选择
	{	
		if(Flag_Status_Motion == STATUS_REST)  //在静止过程中
		{
			if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //工作台在原点
			{				
				Button_Recover = 0;  //在原点按下复位无响应
				if(Button_Auto == 1) //按下自动模式按键
				{
					Present_Mode = MODE_AUTO; 	   //进入自动模式
					Flag_Mode = MODE_AUTO;  	   //记录自动模式标志
					Button_Auto = 0;
				}
				else if(Button_Single == 1)  //按下单步模式按键
				{
					Present_Mode = MODE_SINGLE;	    //进入单步模式
					Flag_Mode = MODE_SINGLE;  		//记录单步模式标志
					Button_Single = 0;				
				}
			}
			else    //工作台不在原点
			{				
				if(Button_Recover == 1)  		//复位按下，电机复位
				{
					Motor_Restore();  	   //电机复位函数，只在按钮按下后执行，执行一次！！！
					Button_Recover = 0;
				}
				
				Button_Auto = 0; 	//按下自动模式按钮，无响应
				Button_Single = 0; //按下单步模式按钮，无响应			
			}			
			//在不在原点，都会响应手动模式
			if(Button_Manual == 1)  //按下手动模式
			{ 			
				Present_Mode = 4;	          //触摸屏切换到第四页，之后自动被触摸屏置0
				Flag_Mode = 4;  			  //PLC记录当前页数
				Button_Manual = 0;
			}		
		}
		else  //在复位过程中，按下按键，按钮不响应
		{
			Button_Auto = 0; 	//按下自动模式按钮，无响应
			Button_Single = 0;//按下单步模式按钮，无响应
			Button_Manual = 0;//按下手动模式按钮，无响应
			Button_Recover = 0; //按下复位按钮，无响应
		}
		
		if(Button_Stop == 1)
		{
			Motor_Dir = DIR_STOP; //电机停止转动	
			Status_Return = 0; //返回状态灯置0
			Flag_Status_Motion = STATUS_REST; //运动状态为停止
			Button_Stop = 0;
		}
	}

/******************* 自动模式界面--进入自动模式 ****************************/
	if(Flag_Mode == MODE_AUTO)
	{
		if(Button_Start == 1)  //按下启动按钮
		{
			if(Flag_Status_Motion == STATUS_REST) //停止状态
			{
				if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //工作台在原点
				{
					if((Data_Speed1>0) && (Data_Speed2>0) && \
						 (Data_Speed3>0) && (Data_Speed4>0) && \
						 (Data_BKSpeed>0) ) //五段速度均大于0
					{
						Present_Data_Speed[1] = Data_Speed1;  //记录当前输入的一段速度				
						Present_Data_Speed[2] = Data_Speed2;  //记录当前输入的二段速度				
						Present_Data_Speed[3] = Data_Speed3;  //记录当前输入的三段速度				
						Present_Data_Speed[4] = Data_Speed4;  //记录当前输入的四段速度
						Present_Data_Speed[5] = Data_BKSpeed; //记录当前输入的返回速度
						
						Data_Stage = 1;  //当前段数为第一段						
						Flag_Status_Motion = STATUS_AUTOMOD;  //标志当前运动状态为自动运动状态
					}
				}	
			}	
			Button_Start = 0;
		}
		else if(Button_Recover == 1)  //按下复位
		{
			if(Flag_Status_Motion == STATUS_REST) //停止状态
			{			
				Motor_Restore(); 					//复位
			}
			Button_Recover = 0;
		}
		else if(Button_Stop == 1) //按下停止按钮
		{
			Motor_Dir = DIR_STOP; 	 //电机停止转动	
			Status_Return = 0; //返回灯灭
			Status_Wait = 0;   //等待指示灯灭
			Flag_Status_Motion = STATUS_REST; //标志当前状态为静止
			Data_Stage = 0;    //段数归零
			Button_Stop = 0;  
		}	
	}

/******************* 单步模式界面--按下按钮 		****************************/
	if(Flag_Mode == MODE_SINGLE)
	{
		if(Button_Start == 1) //按下启动按钮
		{
			if(Flag_Status_Motion == STATUS_REST)  //当前运动状态为静止
			{
				Data_Stage++;  //段数加一
				if(Data_Stage == 1) //第一次按下
				{
					if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //工作台在原点
					{
						Flag_Status_Motion = STATUS_SINGMOD; //当前运动模式为单步模式
					}
					else  //第一步不在原点
					{
						Data_Stage = 0;  //段数归0
					}
				}
				else if(Data_Stage <= 5) //五步之内
				{
					Flag_Status_Motion = STATUS_SINGMOD; //当前运动模式为单步模式
				}	
			}	
			Button_Start = 0;			
		}		
		else if(Button_Recover == 1)  //按下复位
		{
			if(Flag_Status_Motion == STATUS_REST) //停止状态
			{
				Motor_Restore(); //复位函数中会判断原点
			    Data_Stage = 0; //段数归零
			}
			Button_Recover = 0;
		}
		else if(Button_Stop == 1)
		{
			Motor_Dir = DIR_STOP; //电机停止转动	
			Flag_Status_Motion = STATUS_REST; //运动状态为停止
			Data_Stage = 0; //段数归零
			Status_Return = 0;
			Button_Stop = 0;
		}	
	}

/******************* 手动模式界面--手动控制按钮 ****************************/
	if(Flag_Mode == MODE_MANUAL)
	{		
		if(Flag_Status_Motion == STATUS_REST)  //在静止过程中
		{		
			if(Button_Left && (Flag_Button_Left==0)) //第一次按下才发命令
			{
				if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT)!=1) //不在左极限
				{
					Flag_Button_Left = 1;
					Motor_Dir = DIR_LEFT; //电机向左转动
				}			
			}
			else if((Button_Left==0) && Flag_Button_Left)
			{
				Flag_Button_Left = 0;   //按键松开了
				Motor_Dir = DIR_STOP; 	//电机停止转动
			}
			
			if(Button_Right && (Flag_Button_Right==0))
			{
				if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT)!=1) //不在右极限
				{
					Flag_Button_Right = 1;
					Motor_Dir = DIR_RIGHT;  //电机向右转动
				}	
			}
			else if((Button_Right==0) && Flag_Button_Right) //按键松开
			{
				Flag_Button_Right = 0;  //按键松开了
				Motor_Dir = DIR_STOP;  //电机停止转动
			}
			
			if(Button_Recover == 1)  //按下复位
			{
				if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //工作台在原点
				{
					//未处理
				}
				else  //工作台不在原点
				{
					Status_Return = 1; //返回状态灯置1
					Motor_Restore(); 				 //开始复位，给定了PID的位置和速度，进入了第三复位状态！！！！
				}
				Button_Recover = 0;
			}
		}	
		else  //不在静止过程中
		{
			Button_Recover = 0; //无响应
		}
		
		if(Button_Stop == 1) //按下了停止按钮
		{
			Motor_Dir = DIR_STOP; //电机停止转动	
			Status_Return = 0; //返回状态灯置0
			Flag_Status_Motion = STATUS_REST; //运动状态为停止
			Button_Stop = 0;
		}
	}	
	
/******************* 后三个页面--返回按钮按下 	****************************/
	if(Button_BackMain == 1)
	{
		Button_BackMain = 0;		
		Present_Mode = MODE_SELECT;	   //进入模式选择
		Flag_Mode = MODE_SELECT;  	   //记录模式选择标志
		Motor_Dir = DIR_STOP;   //按下返回，电机停止运动
		Flag_Status_Motion = STATUS_REST; //运动状态为停止
		Status_Return = 0; //返回状态灯置0
		Data_Stage = 0;   //段数归零
	}	

/******************* 状态指示灯处理  						****************************/	
	if(Data_RTSpeed > 0)
	{
		Status_DirLeft = 0;
		Status_DirRight = 1;		
	}
	else if(Data_RTSpeed == 0)
	{
		Status_DirLeft = 0;
		Status_DirRight = 0;		
	}
	else if(Data_RTSpeed < 0)
	{
		Status_DirLeft = 1;			
		Status_DirRight = 0;		
	}
	//左右极限状态
	if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == 1)
	{
		Status_EndLeft = 1;
	}
	else
	{
		Status_EndLeft = 0;
	}
	if(GPIO_ReadInputDataBit(GPIOI, RIGHT_LIMIT) == 1)
	{
		Status_EndRight = 1;
	}
	else 
	{
		Status_EndRight = 0;	
	}
	if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //工作台在原点
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
	if((Motor_Dir!=Last_Dir) || (Data_RunSpeed!=Last_Speed)) //停止可以及时响应
	{
		TIM_Cmd(TIM2, ENABLE); 	//使能定时器2
	}
}

/**
 *作用：电机复位回原点的第一步运动
 *流程：复位状态进入第一步，修改频率为50
**/
void Motor_Restore(void)
{	
	Status_Return = 1;  //返回灯亮
	
	if(Flag_Init == DOING)  //表示正在上电复位
	{
		Data_RunSpeed = 15.0f;  //速度15mm/s
		
		if(GPIO_ReadInputDataBit(GPIOI, LEFT_LIMIT) == Bit_SET)  //位置在左极限
		{
			Motor_Dir = DIR_RIGHT; 		//电机向右转动
			Flag_Status_Motion = STATUS_TWOSTEP; //表示当前正在复位的第二步
			Location_Left_Limit = 1.0f;  //记录左极限当前位置
		}
		else   //除了左极限的其他位置
		{
			Motor_Dir = DIR_LEFT; 				//电机向左转动
			Flag_Status_Motion = STATUS_ONESTEP; //表示当前正在复位的第一步
		}		
	}
	else if(Flag_Init == DONE)  //表示上电复位完成了
	{
		if((Data_Location>=-0.0f) && (Data_Location<0.05f)) //工作台在原点
		{
			Status_Return = 0;  //返回灯灭
		}
		else   //工作台不在原点
		{
			Flag_Status_Motion = STATUS_THREESTEP; //表示当前正在复位的第三步			
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
		if(Flag_Status_Motion == STATUS_ONESTEP) //当前为复位状态第一步
		{
			Location_Left_Limit = Data_Location;  //记录左极限当前位置			
			Flag_Status_Motion = STATUS_TWOSTEP; 				//复位进入了第二步
			Motor_Dir = DIR_RIGHT;   	//复位就向右运动
			Data_RunSpeed = 15.0f;
		}
		else  //其他状况碰到左极限
		{
			Motor_Dir = DIR_STOP; //电机停止转动				
			Flag_Status_Motion = STATUS_REST;
			Data_Stage = 0;  //段数归零
		}
		EXTI_ClearFlag(EXTI_Line7);		
	}
	
	if(EXTI_GetFlagStatus(EXTI_Line6))  //原点
	{	
		if(Flag_Status_Motion == STATUS_ONESTEP)
		{		
			//复位回原点时间动态处理，20mm/速度/0.2s = TIM5_Count
			TIM5_Count = (u8)(15.0f/Data_RunSpeed/0.2f); //定时器5将进入x次中断后才会关闭					
			TIM_Cmd(TIM5, ENABLE); 	//使能定时器5，中断计时			
		}
		else if(Flag_Status_Motion == STATUS_TWOSTEP)  //没在复位状态，遇到上升沿未处理
		{
			TIM5_Count = 9; //定时器5将进入1次中断后才会关闭
			TIM_Cmd(TIM5, ENABLE); 	//使能定时器5，中断计时
			Data_RunSpeed = 2.0f; 	//低速运行一小段距离
		}	
		EXTI_ClearFlag(EXTI_Line6);
	}
	
	if(EXTI_GetFlagStatus(EXTI_Line5))  //右极限
	{
		Motor_Dir = DIR_STOP; //电机停止转动	
		Flag_Status_Motion = STATUS_REST;	 //当前为静止状态	
		EXTI_ClearFlag(EXTI_Line5);
	}
}

void TIM2_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET) 	//溢出中断
	{
		if(Last_Dir != Motor_Dir)  //方向改变
		{
			Last_Dir = Motor_Dir;
			switch(Last_Dir)
			{
				case DIR_STOP:
					Motor_Stop();  //电机停止转动	
					if(Flag_Init == DOING)  //正在上电复位
					{
						TIM5_Count = 2; 				//200ms*2之后进入中断，位置清零
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
		}
		else if(Last_Speed != Data_RunSpeed) //速度改变
		{	
			Last_Speed = Data_RunSpeed;
			//近似认为速度和频率呈线性关系，比例系数求得为4.13
			FREQ_Change_Freq(Last_Speed * 4.13f); //更改变频器的频率	
		}	
		TIM_Cmd(TIM2, DISABLE); //失能定时器2
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 	//清除中断标志位
	}
}

void TIM3_IRQHandler(void)
{	
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET) 	//溢出中断
	{	
		Increment = TIM4->CNT;
		TIM4->CNT = 0;
		
		//换算得到速度值：Increment*2.5mm/1024脉冲/20ms
		if(Increment == 0 || Increment < 512)
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
		if(Flag_Status_Motion == STATUS_AUTOMOD) //自动模式运动状态下
		{
			if(Data_Stage <= 3) //前四段运行中
			{			
				if(Encoder_Pulse_NUM >= 32768 * Data_Stage)  //运动超过了40mm，换速
				{
					Data_Stage++;  	 //段数加1，最多加到4						
				}
			}		
			if(Data_Stage <= 4)
			{
				PID_Control_SPD(160.03f, Present_Data_Speed[Data_Stage]); //更改运行速度，结束后进入中断			
			}		
			else if(Data_Stage == 5)  //回原点
			{
				PID_Control_SPD(0.03f, Present_Data_Speed[Data_Stage]); //更改运行速度，结束后进入中断	
			}				
		}				
		else if(Flag_Status_Motion == STATUS_TWOSTEP) //在复位的第二步状态中
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
		else if(Flag_Status_Motion == STATUS_THREESTEP) //按下了复位按钮
		{
			PID_Control_SPD(0.03f, 15.0f); //更改运行速度
		}
		else if(Flag_Status_Motion == STATUS_SINGMOD)
		{
			if(Data_Stage <= 4)  //段数小于等于4
			{
				PID_Control_SPD(Data_Stage*40.0f+0.03f, Data_Speed);				
			}
			else if(Data_Stage == 5)
			{
				PID_Control_SPD(0.03f, Data_BKSpeed);								
			}
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
			if((Flag_Init==DOING) && (Flag_Status_Motion==STATUS_REST))  	  //上电复位
 			{
				Encoder_Pulse_NUM = 0;  	//位置归零
				Flag_Init = DONE;    			//标志上电复位完成
				//按钮清空
				Button_Recover = 0; //复位按钮 
				Button_Stop = 0;    //停止按钮
				Button_Auto = 0;    //自动模式切换按钮
				Button_Single = 0;  //单步模式切换按钮
			    Button_Manual = 0;  //手动模式切换按钮
			}					
			if(Flag_Status_Motion == STATUS_ONESTEP) //当前处于第一步
			{
				Motor_Dir = DIR_RIGHT; 		//电机向右转动					
				Data_RunSpeed = 2.0f;			
				Flag_Status_Motion = STATUS_TWOSTEP; //进入第二步复位
			}
			else if(Flag_Status_Motion == STATUS_TWOSTEP) //达到中心点
			{
				Motor_Dir = DIR_STOP; 		//电机停止转动
				Flag_Status_Motion = STATUS_REST; //运动状态改为停止				
				Data_Stage = 0; 	//自动模式的段数改为0段
				Status_Return = 0; //返回状态灯置0		
			}		
			if(Motor_Dir == DIR_STOP) //消除抖动，200ms后依然是静止状态
			{
				if(Flag_Status_Motion == STATUS_SINGMOD)
				{
					Flag_Status_Motion = STATUS_REST; //运动状态改为停止
					if(Data_Stage == 5)
					{
						Data_Stage = 0;
					}
					Motor_Dir = DIR_STOP;
				}
				else if(Flag_Status_Motion == STATUS_THREESTEP)
				{
					Flag_Status_Motion = STATUS_REST; //运动状态改为停止
					Data_Stage = 0; //自动模式的段数改为0段
					Status_Return = 0; //返回状态灯灭
				}
				else if(Flag_Status_Motion == STATUS_AUTOMOD)
				{
					if((Data_Stage==4) && (Status_Wait==0)) //说明需要定时3s
					{
						Status_Wait = 1;	//等待指示灯亮
						TIM_Cmd(TIM5, ENABLE);  //使能定时器5
						TIM5_Count = 15;   			//3s定时
					}
					else if((Data_Stage==4) && (Status_Wait==1))
					{
						Status_Wait = 0;	//等待指示灯灭
						Status_Return = 1;//返回状态灯亮
						Data_Stage++; 		//第五段						
					}			
					else if(Data_Stage == 5)  //回原点结束了
					{
						Flag_Status_Motion = STATUS_REST; //运动状态改为停止	
						Data_Stage = 0; //自动模式的段数改为0段
						Status_Return = 0; //返回状态灯灭
					}
				}
			}			
		}		
	}
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 	 //清除中断标志位
}

