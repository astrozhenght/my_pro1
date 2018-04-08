#include "pid.h"
#include "control.h"
#include "modbus_slave.h"
#include "sys.h"
#include "timer.h"

//范围[0.0, 0.05]

float P,I,D;
float Err[2];
float SUM_Err = 0.0f;

void PID_Init(void)
{
	P = 6.0f;  	//响应速度
	I = 4.0f;   //静态误差
	D = 6.0f;   //阻尼力
}

/**
 *info:位置式PID
 *参数：期望位置，当前位置，最大速度
 *返回值：PID得到的速度值
**/
float PID_Calculate(float exp_distance, float now_distance, float limit)
{
	static u8 flag = 0;
	float output;
	Err[0] = exp_distance - now_distance;  //给定值-测量值，正转时误差是正值
	Err[1] = Err[0];

	//积分限幅
	if((Err[0]<0.1f) && (Err[0]>-0.1f))  //0.1mm以内开始计算积分项！！0.05不行
	{
		SUM_Err += Err[0];
		flag = 1;
	}
	else
	{
		flag = 0;
	}
	if(SUM_Err > 0.15f)  //最大增加0.6mm/s
	{
		SUM_Err = 0.15f;
	}
	else if(SUM_Err < -0.15f)
	{
		SUM_Err = -0.15f;
	}
	
	//积分项最大增大0.6mm/s
	output = P*Err[0] + flag*I*SUM_Err + D*(Err[0] - Err[1]);
	
	//输出限幅
	if(output > limit)  //大于最大值
		output = limit;
	else if(output < -limit) //小于最小值
		output = -limit;
	
	//限定符号
	if(output < 0)
		output *= -1.0f; 
	return output;
}

/**
 *需要给定位置和速度
 *位置：position
 *速度：speed
 *note：结束后TIM5重新判断是否结束
 *精度：+-0.03mm
**/
void PID_Control_SPD(float position, float speed)
{
	//pid运算后的值赋给速度
	Page4_Data_RunSpeed = PID_Calculate(position, Page2_Data_Location, speed);  //给定40mm，速度最大15mm/s
				
	if(Page2_Data_Location > (position+0.02f))
	{
		Motor_Dir = DIR_LEFT;  //电机左转
	}
	else if(Page2_Data_Location < (position-0.03f))
	{
		Motor_Dir = DIR_RIGHT; //电机右转
	}
	else //到了目标点，开启定时器，进入中断后判断状态是否是停止
	{	
		if(Motor_Dir != DIR_STOP)  //等于停止的时候进入一次！！！
		{
			TIM5_Count = 2;	  //400ms后
			TIM5->CNT = 0;    //向上计数，计数器值归零
			TIM_Cmd(TIM5, ENABLE); 	//使能定时器5，中断计时		
			Motor_Dir = DIR_STOP;  	//电机停止转动
		}			
	}
}

