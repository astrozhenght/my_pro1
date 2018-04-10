#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"

extern vu8   TIM5_Count;
extern u8    Motor_Dir; 	//电机方向
extern float EXP_LOC;

extern vu8   Status_Motion;	
extern vu8   Flag_Init;

#define DOING       			0  //上电复位状态
#define DONE       				1  //上电复位完成状态

//系统状态
#define STATUS_REST       		0  //静止状态
#define STATUS_ONESTEP   		1  //复位第一步状态
#define STATUS_TWOSTEP   		2  //复位第二步状态
//#define STATUS_THREESTEP   		3  //复位第三步状态
#define STATUS_AUTOMOD   		4  //自动状态
#define STATUS_SINGMOD   		5  //单步状态

#define MODE_SELECT        		1  //模式选择
#define MODE_AUTO       		2  //自动模式
#define MODE_SINGLE       		3  //单步模式
#define MODE_MANUAL        		4  //手动模式

#define DIR_STOP			   	1  //停止
#define DIR_LEFT   				2  //向左运动
#define DIR_RIGHT  				3  //向右运动

void Mode_Control(void);
void Motor_Restore(void);

void FreqChg_Control(void);


#endif


