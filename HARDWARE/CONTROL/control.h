#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"

#define DOING       			0  //上电复位状态
#define DONE       				1  //上电复位完成状态

//系统状态
#define STATUS_REST       		0  //静止状态
#define STATUS_ONESTEP   		1  //复位第一步状态
#define STATUS_TWOSTEP   		2  //复位第二步状态
#define STATUS_AUTOMOD   		3  //自动状态
#define STATUS_SINGMOD   		4  //单步状态

#define MODE_SELECT        		1  //模式选择
#define MODE_AUTO       		2  //自动模式
#define MODE_SINGLE       		3  //单步模式
#define MODE_MANUAL        		4  //手动模式

#define DIR_STOP			   	1  //停止
#define DIR_LEFT   				2  //向左运动
#define DIR_RIGHT  				3  //向右运动

#define RECEIVE2			    1 
#define SEND2    				2  
#define WAITING    				3  

extern vu8   TIM5_Count;
extern float EXP_LOC;

extern vu8   Status_Motion;	
extern vu8   Flag_Init;
extern vu8   order_type;   //变频器命令类型

extern float Last_Speed;   //上一次的速度值，如果和当前的速度不相等就要改变速度
extern vu8   Motor_Dir;    //电机当前方向，默认停止
extern vu8   Last_Dir;     //电机上次的方向，默认停止

void Mode_Control(void);
void Motor_Restore(void);

void FreqChg_Control(void);
void FreqRev_Deal(void);  //发送完成之后进入


#endif


