#ifndef _CONTROL_H
#define _CONTROL_H
#include "sys.h"

extern vu8   TIM5_Count;
extern u8    Motor_Dir;
extern float EXP_LOC;
	
extern vu8   Flag_Status_Motion;	
extern vu8   Flag_Init; 	//当前为上电复位状态

#define DOING       					0  //上电复位状态
#define DONE       						1  //上电复位完成状态

//电机复位状态
#define STATUS_REST       		0  //电机处在静止状态
#define STATUS_ONESTEP   			1  //复位第一步进行中
#define STATUS_TWOSTEP   			2  //复位第二步进行中
#define STATUS_THREESTEP   		3  //复位第三步进行中
#define STATUS_AUTOMOD   			4  //自动进行中
#define STATUS_SINGMOD   			5  //单步进行中

#define PAGE_ONE        			1  //第一页
#define PAGE_TWO       			  2  //第二页
#define PAGE_THREE       			3  //第三页
#define PAGE_FOUR        			4  //第四页

#define DIR_STOP			   			1  //停止
#define DIR_LEFT   						2  //向左运动
#define DIR_RIGHT  						3  //向右运动

#define CHG_NONE			   			0  //不改变
#define CHG_DIR			   			  1  //改变方向
#define CHG_SPD			   			  2  //改变速度

void Mode_Control(void);
void Motor_Restore(void);

void FreqChg_Control(void);


#endif


