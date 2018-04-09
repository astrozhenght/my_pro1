#ifndef _MODBUS_SLAVE_H
#define _MODBUS_SLAVE_H
#include "sys.h"

extern u16   Present_Mode;    //默认为模式选择
extern u16   Status_Recover;  //复位指示灯 
extern u16   Button_Recover;  //复位按钮 
extern u16   Button_Auto;     //自动模式切换按钮
extern u16   Button_Single;   //单步模式切换按钮
extern u16   Button_Manual;   //手动模式切换按钮

extern u16   Button_Start;    //启动按钮 
extern u16   Button_Stop;     //停止按钮 
extern u16   Button_BackMain; //返回主界面按钮 
extern u16   Status_Alarm;    //警报指示灯
extern u16   Status_Pause;    //暂停指示灯
extern u16   Status_Return;   //返回指示灯
extern u16   Status_Wait;     //等待指示灯
extern u16   Status_EndLeft;  //左极限指示灯
extern u16   Status_EndRight; //右极限指示灯
extern u16   Status_DirLeft;  //左运动指示灯
extern u16   Status_DirRight; //右运动指示灯

extern float Data_RTSpeed; 	//实时速度
extern float Data_Location;   //实时位置
extern float Data_Speed1;     //段速1
extern float Data_Speed2;     //段速2
extern float Data_Speed3;     //段速3
extern float Data_Speed4;     //段速4
extern float Data_BKSpeed;    //返回速度

extern u16   Data_Stage;   	//单步段数
extern float Data_Speed; 		//单步段速

extern float Data_RunSpeed; 	//运行速度
extern u16   Button_Left;     //左运动按钮 
extern u16   Button_Right;    //右运动按钮 
extern int   Data_Animation;  //动画位置

void Modbus_Parse(void);

u16 CRC16(u8 *addr, int num);  

void Modbus_03_Solve(void);
void Modbus_05_Solve(void);
void Modbus_06_Solve(void);
void Modbus_10_Solve(void);

void Modbus_RegMap(void);

#endif
