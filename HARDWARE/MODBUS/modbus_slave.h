#ifndef _MODBUS_SLAVE_H
#define _MODBUS_SLAVE_H
#include "sys.h"

extern u16     Present_Page;    			//默认为一画面

extern u16     Page1_Status_Recover;  //复位指示灯 
extern u16     Page1_Button_Recover;  //复位按钮 
extern u16     Page1_Button_Auto;     //自动模式切换按钮
extern u16     Page1_Button_Single;   //单步模式切换按钮
extern u16     Page1_Button_Manual;   //手动模式切换按钮

extern u16     Page2_Button_Start;    //启动按钮 
extern u16     Page2_Button_Stop;     //停止按钮 
extern u16     Page2_Button_BackMain; //返回主界面按钮 
extern u16     Page2_Status_Alarm;    //警报指示灯
extern u16     Page2_Status_Pause;    //暂停指示灯
extern u16     Page2_Status_Return;   //返回指示灯
extern u16     Page2_Status_Wait;     //等待指示灯
extern u16     Page2_Status_EndLeft;  //左极限指示灯
extern u16     Page2_Status_EndRight; //右极限指示灯
extern u16     Page2_Status_DirLeft;  //左运动指示灯
extern u16     Page2_Status_DirRight; //右运动指示灯

extern float   Page2_Data_RTSpeed; 	  //实时速度
extern float   Page2_Data_Location;   //实时位置
extern float   Page2_Data_Speed1;     //段速1
extern float   Page2_Data_Speed2;     //段速2
extern float   Page2_Data_Speed3;     //段速3
extern float   Page2_Data_Speed4;     //段速4
extern float   Page2_Data_BKSpeed;    //返回速度

extern u16     Page3_Data_Stage;   		//单步段数
extern float   Page3_Data_Speed; 			//单步段速

extern float   Page4_Data_RunSpeed; 	//运行速度
extern u16     Page4_Button_Left;     //左运动按钮 
extern u16     Page4_Button_Right;    //右运动按钮 
extern int     Page4_Data_Animation;  //动画位置



void Modbus_Parse(void);

u16 CRC16(u8 *addr, int num);  

void Modbus_03_Solve(void);
void Modbus_05_Solve(void);
void Modbus_06_Solve(void);
void Modbus_10_Solve(void);

void Modbus_RegMap(void);

#endif
