#ifndef _MODBUS_MASTER_H
#define _MODBUS_MASTER_H

void FREQ_Change_Freq(float val);

void Motor_Stop(void);
void Motor_Right(void);
void Motor_Left(void);

void Modbus2_Parse(void);
void Modbus2_03_Solve(void);
void Modbus2_06_Solve(void);
void Modbus2_08_Solve(void);


#endif


