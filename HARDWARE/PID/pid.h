#ifndef _PID_H
#define _PID_H

extern float P,I,D;

void PID_Init(void);

void PID_Control_SPD(float position, float speed);



#endif


