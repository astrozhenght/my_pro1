#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED1 					GPIO_Pin_5
#define LED2 					GPIO_Pin_4
#define LED3 					GPIO_Pin_3
#define LED4 					GPIO_Pin_2

#define LED_ON(X) 		GPIO_ResetBits(GPIOH, X) 
#define LED_OFF(X) 		GPIO_SetBits(GPIOH, X) 

void LED_Init(void);
void LED_Toggle(u16 LEDx);


#endif

