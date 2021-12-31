
#ifndef _MY_GPIO_H_
#define _MY_GPIO_H_

#include "platform.h"
#include "encoding.h"

void GPIO_On(int pin_name);
void GPIO_Off(int pin_name);
void GPIO_Toggle(int pin_name);
int GPIO_ReadInput(int pin_name);

void GPIO_Disable(int pin_type, int pin_name);
void GPIO_Enable(int pin_type, int pin_name);

void LED_On(int pin_name);
void LED_Off(int pin_name);
void LED_Toggle(int pin_name);

#endif