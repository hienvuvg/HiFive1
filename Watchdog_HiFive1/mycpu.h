
#ifndef _MY_CPU_H_
#define _MY_CPU_H_

#include "platform.h"
#include "encoding.h"

void use_hfrosc(int div, int trim);

void Clock_Default();

void Delay_Config(void);

void my_delay_ms(int ms);

void my_delay_us(int us);

void my_delay_ns(int ns);

void SysTick_Config(void);

#endif