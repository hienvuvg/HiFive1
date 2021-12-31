
#ifndef _MY_UART_H_
#define _MY_UART_H_

#include <stdio.h> // For uart
#include "platform.h"
#include "encoding.h"

void uart_init(size_t baud_rate);
void UART_Transmit_Byte(unsigned char data);
void UART_Transmit(char* s);

#endif