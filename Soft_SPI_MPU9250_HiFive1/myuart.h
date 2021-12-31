
#ifndef _MY_UART_H_
#define _MY_UART_H_

#include <stdio.h> // For uart
#include <stdint.h>
#include "platform.h"
#include "encoding.h"
#include "mycpu.h"

void UART_Config(size_t baud_rate);
void UART_Transmit_Byte(uint8_t data);
uint8_t UART_Receive_Byte(void);
void UART_Transmit(uint8_t* s);
uint8_t UART_Receive(uint8_t (*s)[], int length);
uint8_t CorrectString(uint8_t (*A)[], uint8_t length, uint8_t shift);
void float_print(float data);
void int_print(int data);

#endif