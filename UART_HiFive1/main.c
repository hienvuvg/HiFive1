/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2019 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : blinky.c
*/

asm(".global _printf_float");
asm(".global _scanf_float");

#include <stdint.h>
#include <stdio.h>
#include "platform.h"
#include "encoding.h"
#include "mygpio.h"
#include "myuart.h"
#include "mycpu.h"

#define BT1_OFFSET 20

void GPIO_Config(void){
  GPIO_Disable(GPIO_INPUT_EN, GREEN_LED_OFFSET);
  GPIO_Disable(GPIO_INPUT_EN, BLUE_LED_OFFSET);
  GPIO_Disable(GPIO_INPUT_EN, RED_LED_OFFSET);

  GPIO_Enable(GPIO_OUTPUT_EN, GREEN_LED_OFFSET);
  GPIO_Enable(GPIO_OUTPUT_EN, BLUE_LED_OFFSET);
  GPIO_Enable(GPIO_OUTPUT_EN, RED_LED_OFFSET);

  GPIO_Enable(GPIO_INPUT_EN, BT1_OFFSET);
  GPIO_Disable(GPIO_OUTPUT_EN, BT1_OFFSET);
  GPIO_Enable(GPIO_PULLUP_EN, BT1_OFFSET);

  LED_Off(GREEN_LED_OFFSET);
  LED_Off(BLUE_LED_OFFSET);
  LED_Off(RED_LED_OFFSET);
}

void My_Delay(int ms){
    int temp = 0;
    while(ms--){
        temp = 1700;
        while(temp--);
    }
}


char n = 0;
float m = 0;
char buff[100];
float adc_read = 0;

void cleanbuff(void){
    for (int i=0; i < 100; i++ ){
        buff[i] = 0;
    }
}

void printbuff(void){
    uint8_t string_len = 1;
    for(int i = 0; buff[i] != '\n'; i++){
        string_len++;
    }
    UART_Transmit(buff);
}


/*
void float_print(void){
  cleanbuff();

  adc_read = m++/7.0f;

  char *tmpSign = (adc_read < 0) ? "-" : "";
  float tmpVal = (adc_read < 0) ? -adc_read : adc_read;

  int tmpInt1 = tmpVal;                  // Get the integer (678).
  float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
  int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

  // Print as parts, note that you need 0-padding for fractional bit.

  sprintf (buff, "Value = %s%d.%d\n", tmpSign, tmpInt1, tmpInt2);

  UART_puts(buff);
}
*/

uint8_t data;
uint8_t temp[100];

void main(void)
{
  UART_Config(115200); // Max 1MHz
  //UART_Config(1000000); // Max 1MHz
  GPIO_Config();
  Delay_Config();
  uint8_t* sa = "damn it";
  UART_Transmit(sa);

  while(1){
    int_print(4);
    float_print(3.1415);
    LED_Toggle(GREEN_LED_OFFSET);
    My_Delay(1000);
    /*
    data = UART_Receive_Byte();
    LED_Toggle(GREEN_LED_OFFSET);
    UART_Transmit_Byte(data);
    LED_Toggle(RED_LED_OFFSET);
    */

    //UART_Receive(&temp,11);
    //CorrectString(&temp,11);
    //UART_Transmit(temp);

  }
}

/*************************** End of file ****************************/
