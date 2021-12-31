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

#include <stdint.h>
#include "mygpio.h"

#define BT1_OFFSET 20

int n = 0;
int led = GREEN_LED_OFFSET;

void My_Delay(int ms){
	int temp = 0;
	while(ms--){
		temp = 30000;
		while(temp--);
	}
}

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

void main(void)
{
  GPIO_Config();

  while(1){

    if (!GPIO_ReadInput(BT1_OFFSET)) {
      while(!GPIO_ReadInput(BT1_OFFSET));

      LED_Off(led);

      if (led == GREEN_LED_OFFSET) led = RED_LED_OFFSET;
      else if (led == RED_LED_OFFSET) led = BLUE_LED_OFFSET;
            else if (led == BLUE_LED_OFFSET) led = GREEN_LED_OFFSET;
    }
    
    LED_On(led);
    My_Delay(500);
    LED_Off(led);
    My_Delay(500);

  }
}

/*************************** End of file ****************************/
