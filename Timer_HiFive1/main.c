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
#include "platform.h"
#include "encoding.h"
#include "mygpio.h"
#include "myuart.h"

int n = 0;
const uint8_t string_length = 100;
int8_t buff[100];

#define BT1_OFFSET 20

void GPIO_Config(void){
	
  GPIO_Enable(GPIO_INPUT_EN, BT1_OFFSET);
  GPIO_Disable(GPIO_OUTPUT_EN, BT1_OFFSET);
  GPIO_Enable(GPIO_PULLUP_EN, BT1_OFFSET);

  LED_Off(GREEN_LED_OFFSET);
  LED_Off(BLUE_LED_OFFSET);
  LED_Off(RED_LED_OFFSET);
}

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

void VarPrint(int var1, int var2){
	cleanbuff();
	sprintf(buff,"n: %d   i: %d\n",var1 , var2);
	printbuff();
}


static void
schedule_timer_interrupt(unsigned ticks)
{
  volatile uint64_t *mtime = (uint64_t *)(CLINT_CTRL_ADDR + CLINT_MTIME);
  volatile uint64_t *mtimecmp = (uint64_t *)(CLINT_CTRL_ADDR + CLINT_MTIMECMP);
  *mtimecmp = *mtime + ticks;
}

void handle_m_time_interrupt(){

  // Configure next timer interrupt
  schedule_timer_interrupt(RTC_FREQ);

  // Toggle LED
  GPIO_REG(GPIO_OUTPUT_VAL) ^= (0x1 << GREEN_LED_OFFSET);
  GPIO_REG(GPIO_OUTPUT_VAL) ^= (0x1 << 18);

  n++;
}

uintptr_t handle_trap(uintptr_t mcause, uintptr_t epc){

  if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_TIMER))
    handle_m_time_interrupt();
  else
    while (1);
  return epc;
}

void main(void){

  int i;
  
  GPIO_Config();
  uart_init(115200);

  // Clear interrupts
  clear_csr(mie, MIP_MEIP);
  clear_csr(mie, MIP_MTIP);

  // Configure next timer interrupt
  schedule_timer_interrupt(RTC_FREQ);

  // Enable the Machine-Timer bit in MIE
  set_csr(mie, MIP_MTIP);

  // Enable interrupts in general.
  set_csr(mstatus, MSTATUS_MIE);
  
  i = 0;

  while(1) {
    i++; // it increases each millisecond, super slow
    VarPrint(n,i);
  }
}

/*************************** End of file ****************************/
