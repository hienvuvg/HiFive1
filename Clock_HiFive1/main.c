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
#include <stdio.h> // For uart
#include "platform.h"
#include "encoding.h"
#include "mygpio.h"

//#include "sifive/devices/uart.h"

// $(ProjectDir)/HiFive1_RevB/freedom-e-sdk/bsp/include/sifive/devices

int cpu_freq = 0, timer_freq = 0;

static void
schedule_timer_interrupt(unsigned ticks)
{
  volatile uint64_t *mtime = (uint64_t *)(CLINT_CTRL_ADDR + CLINT_MTIME);
  volatile uint64_t *mtimecmp = (uint64_t *)(CLINT_CTRL_ADDR + CLINT_MTIMECMP);
  *mtimecmp = *mtime + ticks;
}

void
handle_m_time_interrupt()
{
  // Configure next timer interrupt
  schedule_timer_interrupt(RTC_FREQ);

  // Toggle LED
  GPIO_REG(GPIO_OUTPUT_VAL) ^= (0x1 << GREEN_LED_OFFSET);
  GPIO_REG(GPIO_OUTPUT_VAL) ^= (0x1 << 18);
}

uintptr_t handle_trap(uintptr_t mcause, uintptr_t epc)
{
  if ((mcause & MCAUSE_INT) && ((mcause & MCAUSE_CAUSE) == IRQ_M_TIMER))
    handle_m_time_interrupt();
  else
    while (1);
  return epc;
}

static void uart_init(size_t baud_rate)
{
  GPIO_REG(GPIO_IOF_SEL) &= ~IOF0_UART0_MASK; // Disable
  GPIO_REG(GPIO_IOF_EN) |= IOF0_UART0_MASK;  // Enable
  UART0_REG(UART_REG_DIV) = get_cpu_freq() / baud_rate - 1;
  UART0_REG(UART_REG_TXCTRL) |= UART_TXEN; // Enable
}


/////
static void use_hfrosc(int div, int trim)
{
  // Make sure the HFROSC is running at its default setting
  PRCI_REG(PRCI_HFROSCCFG) = (ROSC_DIV(div) | ROSC_TRIM(trim) | ROSC_EN(1));
  while ((PRCI_REG(PRCI_HFROSCCFG) & ROSC_RDY(1)) == 0) ;
  PRCI_REG(PRCI_PLLCFG) &= ~PLL_SEL(1);
}

static void use_default_clocks()
{
  // Turn off the LFROSC
  AON_REG(AON_LFROSC) &= ~ROSC_EN(1);

  // Use HFROSC
  use_hfrosc(4, 16);
}
/////

int main(void){
  //use_default_clocks();
  uart_init(115200);
  //use_pll(0, 0, 1, 31, 1); // 300MHz
  while(1){
    cpu_freq = get_cpu_freq();
    timer_freq = get_timer_freq();
  }
  return 0;
}
/*
void main(void)
{
  int i;

  // Configure LED 
  GPIO_REG(GPIO_INPUT_EN) &= ~(0x1 << GREEN_LED_OFFSET);
  GPIO_REG(GPIO_OUTPUT_EN) |= (0x1 << GREEN_LED_OFFSET);
  GPIO_REG(GPIO_OUTPUT_VAL) |= (0x1 << GREEN_LED_OFFSET);
  GPIO_REG(GPIO_INPUT_EN) &= ~(0x1 << 18);
  GPIO_REG(GPIO_OUTPUT_EN) |= (0x1 << 18);
  GPIO_REG(GPIO_OUTPUT_VAL) |= (0x1 << 18);
  
  // Clear interrupts
  clear_csr(mie, MIP_MEIP);
  clear_csr(mie, MIP_MTIP);

  // Configure next timer interrupt
  schedule_timer_interrupt(RTC_FREQ);

  // Enable the Machine-Timer bit in MIE
  set_csr(mie, MIP_MTIP);

  // Enable interrupts in general.
  set_csr(mstatus, MSTATUS_MIE);

  do {
    i++;
  } while (1);
}
*/

/*************************** End of file ****************************/
