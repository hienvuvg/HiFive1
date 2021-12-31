#include "mycpu.h"

int delay_ratio_ms = 300000000;
int delay_ratio_us = 300000;
int delay_ratio_ns = 300;

void use_hfrosc(int div, int trim){
  // Make sure the HFROSC is running at its default setting
  PRCI_REG(PRCI_HFROSCCFG) = (ROSC_DIV(div) | ROSC_TRIM(trim) | ROSC_EN(1));
  while ((PRCI_REG(PRCI_HFROSCCFG) & ROSC_RDY(1)) == 0) ;
  PRCI_REG(PRCI_PLLCFG) &= ~PLL_SEL(1);
}

void Clock_Default(){
  // Turn off the LFROSC
  AON_REG(AON_LFROSC) &= ~ROSC_EN(1);

  // Use HFROSC
  use_hfrosc(4, 16);
}

void Delay_Config(void){
  delay_ratio_ms = get_cpu_freq()/ 10000; // Trial and error
  delay_ratio_us = delay_ratio_ms/1000;  // Trial and error
  delay_ratio_ns = delay_ratio_us/1000;  // Trial and error
}

void my_delay_ms(int ms){
  int temp = 0;
  while(ms--){
    temp = delay_ratio_ms;
    while(temp--);
  }
}

void my_delay_us(int us){
  int temp = 0;
  while(us--){
    temp = delay_ratio_us;
    while(temp--);
  }
}

void my_delay_ns(int ns){
  int temp = 0;
  while(ns--){
    temp = delay_ratio_ns;
    while(temp--);
  }
}

void SysTick_Config(void){

  AON_REG(AON_RTCCFG) &= ~(1 << 0); 
  AON_REG(AON_RTCCFG) &= ~(1 << 1); 
  AON_REG(AON_RTCCFG) &= ~(1 << 2); 
  AON_REG(AON_RTCCFG) &= ~(1 << 3); 
  //AON_REG(AON_RTCCFG) |= AON_RTCCFG_SCALE; // Counter scale value from rtchi and rtclo to rtcs; rtcs increases every second
  AON_REG(AON_RTCCFG) |= 0x05; // Counter scale value from rtchi and rtclo to rtcs; rtcs increases every millisecond
  
  AON_REG(AON_RTCCFG) |= AON_RTCCFG_ENALWAYS;
  
  AON_REG(AON_RTCLO) = 0;
  AON_REG(AON_RTCHI) = 0;
}