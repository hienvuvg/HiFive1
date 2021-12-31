#include "myuart.h"


void uart_init(size_t baud_rate){
  // Word length default 8 bit
  // Parity default none
  // No Sampling, no prescaler
  // Transmit watermark level: None
  GPIO_REG(GPIO_IOF_SEL) &= ~IOF0_UART0_MASK; // Disable as original
  //GPIO_REG(GPIO_IOF_SEL) |= IOF0_UART0_MASK; // Enable
  GPIO_REG(GPIO_IOF_EN) |= IOF0_UART0_MASK;  // Enable
  UART0_REG(UART_REG_DIV) = get_cpu_freq() / baud_rate - 1;
  UART0_REG(UART_REG_TXCTRL) |= UART_TXEN; // Transmit Enable
  UART0_REG(UART_REG_TXCTRL) |= (0x1 << 1); // Number of stop bit: 1
  //UART0_REG(UART_REG_TXCTRL) &= ~(0x1 << 1); // Number of stop bit: 0
  UART0_REG(UART_REG_RXCTRL) |= UART_RXEN; // Receive Enable
}


void UART_Transmit_Byte(unsigned char data){
  while(!(!(UART0_REG(UART_REG_TXFIFO) & 0x80000000))); // Check if the FIFO is full at bit 32
  UART0_REG(UART_REG_TXFIFO) = data;
}

void UART_Transmit(char* s){
    // transmit character until NULL is reached
    while(*s > 0) UART_Transmit_Byte(*s++);
}
