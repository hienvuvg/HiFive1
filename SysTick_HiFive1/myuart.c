#include "myuart.h"


void UART_Config(size_t baud_rate){
  // Word length default 8 bit
  // Parity default none
  // No Sampling, no prescaler
  // Transmit watermark level: None
  GPIO_REG(GPIO_IOF_SEL) &= ~IOF0_UART0_MASK; // Disable
  GPIO_REG(GPIO_IOF_EN) |= IOF0_UART0_MASK;  // Enable

  UART0_REG(UART_REG_DIV) = get_cpu_freq() / baud_rate - 1;
  UART0_REG(UART_REG_TXCTRL) |= UART_TXEN; // Transmit Enable
  UART0_REG(UART_REG_RXCTRL) |= UART_RXEN; // Receive Enable
  UART0_REG(UART_REG_TXCTRL) |= (0x1 << 1); // Number of stop bit: 1
  UART0_REG(UART_REG_RXCTRL) |= (0x1 << 1); // Number of stop bit: 1
  //UART0_REG(UART_REG_TXCTRL) &= ~(0x1 << 1); // Number of stop bit: 0
  UART0_REG(UART_REG_TXCTRL) |= UART_TXWM(1); // Receive Enable
  UART0_REG(UART_REG_RXCTRL) |= UART_RXWM(1); // Receive Enable
}

void UART_Transmit_Byte(uint8_t data){
  while(UART0_REG(UART_REG_TXFIFO) >> 31); // Check if the FIFO is full at bit 32
  UART0_REG(UART_REG_TXFIFO) = data;
}

void UART_Transmit(uint8_t* s){
    // transmit character until NULL is reached
    while(*s > 0) UART_Transmit_Byte(*s++);
}

uint8_t UART_Receive_Byte(void){
  uint8_t data;
  while(UART0_REG(UART_REG_RXFIFO) >> 31); // Check if the FIFO is empty at bit 32
  //my_delay_ms(100);
  data = UART0_REG(UART_REG_RXFIFO) & 0xFF;
  return data;
}

uint8_t UART_Receive(uint8_t (*s)[], int length){
  
    if (length != 11) return 0;
    int i = 0;
    while(UART0_REG(UART_REG_RXFIFO) >> 31); // Check if the FIFO is empty at bit 32
    while(i < length){
      (*s)[i] =  UART0_REG(UART_REG_RXFIFO) & 0xFF;
      my_delay_us(100);
      i++;
    }
    return 1;
}

uint8_t CorrectString(uint8_t (*A)[], uint8_t length, uint8_t shift){
    uint8_t j = 0;

    if (length != 11) return 0;
    
    for (uint8_t i = 0; i< (length - shift);i++){
        (*A)[i] = (*A)[i+shift];
    }
    (*A)[length - shift] = 0;
}

void float_print(float data){
  uint8_t pbuff[20];
  sprintf(pbuff,"%f\n", data);
  UART_Transmit(pbuff);
}

void int_print(int data){
  uint8_t pbuff[20];
  sprintf(pbuff,"%d\n", data);
  UART_Transmit(pbuff);
}

