#include "myspi.h"

int spi_delay = 1000; // Random

void SPI_Config(int frequency){ // Precise at 300MHz, input shoud be half of the real one when using default slow system clock frequency
  // CS output
  GPIO_Disable(GPIO_INPUT_EN, SPI_CS_PIN);
  GPIO_Enable(GPIO_OUTPUT_EN, SPI_CS_PIN);
  GPIO_On(SPI_CS_PIN); // Deactive

  // MISO input
  GPIO_Disable(GPIO_OUTPUT_EN, SPI_MISO_PIN);
  GPIO_Enable(GPIO_INPUT_EN, SPI_MISO_PIN);
  GPIO_Enable(GPIO_PULLUP_EN, SPI_MISO_PIN);

  // clock output
  GPIO_Disable(GPIO_INPUT_EN, SPI_SCK_PIN);
  GPIO_Enable(GPIO_OUTPUT_EN, SPI_SCK_PIN);
  GPIO_Off(SPI_SCK_PIN); // Deactive

  // MOSI output
  GPIO_Disable(GPIO_INPUT_EN, SPI_MOSI_PIN);
  GPIO_Enable(GPIO_OUTPUT_EN, SPI_MOSI_PIN);
  GPIO_Off(SPI_MOSI_PIN); // Deactive

  spi_delay = get_cpu_freq()/(50*frequency);
}

///////////////////////////

void SPI1_Soft_Transmit(uint8_t data){
  for(int i = 0; i < 8; i++){
    GPIO_Off(SPI_SCK_PIN); // Off clock

    if (data & 0x80) GPIO_On(SPI_MOSI_PIN);
    else GPIO_Off(SPI_MOSI_PIN);
    data = data << 1;

    my_delay_ns(spi_delay);
    GPIO_On(SPI_SCK_PIN); // On clock
    my_delay_ns(spi_delay);
  }
  GPIO_Off(SPI_SCK_PIN); // Off clock
  my_delay_ns(spi_delay);

  GPIO_Off(SPI_MOSI_PIN);
  my_delay_ns(spi_delay*3);
}

uint8_t SPI1_Soft_Receive(void){
    uint8_t data = 0; 

    for(int i = 0; i < 8; i++){
      GPIO_On(SPI_SCK_PIN); // On clock
      data = data << 1;
      data = data | GPIO_ReadInput(SPI_MISO_PIN);
      my_delay_ns(spi_delay);
      GPIO_Off(SPI_SCK_PIN); // Off clock
      my_delay_ns(spi_delay);
    }
    my_delay_ns(spi_delay*3);
    return data;
}

void SPI_Activate(void){
	GPIO_Off(SPI_CS_PIN);
        my_delay_us(2);
}

void SPI_Deactivate(void){
	GPIO_On(SPI_CS_PIN);
        my_delay_us(5);
}

/////////////////////////////////////

