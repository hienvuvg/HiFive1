
#ifndef _MY_SPI_H_
#define _MY_SPI_H_

#include <stdint.h>
#include <stdio.h> 
#include "platform.h"
#include "encoding.h"
#include "mygpio.h"
#include "mycpu.h"

#define SPI_CS_PIN 2 // GPIO_2 default
//#define SPI_CS_PIN 23

#define SPI_MISO_PIN 4 
#define SPI_SCK_PIN 5 
#define SPI_MOSI_PIN 3

void SPI_Config(int frequency);
uint8_t SPI1_Soft_Receive(void);
void SPI1_Soft_Transmit(uint8_t data);
void SPI_Activate(void);
void SPI_Deactivate(void);

#endif