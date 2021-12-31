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
#include <stdio.h> 
#include "platform.h"
#include "encoding.h"
#include "mygpio.h"
#include "myuart.h"
#include "mpu9250.h"
#include "myspi.h"
#include "mycpu.h"


#define BT1_OFFSET 20

#define M_PI 3.14159265358979323846

int n = 0;
const uint8_t string_length = 100;
int8_t buff[100];

const float G = 9.807;

int16_t AccRaw[3], GyroRaw[3], MagRaw[3];
float CorrectedAcc[3], CorrectedGyro[3];
float CorrectedMag[3], AlignedMag[3][1];

float accScale, gyroScale;

uint8_t gyro_range = GYRO_RANGE;
uint8_t acc_range = ACCEL_RANGE;
uint8_t temprx = 0;


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

void RawPrint(void){
	cleanbuff();
	sprintf(buff,"Raw\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
                      GyroRaw[0], GyroRaw[1], GyroRaw[2],
                      AccRaw[0], AccRaw[1], AccRaw[2],
                      MagRaw[0], MagRaw[1], MagRaw[2]);
	printbuff();
}

void FloatPrint(float var){
	cleanbuff();
	sprintf(buff,"%f\n",var);
	printbuff();
}

void HexPrint(int var){
	cleanbuff();
	sprintf(buff,"Hex: 0x%x   Dec: %d\n",var , var);
	printbuff();
}

void DataPrint(void){
	cleanbuff();
	sprintf(buff,"%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.3f\t%4.0f\t%4.0f\t%4.0f\n",
                    CorrectedGyro[0], CorrectedGyro[1], CorrectedGyro[2],
                    CorrectedAcc[0], CorrectedAcc[1], CorrectedAcc[2],
                    CorrectedMag[0], CorrectedMag[1], CorrectedMag[2]);
	printbuff();
}

void setAccelRange(uint8_t range) {
	if (range == ACCEL_FS_SEL_2G) accScale = G * 2.0/32767.5;
	if (range == ACCEL_FS_SEL_4G) accScale = G * 4.0/32767.5;
	if (range == ACCEL_FS_SEL_8G) accScale = G * 8.0/32767.5;
	if (range == ACCEL_FS_SEL_16G) accScale = G * 16.0/32767.5;
}

void setGyroRange(uint8_t range) {
	if (range == GYRO_FS_SEL_250DPS) gyroScale = 250.0*M_PI/32767.5/180.0;
	if (range == GYRO_FS_SEL_500DPS) gyroScale = 500.0*M_PI/32767.5/180.0;
	if (range == GYRO_FS_SEL_1000DPS) gyroScale = 1000.0*M_PI/32767.5/180.0;
	if (range == GYRO_FS_SEL_2000DPS) gyroScale = 2000.0*M_PI/32767.5/180.0;
}

void ConvertIMUData(void){
	for (int i=0; i<3; i++ ){
		CorrectedGyro[i]= (double)(GyroRaw[i]*gyroScale);
	}
	for (int i=0; i<3; i++ ){
		CorrectedAcc[i]= (double)(AccRaw[i]*accScale);
	}
}

void GPIO_Config(void){
	
  GPIO_Enable(GPIO_INPUT_EN, BT1_OFFSET);
  GPIO_Disable(GPIO_OUTPUT_EN, BT1_OFFSET);
  GPIO_Enable(GPIO_PULLUP_EN, BT1_OFFSET);

  LED_Off(GREEN_LED_OFFSET);
  LED_Off(BLUE_LED_OFFSET);
  LED_Off(RED_LED_OFFSET);
}


void main(void)
{
  //Clock_Default();
  Delay_Config();
  GPIO_Config();
  UART_Config(115200);

  SPI_Config(500000);
  
  setAccelRange(acc_range);
  setGyroRange(gyro_range);

  while(!MPU9250_Init());
  
  //LED_On(GREEN_LED_OFFSET);

  while(1){

    //writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR);
    //HexPrint(whoAmI());
    //writeRegister(0x81,0x81);
    //HexPrint(whoAmIAK8963());
    MPU9250_GetData(GyroRaw, AccRaw, MagRaw); RawPrint();
    //MPU9250_GetData(GyroRaw, AccRaw, MagRaw); ConvertIMUData(); DataPrint();
  }
}

/*************************** End of file ****************************/
