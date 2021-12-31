#include "mygpio.h"

void GPIO_On(int pin_name){GPIO_REG(GPIO_OUTPUT_VAL) |= (0x1 << pin_name);}
void GPIO_Off(int pin_name){GPIO_REG(GPIO_OUTPUT_VAL) &= ~(0x1 << pin_name);}
void GPIO_Toggle(int pin_name){GPIO_REG(GPIO_OUTPUT_VAL) ^= (0x1 << pin_name);}
int GPIO_ReadInput(int pin_name){return ((GPIO_REG(GPIO_INPUT_VAL) & (0x1 << pin_name)) >> pin_name);}

void GPIO_Disable(int pin_type, int pin_name){GPIO_REG(pin_type) &= ~(0x1 << pin_name);}
void GPIO_Enable(int pin_type, int pin_name){GPIO_REG(pin_type) |= (0x1 << pin_name);}

void LED_On(int pin_name){GPIO_Off(pin_name);}
void LED_Off(int pin_name){GPIO_On(pin_name);}
void LED_Toggle(int pin_name){GPIO_Toggle(pin_name);}