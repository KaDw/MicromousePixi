#ifndef __TINYLIB_H__
#define __TINYLIB_H__

#include "stm32f4xx.h"
#include "gpio.h"


#define PI									3.14159265359


int abs(int);

int8_t sgn(int);

void _delay_us(int t);

void _delay_ms(int t);
#endif
