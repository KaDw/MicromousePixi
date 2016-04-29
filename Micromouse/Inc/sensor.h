#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"



// calibration
extern uint32_t cal[4];
// sensor data
extern uint32_t sens[4];


typedef enum {
	CH2 = 2,
	CH3,
	CH10 = 10,
	CH11,
	CH12,
	CH13
} CHx;

void ADC_read_ambient(void);	
void ADC_read_channel(uint8_t channel, uint32_t *buf);
void ADC_read_2channel(uint8_t CHx1, uint8_t CHx2, uint32_t *buf);

#endif
