#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"



// calibration
extern uint32_t cal[5];
// sensor data
extern uint32_t sens[4];
extern uint32_t vbat;

typedef enum {
	CH2 = 2,
	CH3,
	CH9 = 9,
	CH10,
	CH11,
	CH12,
	CH13
} CHx;

void ADCreadAmbient(void);	
void ADCreadChannel(uint8_t channel, uint32_t *buf);
void ADCread2Channel(uint8_t CHx1, uint8_t CHx2, uint32_t *buf);

#endif
