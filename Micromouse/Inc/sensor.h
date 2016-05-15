#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"



// calibration
extern uint32_t cal[7];
// sensor data
extern uint32_t sens[6];
extern uint32_t vbat;
extern uint8_t batError;

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
void SensorOff(void);

#endif
