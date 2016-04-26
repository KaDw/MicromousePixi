#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"


extern uint32_t LFSensor;
extern uint32_t RFSensor;
extern uint32_t LSensor;
extern uint32_t RSensor;

// calibration
extern uint32_t cal_LFSensor;
extern uint32_t cal_RFSensor;
extern uint32_t cal_LSensor;
extern uint32_t cal_RSensor;

void ADC_read_channel(uint32_t channel, uint32_t *buf);
void ADC_read_ambient();

#endif
