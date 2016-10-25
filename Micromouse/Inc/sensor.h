#ifndef SENSOR_H_
#define SENSOR_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

// safe min max functions
#define MAX(a,b) \
  ({ __typeof__ (a) _a = (a); \
      __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b; })

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

/*
sens[0] - LF - Left
sens[1] - RF - Right
sens[3] - L - Left Cross
sens[4] - R - Right Cross
sens[5] - LS - Left Side
sens[6] - RS - Right Side

	L		LF	RF		R
	\		|		|   /
	 \	|		|  /
		\	|		| /
LS----		 ---- RS

*/
#define SENS_LF sens[0]
#define SENS_RF sens[1]
#define SENS_L  sens[2]
#define SENS_R  sens[3]
#define SENS_LS sens[4]
#define SENS_RS sens[5]



// calibration
//typedef struct{
//	uint32_t sens;
//	uint32_t buf[3];
//} _sensor;
//extern _sensor sensor[6];
extern uint32_t cal[7];
// sensor data
extern uint32_t sens[6];
//extern uint32_t fuzzy[6];
extern uint32_t read[6];
extern volatile uint32_t vbat;
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

uint32_t LinADC(uint32_t* sens);
void ADCreadAmbient(void);	
void ADCreadChannel(uint8_t channel, uint32_t *buf);
void ADCread2Channel(uint8_t CHx1, uint8_t CHx2, uint32_t *buf);
void SensorOff(void);
//uint32_t Sort(_sensor sensor);
//void Move(_sensor *sensor);
#endif
