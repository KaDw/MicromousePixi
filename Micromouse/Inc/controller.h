#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "stm32f4xx_hal.h"

int speed_to_counts(int mm);
int counts_to_speed(int ticks);
void getEncoderStatus();
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd);
void speedProfile(void);
void moveOneCell();
	

#endif 