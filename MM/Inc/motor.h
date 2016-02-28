#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"

//
//HEADER
//

const float PI									= 3.14;
const int MOTOR_DRIVER_D_CNT 		= 1000;
const int MOTOR_EPSILON 				= 10; // enc tick
const int TICKS_PER_REVOLUTION	= 420;
const int HALF_WHEELBASE				= 100/2; // mm
const int WHEEL_DIAMETER 				= 45; // mm

const float KP = 1.0;
const float KD = 0.0;
const float KI = 0.0;


#define MOTOR_GPIO GPIOC

typedef enum
{
	MOTOR_STOPPED,
	MOTOR_RUNNING_STOP, // stop wheels when end
	MOTOR_RUNNING_FLOAT, // float wheels when end
	MOTOR_FLOATING
} MotorStat;

typedef struct
{
	int vel;
	uint16_t ePosL, ePosR;
	int velL, velR;
	MotorStat status;
	int (*condition)();
} Motors;


// Functions

// most usefull
void Go(int left, int right, int vel, int(*condition)());
void Turn(int angle, int radius, int vel, int(*condition)());
int MotorGoTo(Motors*);
void MotorSetVel(Motors*);

// encoder
int getEncL();
int getEncR();
int mmToTicks(int mm);

// driver calculations
void MotorDriverP(Motors*);
void MotorDriverD(Motors*);

//
//END HEADER
//

#endif