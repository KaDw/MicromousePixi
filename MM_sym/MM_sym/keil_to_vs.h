#pragma once

#define WIN32
#ifdef WIN32
#include <cmath>
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef signed short int int16_t;
typedef unsigned short int uint16_t;

typedef enum
{
	MOTOR_STOPPED,
	MOTOR_RUNNING_STOP, // motor is running now, stop wheels when end running
	MOTOR_RUNNING_FLOAT, // motor is running now, float wheels when end running
	MOTOR_FLOATING,
	MOTOR_CONST_VEL
} MotorStat;


struct _Motors_t;
struct _Motors_t
{
	int vel;
	uint16_t ePosL, ePosR;
	int velL, velR;
	MotorStat status;
	int(*driver)(struct _Motors_t*);
};
typedef struct _Motors_t Motors_t;

void TurnA(Motors_t* m, int angle, int radius, int vel, int(*driver)(Motors_t*));
void GoA(Motors_t* m, int left, int right, int vel, int(*driver)(Motors_t*));
#endif