// @author:		Karol Trzciński
// @date:		march 2016
// @project:	MicroMouse
// @brief:		Control Motor Module, with complex driver and encoder routine
//				In this file there is no decision function

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"
#include "UI.h"
//#include "tinylib.h"

#define PI  3.1415926535f

// position controller
#define MOTOR_MAX_PWM					999
#define MAX_ANGULAR_VEL				200.7f /* deg/s */
#define WHEEL_DIAMETER 				37.0f  /* mm    */
#define TICKS_PER_REVOLUTION	3520.0f

#define MOTOR_GPIO 						GPIOC
#define MOTOR_HTIM  					htim12
#define MOTOR_CH_L						TIM_CHANNEL_1
#define MOTOR_CH_R						TIM_CHANNEL_2

#define MOTOR_HTIM_ENC_L 			htim3
#define MOTOR_HTIM_ENC_R 			htim4


extern const float MOTOR_DRIVER_T;
extern const float HALF_WHEELBASE;

extern TIM_HandleTypeDef MOTOR_HTIM, MOTOR_HTIM_ENC_L, MOTOR_HTIM_ENC_R;


int abs(int);
int sgn(int);
float fast_sqrt(float x);


typedef enum
{
	MOTOR_STOPPED,
	MOTOR_RUNNING_STOP, // motor is running now, stop wheels when end running
	MOTOR_RUNNING_FLOAT, // motor is running now, float wheels when end running
	MOTOR_FLOATING,
	MOTOR_CONST_VEL,
	MOTOR_ELSE
} MotorStat;


//========================
//====== VELOCITY ========
//========================

// P-imp/T 
#define MOTOR_VELV_KP					5.f /* 136.1f */
#define MOTOR_VELV_KI					0.f /* 1888.f */
#define MOTOR_VELV_KD					0.8f /* -0.7899f */

// ACC_V [mm/s/s] 	ACC_W[rad/s/s]
#define MOTOR_ACC_V						(1300.0f)
#define MOTOR_ACC_W						5.0f // 174.532925 rad/s/s = 10000 deg/s/s

// flags determine sensor int turn
extern int _motor_flag;
#define SET(R,B)							(R|=B)
#define CLR(R,B)							(R&=~B)
#define GET(R,B)							(!!(R&B))
#define FLAG_PID							1
#define FLAG_GYRO							2
#define FLAG_SENSOR						4
#define FLAG_ENCODER					8
#define FLAG_FREEZE						16
#define ENABLE_PID()					(_motor_flag|=FLAG_PID)
#define ENABLE_GYRO() 				(_motor_flag|=FLAG_GYRO)
#define ENABLE_SENSOR() 			(_motor_flag|=FLAG_SENSOR)
#define ENABLE_ENCODER() 			(_motor_flag|=FLAG_ENCODER)
#define DISABLE_PID 					(_motor_flag&=~FLAG_PID)
#define DISABLE_GYRO 					(_motor_flag&=~FLAG_GYRO)
#define DISABLE_SENSOR 				(_motor_flag&=~FLAG_SENSOR)
#define DISABLE_ENCODER 			(_motor_flag&=~FLAG_ENCODER)
#define MOTOR_FREEZE_EN()			SET(_motor_flag,FLAG_FREEZE)
#define MOTOR_FREEZE_DIS()		CLR(_motor_flag,FLAG_FREEZE)
#define MOTOR_FREEZE()				GET(_motor_flag,FLAG_FREEZE)	

#define EncL									MOTOR_HTIM_ENC_L.Instance->CNT
#define EncR									MOTOR_HTIM_ENC_R.Instance->CNT

extern float sensorGyroW;

typedef struct
{
	int PWM; // [0..MOTOR_MAX_PWM]
	int vel, targetVel; // mm/s
	int16_t lastEnc, encChange;
	unsigned int enc, idealEnc;
	int errP, errI, errD;
	int intDisable; //[bool] interrupt Disable
	//float KP, KI, KD;
} _MotorV;

typedef struct
{
	_MotorV mot[2];
	unsigned int time;
	MotorStat status;
} MotorsV;

void MotorStepResponse(uint16_t PwmL, uint16_t PwmR, uint16_t time);

void MotorInit(void);
void MotorUpdate(void);
void MotorStop(void);
void MotorFloat(void);
void MotorSetPWM(void);
void MotorSetPWMRaw(int left, int right);
void MotorGo(int left, int right, float vel); // [mm] [mm] [mm/s]
void MotorGoA(int left, int right, float vel); // [mm] [mm] [mm/s]
void MotorTurn(int angle, int r, float vel);
void MotorTurnA(int angle, int r, float vel);
void MotorRotR90A(void);

#endif
