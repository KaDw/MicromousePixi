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

#define PI 3.14

// position controller
#define MOTOR_MAX_VEL					1000
#define MOTOR_ACC							5
#define MOTOR_DRIVER_FREQ			1000
#define MOTOR_EPSILON 				15 /* enc tick 15~1mm*/
#define MOTOR_SLOW_TICK				200
#define MOTOR_SLOW_VEL				200
#define TICKS_PER_REVOLUTION	1760
#define HALF_WHEELBASE				(66/2) /* mm*/
#define WHEEL_DIAMETER 				37 /* mm*/

#define MOTOR_GPIO 						GPIOC
#define MOTOR_HTIM  					htim12
#define MOTOR_CH_L						TIM_CHANNEL_1
#define MOTOR_CH_R						TIM_CHANNEL_2

#define MOTOR_HTIM_ENC_L 			htim3
#define MOTOR_HTIM_ENC_R 			htim4

extern TIM_HandleTypeDef MOTOR_HTIM, MOTOR_HTIM_ENC_L, MOTOR_HTIM_ENC_R;


int abs(int);


typedef enum
{
	MOTOR_STOPPED,
	MOTOR_RUNNING_STOP, // motor is running now, stop wheels when end running
	MOTOR_RUNNING_FLOAT, // motor is running now, float wheels when end running
	MOTOR_FLOATING,
	MOTOR_CONST_VEL
} MotorStat;


//========================
//====== VELOCITY ========
//========================
#define MOTOR_VELV_KP					1.0f
#define MOTOR_VELV_KD					0.0f
#define MOTOR_VELW_KP					1.0f
#define MOTOR_VELW_KD					0.0f
#define MOTOR_ACC_V						5.0f
#define MOTOR_ACC_W						5.0f

// flags determine sensor int turn
extern int _motor_flag;
#define FLAG_PID							1
#define FLAG_GYRO							2
#define FLAG_SENSOR						4
#define FLAG_ENCODER					8
#define ENABLE_PID 						(_motor_flag|=FLAG_PID)
#define ENABLE_GYRO 					(_motor_flag|=FLAG_GYRO)
#define ENABLE_SENSOR 				(_motor_flag|=FLAG_SENSOR)
#define ENABLE_ENCODER 				(_motor_flag|=FLAG_ENCODER)
#define DISABLE_PID 					(_motor_flag&=~FLAG_PID)
#define DISABLE_GYRO 					(_motor_flag&=~FLAG_GYRO)
#define DISABLE_SENSOR 				(_motor_flag&=~FLAG_SENSOR)
#define DISABLE_ENCODER 			(_motor_flag&=~FLAG_ENCODER)

#define EncL									MOTOR_HTIM_ENC_L.Instance->CNT
#define EncR								MOTOR_HTIM_ENC_R.Instance->CNT

extern float sensorGyroW;

typedef struct
{
	int16_t lastEnc, encChange;
	int enc;
	int PWM;
} _MotorV;

typedef struct
{
	_MotorV mot[2];
	float targetV, currentV, PosErrV, lastPosErrV;
	float targetW, currentW, PosErrW, lastPosErrW;
	int distLeft;
} MotorsV;

void MotorInit();
void MotorUpdateEnc();
void MotorStop();
void MotorSetPWMRaw(int left, int right);
//========================
//====== POSITION ========
//========================
/*
#define KP  									2.0
#define KD  									1.0
#define KI  									0.1


typedef uint16_t 	pos_t;
typedef int16_t		posDif_t;

typedef struct
{
	int V, lastV, PWM;
	int T1, T2, T3;
	pos_t S1, S2, S3, wS; // whole track
	pos_t enc;
	posDif_t err;
	int errI, errP;
} _Motor_t;


typedef struct Motors_t Motors_t;
///
/// Its basic motor module struct.
/// Contains each variable needed to calculate new, better, motor PWM duty
/// @param driver is a poiter to function which can be additional motor driver
///			and this function are able to abort movement and current movement procedure.
///			This function are colled as a last motor driver, so it knows predicted motor power.
///
struct Motors_t
{
	int vel, t;
	MotorStat status;
	int(*driver)(Motors_t*);
	_Motor_t mot[2];
};



// ======================
// ==== BRIEF REVIEW ====
// ======================

// ==== most usefull functions in blocking mode ====
// MotorInit(), Go(), Turn()
//

// ==== most usefull functions in non blocking mode ====
// MotorInit(), GoA(), TurnA(), MotorUpdate()
//

// ==== and low-level control functions ====
// Motors_tetVel(), Motors_ttop(), MotorFloat()
//

// ==== encoder functions ====
// getEncL(), getEncR(), mmToTicks();
//

// ==== motor drivers ====
// MotorDriverP(), MotorDriverD();
//




// ===============================
// ==== FUNCTIONS DECLARATION ====
// ===============================

///
/// Convert distance in milimeters to encoder ticks
/// @return dinstance in encoder ticks
/// @param distance in mm
/// @see WHEEL_DIAMETER, TICKS_PER_REVOLUTION
/// @before none
/// @after none
/// 
int mmToTicks(int mm);

///
/// Init motor module. In every application this function has to be called before using another function from this module
/// @return void
/// @see Go(), Turn(), TICKS_PER_REVOLUTION, MOTOR_MAX_VEL, HALF_WHEELBASE, WHEEL_DIAMETER, TICKS_PER_REVOLUTION, KP, KD, KI
/// @before none
/// @after none
///
void MotorInit(void);

///
///	returns tics from right wheel encoder
/// @return tics from right wheel encoder
/// @see getEncL(), Motors_tetVel() 
/// @before MotorInit()
/// @after
///
int getEncL(void);


///
///	returns tics from left wheel encoder
/// @return tics from left wheel encoder
/// @see getEncL(), MotorSetPWM() 
/// @before MotorInit()
/// @after
///
int getEncR(void);


///
/// Set motor PWM from -MOTOR_MAX_VEL to +MOTOR_MAX_VEL
/// @see MotorFloat(), MotorStop()
/// @before MotorInit()
/// @after
///
void MotorSetPWMRaw(int left, int right);


///
/// Set motor PWM from -MOTOR_MAX_VEL to +MOTOR_MAX_VEL
/// @see MotorFloat(), MotorStop()
/// @before MotorInit()
/// @after
///
void MotorSetPWM(void);


///
/// Set motor velocity from -MOTOR_MAX_VEL to +MOTOR_MAX_VEL
/// 
/// @param pointer to Motors_t struct
/// @see MotorFloat(), MotorStop()
/// @before MotorInit()
/// @after
///
void MotorSetVel(Motors_t* m, uint8_t);


///
/// Block wheels and change state in Motors_t pointer
/// @return void
/// @see MotorFloat(), MotorSetVel()
/// @before MotorInit()
/// @after
///
void MotorStop(void);


///
/// Allows wheels to freely turning and change state int Motors_t pointer
/// @return void
/// @see MotorStop(), MotorSetVel()
/// @before MotorInit()
/// @after
///
void MotorFloat(void);


///
/// This function regulate motor speed.
/// When are you using asynchronous function, then you have to call this function manually and frequently
/// @return 0 when reach target, 1 otherwise
/// @see GoA, TurnA, Go, Turn, KP, KI, KD
/// @before MotorUpdate, GoA or TurnA
/// @after none
///
int MotorUpdate(void);


///
/// Go, this function doesn't block program when working.
/// MotorUpdate must be called frequetly with Motor pointer modified by this function
/// @return void
/// @param left a left wheel distance in mm
/// @param a right wheel distance in mm
/// @param velocity - average
/// @param optional poiter to additional driver (eg. based on sensors).
///        This function returns 0 to abort movement or 1 to continue.
///        This function is called as last motor driver with preset new value.
///        Pointer to current Motors_t struct is a parameter of this function
///        Default pointer is 0.
/// @see Go(), TurnA(), WHEEL_DIAMETER, TICKS_PER_REVOLUTION
/// @before MotorInit()
/// @after none
///
void GoA(int left, int right, int vel, int(*driver)(Motors_t*));


///
/// Go, this function  block program when working
/// @return void
/// @param left a left wheel distance in mm
/// @param a right wheel distance in mm
/// @param velocity - average
/// @param optional poiter to additional driver (eg. based on sensors).
///        This function returns 0 to abort movement or 1 to continue.
///        This function is called as last motor driver with preset new value.
///        Pointer to current Motors_t struct is a parameter of this function
///        Default pointer is 0.
/// @see GoA(), Turn(), WHEEL_DIAMETER, TICKS_PER_REVOLUTION
/// @before MotorInit()
/// @after none
///
void Go(int left, int right, int vel, int(*driver)(Motors_t*));



///
/// Turn, this function doesn't block program when working
/// MotorUpdate must be called frequetly with Motor pointer modified by this function
/// @return void
/// @param angle in degree. When angle < 0 then turn left otherwise right
/// @param radius of turn in mm. From center of robot to center of turn
/// @param velocity - average
/// @param optional poiter to additional driver (eg. based on sensors).
///        This function returns 0 to abort movement or 1 to continue.
///        This function is called as last motor driver with preset new value.
///        Pointer to current Motors_t struct is a parameter of this function
///        Default pointer is 0.
/// @see GoA(), Turn(), HALF_WHEELBASE, WHEEL_DIAMETER, TICKS_PER_REVOLUTION
/// @before MotorInit()
/// @after none
///
void TurnA(int angle, int radius, int vel, int(*driver)(Motors_t*));


///
/// Turn this function block program when working
/// @return void
/// @param angle in degree. When angle < 0 then turn left otherwise right
/// @param radius of turn in mm. From center of robot to center of turn
/// @param velocity - average
/// @param optional poiter to additional driver (eg. based on sensors).
///        This function returns 0 to abort movement or 1 to continue.
///        This function is called as last motor driver with preset new value.
///        Pointer to current Motors_t struct is a parameter of this function
///        Default pointer is 0.
/// @see Go(), TurnA(), HALF_WHEELBASE, WHEEL_DIAMETER, TICKS_PER_REVOLUTION
/// @before MotorInit()
/// @after none
///
void Turn(int angle, int radius, int vel, int(*driver)(Motors_t*));


///
/// truncate velocity parameter
/// @return truncated velocity
/// @param velocity to truncate
/// @see MOTOR_MAX_VEL
/// @before
/// @after
///
int MotorTruncVel(int vel);


///
/// Check reaching to the target
/// @return 1 when target is reached and 0 otherwise
/// @see MotorUpdate(), Go(), Turn()
/// @before MotorInit, GoA or TurnA
/// @after none
///
int MotorEnd(void);


///
/// Private function of Motor module
///
static void MotorDriverP(Motors_t* m);


///
/// Private function of Motor module
/// It works similar to MotorDriverP, but old way style
///
static void MotorDriverP2(Motors_t* m);


///
/// Private function of Motor module
///
static void MotorDriverD(Motors_t* m);


///
/// Private function of Motor module
///
static void MotorDriverVelP(Motors_t* m);

*/
#endif
