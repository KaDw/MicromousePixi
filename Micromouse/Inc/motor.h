// @author:		Karol Trzciński
// @date:		march 2016
// @project:	MicroMouse
// @brief:		Control Motor Module, with complex driver and encoder routine
//				In this file there is no decision function

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f4xx_hal.h"


const float PI									= 3.14159265359;
const int MOTOR_MAX_VEL					= 1000;
const int MOTOR_DRIVER_D_CNT 		= 100;
const int MOTOR_EPSILON 				= 10; // enc tick
const int MOTOR_SLOW_TICK		= 150;
const int MOTOR_SLOW_VEL		= 150;
const int TICKS_PER_REVOLUTION	= 420;
const int HALF_WHEELBASE				= 100/2; // mm
const int WHEEL_DIAMETER 				= 45; // mm

const float KP = 1.0;
const float KD = 0.0;
const float KI = 0.0;


#define MOTOR_GPIO GPIOC
/*#define ML_IN1_Pin 
#define ML_IN2_Pin
#define MR_IN1_Pin 
#define MR_IN2_Pin*/

typedef enum
{
	MOTOR_STOPPED,
	MOTOR_RUNNING_STOP, // motor is running now, stop wheels when end running
	MOTOR_RUNNING_FLOAT, // motor is running now, float wheels when end running
	MOTOR_FLOATING,
	MOTOR_CONST_VEL
} MotorStat;



///
/// Its basic motor module struct.
/// Contains each variable needed to calculate new, better, motor PWM duty
/// @param driver is a poiter to function which can be additional motor driver
///			and this function are able to abort movement and current movement procedure.
///			This function are colled as a last motor driver, so it knows predicted motor power.
///
typedef struct
{
	int vel;
	uint16_t ePosL, ePosR;
	int velL, velR;
	MotorStat status;
	int(*driver)(Motors_t*);
} Motors_t;



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
void MotorInit();

///
///	returns tics from right wheel encoder
/// @return tics from right wheel encoder
/// @see getEncL(), Motors_tetVel() 
/// @before MotorInit()
/// @after
///
int getEncL();


///
///	returns tics from left wheel encoder
/// @return tics from left wheel encoder
/// @see getEncL(), MotorSetPWM() 
/// @before MotorInit()
/// @after
///
int getEncR();


///
/// Set motor PWM from -MOTOR_MAX_VEL to +MOTOR_MAX_VEL
/// @param pointer to Motors_t struct
/// @see MotorFloat(), MotorStop()
/// @before MotorInit()
/// @after
///
void MotorSetPWM(Motors_t* m);


///
/// Set motor velocity from -MOTOR_MAX_VEL to +MOTOR_MAX_VEL
/// 
/// @param pointer to Motors_t struct
/// @see MotorFloat(), MotorStop()
/// @before MotorInit()
/// @after
///
void MotorSetVel(Motors_t* m, uint8_t );


///
/// Block wheels and change state in Motors_t pointer
/// @return void
/// @param pointer to Motors_t struct or 0
/// @see MotorFloat(), MotorSetVel()
/// @before MotorInit()
/// @after
///
void MotorStop(Motors_t* m);


///
/// Allows wheels to freely turning and change state int Motors_t pointer
/// @return void
/// @param pointer to Motors_t struct or 0
/// @see MotorStop(), MotorSetVel()
/// @before MotorInit()
/// @after
///
void MotorFloat(Motors_t* m);


///
/// This function regulate motor speed.
/// When are you using asynchronous function, then you have to call this function manually and frequently
/// @return 0 when reach target, 1 otherwise
/// @param pointer to Motors_t struct with destination coordinates
/// @see GoA, TurnA, Go, Turn, KP, KI, KD
/// @before MotorUpdate, GoA or TurnA
/// @after none
///
int MotorUpdate(Motors_t* m);


///
/// Go, this function doesn't block program when working.
/// MotorUpdate must be called frequetly with Motor pointer modified by this function
/// @return void
/// @param pointer to Motors_t struct which will be modified and manually yoused in MotorUpdate()
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
void GoA(Motors_t* m, int left, int right, int vel, int(*driver)(Motors_t*));


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
/// @param pointer to Motors_t struct which will be modified and manually yoused in MotorUpdate()
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
void TurnA(Motors_t* m, int angle, int radius, int vel, int(*driver)(Motors_t*));


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
/// @param pointer to Motors_t struct with target coordinates
/// @see MotorUpdate(), Go(), Turn()
/// @before MotorInit, GoA or TurnA
/// @after none
///
int MotorEnd(Motors_t* m);


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


#endif