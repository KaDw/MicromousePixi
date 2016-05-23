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
#define MOTOR_MAX_PWM					1000
#define WHEEL_DIAMETER 				37.0f /* mm*/
#define MOTOR_SLOW_TICK				200
#define MOTOR_SLOW_VEL				200
#define TICKS_PER_REVOLUTION	1760.0f

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
#define MOTOR_VELV_KP					35.1f //20
#define MOTOR_VELV_KI					6.60f // 11
#define MOTOR_VELV_KD					0.0f // -0.01676
#define MOTOR_VELW_KP					3.0f
#define MOTOR_VELW_KI					10.0f
#define MOTOR_VELW_KD					0.0f
//#define MOTOR_VELW_KP					8.0f
//#define MOTOR_VELW_KI					1.0f
//#define MOTOR_VELW_KD					0.0f

// ACC_V [mm/s/s] 	ACC_W[rad/s/s]
#define MOTOR_ACC_V						(1300.0f*2.0f) // double becouse for 2 wheels
#define MOTOR_ACC_W						5.0f

// flags determine sensor int turn
extern int _motor_flag;
#define FLAG_PID							1
#define FLAG_GYRO							2
#define FLAG_SENSOR						4
#define FLAG_ENCODER					8
#define ENABLE_PID()					(_motor_flag|=FLAG_PID)
#define ENABLE_GYRO() 				(_motor_flag|=FLAG_GYRO)
#define ENABLE_SENSOR() 			(_motor_flag|=FLAG_SENSOR)
#define ENABLE_ENCODER() 			(_motor_flag|=FLAG_ENCODER)
#define DISABLE_PID 					(_motor_flag&=~FLAG_PID)
#define DISABLE_GYRO 					(_motor_flag&=~FLAG_GYRO)
#define DISABLE_SENSOR 				(_motor_flag&=~FLAG_SENSOR)
#define DISABLE_ENCODER 			(_motor_flag&=~FLAG_ENCODER)

#define EncL									MOTOR_HTIM_ENC_L.Instance->CNT
#define EncR									MOTOR_HTIM_ENC_R.Instance->CNT

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
	float targetV, currentV, previousV; // mm/s
	float targetW, currentW, previousW; // rad/s
	//float PosErrV, lastPosErrV;
	//float	PosErrW, lastPosErrW;
	float errVP, errVI, errVD;
	float errWP, errWI, errWD;
	//int distLeftV, SbreakV; // tick
	//float distLeftW, SbreakW;
	int tv, tw; // [T]
	MotorStat status;
} MotorsV;

void MotorInit(void);
void MotorUpdate(void);
void MotorStop(void);
void MotorFloat(void);
void MotorSetPWM();
void MotorSetPWMRaw(int left, int right);
void MotorGo(int left, int right, float vel); // [mm] [mm] [mm/s]
void MotorGoA(int left, int right, float vel); // [mm] [mm] [mm/s]
void MotorTurn(int angle, int r, float vel);
void MotorTurnA(int angle, int r, float vel);

//========================
//====== POSITION ========
//========================
/*
//      _____________
//     /(1)       (2)\
// ___/               \
//   t=0               \____
//        Velocity     (3)
//                      ___
//                   .~^
//                .'
//             .'
//          .'
//       .'
// ___,-"   Position
//
//
#define MOTOR_ACC_V						600 // mm/s/s

#define MOTOR_VELV_KP  				(12.0f*0.034f)
#define MOTOR_VELV_KD  				0.0f
#define MOTOR_VELV_KI  				(0.001f*1.85f)
#define MOTOR_VELW_KP  				(20.0f*0.034f)
#define MOTOR_VELW_KD  				0.0f
#define MOTOR_VELW_KI  				(1.0e-9f)


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

typedef uint16_t 	pos_t;
typedef int16_t		posDif_t;

typedef struct
{
	int V, lastV; // [mm/s]
	int T1, T2, T3; // T - here [ms]
	int S0, S1, S2, S3, wS; // whole track [tick]
	int enc;
	int16_t lastEnc, encChange;
	float errP, errI, errD;
	int PWM, PWMV;
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
/// @see Go(), Turn(), TICKS_PER_REVOLUTION, MOTOR_MAX_PWM, HALF_WHEELBASE, WHEEL_DIAMETER, TICKS_PER_REVOLUTION, KP, KD, KI
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
/// Set motor PWM from -MOTOR_MAX_PWM to +MOTOR_MAX_PWM
/// @see MotorFloat(), MotorStop()
/// @before MotorInit()
/// @after
///
void MotorSetPWMRaw(int left, int right);


///
/// Set motor PWM from -MOTOR_MAX_PWM to +MOTOR_MAX_PWM
/// @see MotorFloat(), MotorStop()
/// @before MotorInit()
/// @after
///
void MotorSetPWM(void);


///
/// Set motor velocity from -MOTOR_MAX_PWM to +MOTOR_MAX_PWM
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
void MotorGoA(int left, int right, float vel, int(*driver)(Motors_t*));


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
void Go(int left, int right, float vel, int(*driver)(Motors_t*));



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
/// @see MOTOR_MAX_PWM
/// @before
/// @after
///
int MotorTruncPWM(int vel);


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
static void MotorDriverVelP(Motors_t* m);
*/

#endif
