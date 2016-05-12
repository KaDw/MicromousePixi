#define __CONTROL_H__
#include <assert.h>
//#include "queue.h"

#ifndef WIN32
#define WIN32
#endif

#define LAB_SIZE 10
#define CELL_SIZE 100 /*in mm*/
/*
#define SQRT2	1.4142135623730951


#ifdef WIN32
#include <math.h>
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


#ifndef __CONTROL_H
#define __CONTROL_H


#ifdef __cplusplus
extern "C"
{
#endif

	

//////////////////////////
//// HEADER START     ////
//////////////////////////


typedef enum {
	DIR_NONE = 0xff,
	DIR_N = 0,
	DIR_NE= 1, // CROSS
	DIR_E = 2,
	DIR_SE= 3, // CROSS
	DIR_S = 4,
	DIR_SW= 5, // CROSS
	DIR_W = 6,
	DIR_NW= 7, // CROSS
	DIR_MODULO = 8
} Dir_t;

#define ConIsOrt() !(g_CurrDir%2)

// Wall_t definition
// | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
// 7 - set if i were there
// 6 - set if it is blind alley
// 5:4 - move enum
// 3:0 - walls
typedef enum {
	WALL_N = 1,
	WALL_E = 2,
	WALL_S = 4,
	WALL_W = 8,
	WALL_BLIND = 0x40,
	WALL_VISITED = 0x80
} Wall_t;

typedef uint8_t Cell_t;


typedef enum {
	MOVE_F,
	MOVE_FC,
	MOVE_R,
	MOVE_L,
	MOVE_LCL, // left cross and justify to left
	MOVE_LCR, // left cross and justify to right
	MOVE_RCR,
	MOVE_RCL,
} Move_t;


//////////////////////////
//// Hi-level fun     ////
//////////////////////////

void ConInit();
void ConNextStep();
void ConFlood(int8_t dest_x, int8_t dest_y);
Move_t ConNextMove();
void ConFloodCheck(int8_t curr_cost, int8_t cx, int8_t cy);


/// @brief convert 
Wall_t		ConDir2Wall(Dir_t d);

//////////////////////////
//// Cell menagement  ////
//////////////////////////

Cell_t*		ConGetCell(int8_t x, int8_t y);
uint8_t		ConIsVisited(uint8_t x, uint8_t y);
uint8_t		ConIsBlind(uint8_t x, uint8_t y);
uint8_t		ConIsWall(int8_t x, int8_t y, Wall_t w);
uint8_t		ConIsMap(int8_t x, int8_t y);

void		ConSetBlind(uint8_t x, uint8_t y);
void		ConSetVisited(uint8_t x, uint8_t y);
void		ConClearVisited(uint8_t x, uint8_t y);
void		ConSetWall(int8_t x, int8_t y, Wall_t w);
void		ConClearWall(int8_t x, int8_t y, Wall_t w);

#define		ConCompressXY(x,y)	(((x)<<8) | (y))
#define		ConSplitX(w)		((w)>>8)
#define		ConSplitY(w)		((w) & 0xff)

//////////////////////////
//// Moving functions ////
//////////////////////////

/// @brief go forward c cells in ortagonal mode
void		CGoFwd(uint8_t c);

/// @brief go forward c cells in cross mode
void		CGoFwdCross(uint8_t c);

void CGo(int i);

#ifdef WIN32
Dir_t ConGetMouseDir();
int8_t ConGetX();
int8_t ConGetY();
#endif


#ifdef __cplusplus
}
#endif
#endif*/