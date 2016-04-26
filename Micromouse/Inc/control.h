
#include "motor.h"


const int LAB_SIZE 10;


typedef enum{
	DIR_NONE=0xff;
	DIR_N = 0,
	DIR_E = 1,
	DIR_S = 2,
	DIR_W = 3,
	DIR_NE= 4,
	DIR_NW= 5,
	DIR_SE= 6,
	DIR_SW= 7,
} Dir_t;

typedef enum{
	WALL_N = 1,
	WALL_E = 2,
	WALL_S = 4,
	WALL_W = 8,
} Wall_t;

typedef uint8_t Cell_t;

// transform position, depending on 
#define TR_Y(v) (v)
#define TR_X (v) 
#define TR_A(v) 


Wall_t ConWallLoc2Glo(Dir_t currDir, Wall_t localWall); // ControlTransformWall from local orientation to global orientation
Wall_t ConWallGlo2Loc(Dir_t currDir, Wall_t localWall);
Cell_t* (*ConGetCell)(uint8_t x, uint8_t y);
DirLoc2Glob(DIR_t)