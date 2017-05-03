#ifndef __MAP_H__
#define __MAP_H__

#include "stm32f4xx_hal.h"
#define MAZE_X 16
#define MAZE_Y 16

//uint8_t map[MAZE_X][MAZE_Y];

typedef enum
	{
	WALL_N = 1,
	WALL_E = 2,
	WALL_S = 4,
	WALL_W = 8,
	VISITED = 16
} wall_t;

extern uint8_t direction;

void updateMap();
void updateWalls();
void addWalls(uint8_t wall);
void run();
#endif