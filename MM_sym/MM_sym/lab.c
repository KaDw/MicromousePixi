#include "stdafx.h"
#include "lab.h"

Cell_t g_Map[LAB_SIZE][LAB_SIZE];
Cost_t g_Cost[LAB_SIZE][LAB_SIZE];
Sensor_t g_SWD = 0;



void LabInit()
{
	for (LCoord_t x = 0; x < LAB_SIZE; ++x)
	{
		LabSetWall(x, 0, WALL_S);
		LabSetWall(x, LAB_SIZE - 1, WALL_N);
	}

	for (uint8_t y = 0; y < LAB_SIZE; ++y)
	{
		LabSetWall(0, y, WALL_W);
		LabSetWall(LAB_SIZE - 1, y, WALL_E);
	}
}


LCoordFull_t LabCompress(LCoord_t x, LCoord_t y)
{
	assert(LabIsIn(x, y));

	return (x << 4) | y;
}


LCoord_t LabDecompressX(LCoordFull_t c)
{
	return c >> 4;
}


LCoord_t LabDecompressY(LCoordFull_t c)
{
	return c & 0x0f;
}


Cost_t LabCompressCost(Cost_t real, Cost_t imag)
{
	return ((real+imag) << 5) | (imag & 0x1F); // 11 bit for real (2048) and  5 bit for imag(32)
}


Cost_t LabDecompressCostReal(Cost_t c)
{
	return (c >> 5) - (c & 0x1F);
}


Cost_t LabDecomplressImag(Cost_t c)
{
	return c & 0x1F;
}


void LabSetWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	w &= 0x0f; // get only wall info

			   // set wall in pointed cell
	g_Map[x][y] |= w;

	// and in neighbourly cell
	if ((w & WALL_N) && y + 1<LAB_SIZE)
		g_Map[x][y + 1] |= WALL_S;
	if ((w & WALL_E) && x + 1<LAB_SIZE)
		g_Map[x + 1][y] |= WALL_W;
	if ((w & WALL_S) && y - 1 >= 0)
		g_Map[x][y - 1] |= WALL_N;
	if ((w & WALL_W) && x - 1 >= 0)
		g_Map[x - 1][y] |= WALL_E;
}


void LabClearWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	w &= 0x0f; // get only wall info

			   // clear wall in pointed cell
	g_Map[x][y] &= ~w;

	// and in neighbourly cell
	if ((w & WALL_N) && y + 1<LAB_SIZE)
		g_Map[x][y + 1] &= ~WALL_S;
	if ((w & WALL_E) && x + 1<LAB_SIZE)
		g_Map[x + 1][y] &= ~WALL_W;
	if ((w & WALL_S) && y - 1 >= 0)
		g_Map[x][y - 1] &= ~WALL_N;
	if ((w & WALL_W) && x - 1 >= 0)
		g_Map[x - 1][y] &= ~WALL_E;
}


void LabSetParent(LCoord_t x, LCoord_t y, Wall_t p)
{
	assert(LabIsIn(x, y));

	g_Map[x][y] &= ~WALL_PAR_MASK;
	g_Map[x][y] |= p;
}


Cell_t* LabGetCell(int8_t x, int8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	return &g_Map[x][y];
}


uint8_t LabIsWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	return g_Map[x][y] & w;
}


uint8_t LabIsIn(int8_t x, int8_t y)
{
	return (x >= 0 && y >= 0 && x<LAB_SIZE && y<LAB_SIZE);
}


Wall_t LabGetParent(LCoord_t x, LCoord_t y)
{
	assert(LabIsIn(x, y));

	return g_Map[x][y]&WALL_PAR_MASK;
}


Cost_t LabGetCostReal(LCoord_t x, LCoord_t y)
{
	if (!LabIsIn(x, y))
		return MAX_COST;

	return (g_Cost[x][y] >> 5) - (g_Cost[x][y] & 0x1F);
}


Cost_t LabGetCostImag(LCoord_t x, LCoord_t y)
{
	if (!LabIsIn(x, y))
		return MAX_COST;

	return g_Cost[x][y] & 0x1F;
}


uint8_t	LabIsVisited(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	return g_Map[x][y] & WALL_VISITED;
}


uint8_t LabIsBlind(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	return g_Map[x][y] & WALL_BLIND;
}


void LabSetVisited(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	g_Map[x][y] |= WALL_VISITED;
}


void LabClearVisited(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	g_Map[x][y] &= ~WALL_VISITED;
}


void LabSetBlind(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	g_Map[x][y] |= WALL_BLIND;
}

const int8_t LabDeltaX[8] = {
	0,		// DIR_N
	1,		// DIR_NE
	1,		// DIR_E
	1,		// DIR_SE
	0,		// DIR_S
	-1,		// DIR_SW
	-1,		// DIR_W
	-1		// DIR_NW
};

const int8_t LabDeltaY[8] = {
	1,		// DIR_N
	1,		// DIR_NE
	0,		// DIR_E
	-1,		// DIR_SE
	-1,		// DIR_S
	-1,		// DIR_SW
	0,		// DIR_W
	1		// DIR_NW
};

/*int8_t LabGetDeltaX(Dir_t d)
{
return (1 <= d && d <= 3) - (5 <= d && d <= 7);
}

int8_t LabGetDeltaY(Dir_t d)
{
return (d == 7 || d <= 1) - (3 <= d && d <= 5);
}*/
