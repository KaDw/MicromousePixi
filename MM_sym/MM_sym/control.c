#include "stdafx.h"
#include "control.h"


// global map construction
// coordinates after translation
// or with g_mapMirror = 0
// (x,y)
//
// -------------------------
// |     |     |     |     |
// |-----------------------|
// | 0,2 | 1,2 | 2,2 |     |
// |-----------------------|  
// | 0,1 | 1,1 | 2,1 |     |
// |-----------------------|
// | 0,0 | 1,0 | 2,0 |     |
// -------------------------
//

/*
// ==== file global variable ====
Cell_t	g_Map[LAB_SIZE][LAB_SIZE];
int8_t  g_Cost[LAB_SIZE][LAB_SIZE];
Dir_t	g_CurrDir = DIR_N;
uint8_t	g_CurrX = 0;
uint8_t g_CurrY = 0;
Motors_t g_Motors;
uint8_t g_TargetX = LAB_SIZE - 1;
uint8_t g_TargetY = LAB_SIZE - 1;
//uint8_t g_TargetX = LAB_SIZE / 2;
//uint8_t g_TargetY = LAB_SIZE / 2;


//////////////////////////
//// Hi-level fun     ////
//////////////////////////
void ConInit()
{
}

void ConNextStep()
{
	Move_t next_move;

	ConFlood(g_TargetX, g_TargetY);
	next_move = ConNextMove();
}


/// @brief return global direction of next move
Dir_t ConNextDir(Dir_t dir, int8_t m)
{
	Dir_t mDir = DIR_N;
	int8_t mCost = ConGetCost(mDir, 1);

	for (Dir_t d = DIR_N+1; d <= DIR_NE; ++d)
	{
		if (mCost > ConGetCost(d, 1))
		{
			mDir = d;
			mCost = ConGetCost(d, 1);
		}
	}

	return mDir;
}


void ConMove()
{
	int i = 1;
	Dir_t dir = ConNextDir(g_CurrDir, 0); // get the best direction from current position

	// how many cell you have to go forward
	while (dir == ConNextDir(dir, i))
		++i;

	// transform direction to mouse rotation {45deg * (int)(dir)}
	dir -= g_CurrDir;
	if (dir > 4) dir -= 8;
	else if (dir < -4) dir += 8;

	CTurn((int)dir * 45, 0, 100, 0);
	CGo(i);
}


void ConFlood(int8_t dest_x, int8_t dest_y)
{
	const LOCAL_COST = 1;
	int8_t cost;
	int16_t queue_val;

	queue_delete();
	queue_push(ConCompressXY(dest_x, dest_y));

	while (!queue_empty())
	{
		// get info from queue and cell
		queue_val = queue_pop();
		dest_x = ConSplitX(queue_val);
		dest_y = ConSplitY(queue_val);
		cost = g_Cost[dest_x][dest_y];

		// it's not infiniti loop, because cost is always bigger
		// check 4 possibility
		if (ConIsMap(dest_x - 1, dest_y) && !ConIsWall(dest_x, dest_y, WALL_W) && g_Cost[dest_x - 1][dest_y] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x - 1, dest_y));
			g_Cost[dest_x - 1][dest_y] = cost + LOCAL_COST;
		}
		if (ConIsMap(dest_x + 1, dest_y) && !ConIsWall(dest_x, dest_y, WALL_E) && g_Cost[dest_x + 1][dest_y] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x + 1, dest_y));
			g_Cost[dest_x + 1][dest_y] = cost + LOCAL_COST;
		}
		if (ConIsMap(dest_x, dest_y + 1) && !ConIsWall(dest_x, dest_y, WALL_N) && g_Cost[dest_x][dest_y + 1] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x, dest_y + 1));
			g_Cost[dest_x][dest_y + 1] = cost + LOCAL_COST;
		}
		if (ConIsMap(dest_x, dest_y - 1) && !ConIsWall(dest_x, dest_y, WALL_S) && g_Cost[dest_x][dest_y - 1] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x, dest_y - 1));
			g_Cost[dest_x][dest_y - 1] = cost + LOCAL_COST;
		}
	}
}


void CanAStar()
{
	const LOCAL_COST = 2;
	uint8_t dest_x, dest_y;
	int8_t cost;
	int16_t queue_val;

	queue_delete();
	queue_push(ConCompressXY(dest_x, dest_y));

	while (!queue_empty())
	{
		// get info from queue and cell
		queue_val = queue_pop();
		dest_x = ConSplitX(queue_val);
		dest_y = ConSplitY(queue_val);
		cost = g_Cost[dest_x][dest_y];

		// it's not infiniti loop, because cost is always bigger
		// check 4 possibility
		if (ConIsMap(dest_x - 1, dest_y) && !ConIsWall(dest_x, dest_y, WALL_W) && g_Cost[dest_x - 1][dest_y] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x - 1, dest_y));
			g_Cost[dest_x - 1][dest_y] = cost + LOCAL_COST;
		}
		if (ConIsMap(dest_x + 1, dest_y) && !ConIsWall(dest_x, dest_y, WALL_E) && g_Cost[dest_x + 1][dest_y] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x + 1, dest_y));
			g_Cost[dest_x + 1][dest_y] = cost + LOCAL_COST;
		}
		if (ConIsMap(dest_x, dest_y + 1) && !ConIsWall(dest_x, dest_y, WALL_N) && g_Cost[dest_x][dest_y + 1] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x, dest_y + 1));
			g_Cost[dest_x][dest_y + 1] = cost + LOCAL_COST;
		}
		if (ConIsMap(dest_x, dest_y - 1) && !ConIsWall(dest_x, dest_y, WALL_S) && g_Cost[dest_x][dest_y - 1] > (cost + LOCAL_COST))
		{
			queue_push(ConCompressXY(dest_x, dest_y - 1));
			g_Cost[dest_x][dest_y - 1] = cost + LOCAL_COST;
		}
	}
}


Move_t ConNextMove()
{
	int8_t min_i = 0;
	Move_t m = MOVE_F;


}


Dir_t ConGetDir(int8_t rel_x, int8_t rel_y)
{
	// 0000 -> reserved
	// 0001 -> DIR_N
	// 0010 -> DIR_S
	// 0011 -> reserved
	// 0100 -> DIR_E
	// 0101 -> DIR_NE
	// 0110 -> DIR_SE
	// 0111 -> reserved
	// 1000 -> DIR_W
	// 1001 -> DIR_NW
	// 1010 -> DIR_SW
	// 1011 -> reserved
	// 1100 -> reserved
	// 1101 -> reserved
	// 1110 -> reserved
	// 1111 -> reserved
	static Dir_t D[11] = {0, DIR_N, DIR_S, 0, DIR_E, DIR_NE, DIR_SE, 0, DIR_W, DIR_NW, DIR_SW};
	uint8_t index = (rel_x < g_CurrX) << 3 | (g_CurrX < rel_x) << 2;
	index |= (rel_y < g_CurrY) << 1 | (g_CurrY < rel_y) << 0;
	return D[index];
}

uint8_t ConIsWallDir(Dir_t d)
{
	return ConIsWall(g_CurrX, g_CurrY, ConDir2Wall(d));
}

int ConGaugeCell(int x, int y) {
	assert(0 <= x && 0 <= y);
	assert(x <= LAB_SIZE && y <= LAB_SIZE);

	// basic cost (A*)
	Dir_t dir = ConGetDir(x, y);
	int value = x + y + abs(g_TargetX + g_TargetY - x - y);

	// bonus for similar direction
	value -= abs(4 - abs(g_CurrDir - dir));
	
	// add 
	//if (brak sciany)
		value += 100;
	
	// bonus for unvisited cell
	if(!ConIsVisited(x, y))
		value -= 10;

	return value;
}


int ConChooseDir() {
	// translate map to local
	// controlGauageDir for each
	// choose the best one
	return 0;
}



///
/// convert local wall orientation (from sensors) to global wall orienttion (on map)
/// @return wall in global orientation
/// @param current robot orientation
/// @param local wall orientation (from sensors)
/// @see ConWallGlo2Loc()
/// @before none
/// @after none
///
Wall_t ConWallLoc2Glo(Dir_t currDir, Wall_t localWall) {
	assert(g_CurrDir%2==0); //ortagonal mode

	Wall_t wall = 0x0f & localWall;
	localWall = localWall&(~0x0f) | 0x0f & (((wall << currDir) | (wall >> (4 - currDir))));
	return localWall;
}

///
/// convert global wall orienttion (on map) to local wall orientation (from sensors)
/// @return wall in local orientation
/// @param current robot orientation
/// @param global wall orientation (from map)
/// @see ConWallLoc2Wwall()
/// @before none
/// @after none
///
Wall_t ConWallGlo2Loc(Dir_t currDir, Wall_t localWall) {
	return ConWallLoc2Glo(4 - currDir, localWall);
}



Wall_t ConDir2Wall(Dir_t d)
{
	d = (d + g_CurrDir) % DIR_MODULO;
	Wall_t w = (Wall_t)(1 << (d >> 1));
	
	if (d % 2 == 0) // ortagonal
		return w;
	else if (d % 3 == 1)
		return w | WALL_N;
	else
		return w | WALL_S;
}




//////////////////////////
//// Cell menagement  ////
//////////////////////////


void ConSetWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	w &= 0x0f; // get only wall info

	// set wall in pointed cell
	g_Map[x][y] |= w;

	// and in neighbourly cell
	if ((w & WALL_N) && y+1<LAB_SIZE)
		g_Map[x][y + 1] |= WALL_S;
	if ((w & WALL_E) && x+1<LAB_SIZE)
		g_Map[x + 1][y] |= WALL_W;
	if ((w & WALL_S) && y - 1 >= 0)
		g_Map[x][y - 1] |= WALL_N;
	if ((w & WALL_W) && x - 1 >= 0)
		g_Map[x - 1][y] |= WALL_E;
}

void ConClearWall(int8_t x, int8_t y, Wall_t w)
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

Cell_t* ConGetCell(int8_t x, int8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	return &g_Map[x][y];
}

uint8_t ConIsWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	return g_Map[x][y] & w;
}

uint8_t ConIsMap(int8_t x, int8_t y)
{
	return (x >= 0 && y >= 0 && x<LAB_SIZE && y<LAB_SIZE);
}

uint8_t	ConIsVisited(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	return g_Map[x][y] & WALL_VISITED;
}

uint8_t ConIsBlind(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	return g_Map[x][y] & WALL_BLIND;
}

void ConSetVisited(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	g_Map[x][y] |= WALL_VISITED;
}

void ConClearVisited(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	g_Map[x][y] &= ~WALL_VISITED;
}

void ConSetBlind(uint8_t x, uint8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	g_Map[x][y] |= WALL_BLIND;
}



//////////////////////////
//// Moving functions ////
//////////////////////////

void CGoFwd(uint8_t c)
{
	//g_CurrX += c;
	CGo(c*CELL_SIZE, c*CELL_SIZE, c > 1 ? 200 : 100, 0);
}


void CGoFwdCross(uint8_t c)
{
	int dist = (int)(CELL_SIZE*c*SQRT2);
	CGo(dist, dist, c > 1 ? 200 : 100, 0);
}


void CTurn(int angle, int radius, int vel, int(*driver)(Motors_t*))
{
	TurnA(&g_Motors, angle, radius, vel, driver);

	g_CurrDir += angle % 45;
}


void CGo(int i)
{
	if (ConIsOrt())
		GoA(&g_Motors, i*CELL_SIZE, i*CELL_SIZE, 100, 0);
	else
		GoA(&g_Motors, i*CELL_SIZE*SQRT2, i*CELL_SIZE*SQRT2, 100, 0);

	g_CurrX += i * ConDirToDX(g_CurrDir);
	g_CurrY += i * ConDirToDY(g_CurrDir);
}






//////////////////////////
//// Simulator fun    ////
//////////////////////////
#ifdef WIN32

/// @brieg return 1 if in movement in this dirrection x will be increased or -1 when decreased or 0 when constant
int8_t ConDirToDX(Dir_t d)
{
	d %= 8;
	return ((d < 4) - (d>4) - (d == 0));
}

/// @brieg return 1 if in movement in this dirrection y will be increased or -1 when decreased or 0 when constant
int8_t ConDirToDY(Dir_t d)
{
	d %= 8;
	return ((d <= 2 && d <= 6) - (6 >= d || d >= 2));
}

void TurnA(Motors_t* m, int angle, int radius, int vel, int(*driver)(Motors_t*))
{
	m->status = MOTOR_STOPPED;

	angle %= 45;
	g_CurrDir = (g_CurrDir + (angle % 45) );

	if (g_CurrDir < 0)
		g_CurrDir += 8;

	g_CurrDir %= 8;
}

void GoA(Motors_t* m, int left, int right, int vel, int(*driver)(Motors_t*))
{
	m->status = MOTOR_STOPPED;
}

Dir_t ConGetMouseDir()
{
	return g_CurrDir;
}

int8_t ConGetX()
{
	return g_CurrX;
}

int8_t ConGetY()
{
	return g_CurrY;
}


#endif*/