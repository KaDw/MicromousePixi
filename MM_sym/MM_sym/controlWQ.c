#include "stdafx.h"
#include "controlWQ.h"

Mode_t g_Mode{ MODE_TO_TARGET };
LCoord_t g_CurrX{ 0 }, g_CurrY{ 0 };
LCoord_t g_TargetX{ LAB_SIZE / 2 }, g_TargetY{ LAB_SIZE / 2 };
Dir_t g_CurrDir{ DIR_N };


extern Cell_t g_Map[LAB_SIZE][LAB_SIZE];
extern Cost_t g_Cost[LAB_SIZE][LAB_SIZE];
extern Dir_t g_Parent[LAB_SIZE][LAB_SIZE];

#define WQUEUE_BUFF_SIZE (LAB_SIZE*4 + 1)
Cost_t wqueue_buffC[WQUEUE_BUFF_SIZE];
LCoordFull_t wqueue_buffV[WQUEUE_BUFF_SIZE];

//#define LIST_BUFF_SIZE (LAB_SIZE*LAB_SIZE)
#define LIST_BUFF_SIZE (6)
Move_t list_buff[LIST_BUFF_SIZE];

wqueue_s	wq;
list_s		list;


void ConInit()
{
	g_CurrX = g_CurrY = 0;
	g_CurrDir = 0;
	g_TargetX = g_TargetY = LAB_SIZE / 2;
	g_Mode = 0;

	wqueue_init(&wq, wqueue_buffC, wqueue_buffV, WQUEUE_BUFF_SIZE);
	list_init(&list, list_buff, LIST_BUFF_SIZE);

	LabInit();
}

void ConNextStep()
{/*
	LabSetVisited(g_CurrX, g_CurrY);
	SensorUpdateSWD();
	ConUpdateLabFromCenter();


	// change mode if it is time to do that
	if (g_Mode == MODE_TO_TARGET && g_CurrX == g_TargetX && g_CurrY == g_TargetY)
		g_Mode = MODE_TO_START;
	else if (g_Mode == MODE_TO_START && g_CurrX == 0 && g_CurrY == 0)
		g_Mode = MODE_FAST_TARGET;
	else if (g_Mode == MODE_FAST_TARGET && g_CurrX == g_TargetX && g_CurrY == g_TargetY)
		g_Mode = MODE_FAST_START;
	else if (g_Mode == MODE_FAST_START && g_CurrX == 0 && g_CurrY == 0)
		g_Mode = MODE_STOP;

	// flood wright 
	if (g_Mode == MODE_TO_TARGET || g_Mode == MODE_FAST_TARGET)
	{
		ConFloodFrom(g_TargetX, g_TargetY);
		ConFindPathByCostFrom(g_TargetX, g_TargetY);
	}
	else if (g_Mode == MODE_TO_START || g_Mode == MODE_FAST_START)
	{
		ConFloodFrom(0, 0);
		ConFindPathByCostFrom(0, 0);
	}
	else if (g_Mode == MODE_STOP)
	{
		ConFloodFrom(0, 0);
		ConFindPathByCostFrom(0, 0);
		printf("Zatrzymaj sie!! file:%s  line:%d\n", __FILE__, __LINE__);
	}*/
}

void ConMakeMove()
{
	printf("To jeszcze nie dziala!!\n");
}


void ConUpdateLabFromCenter()
{
	//todo: update Lab from Sensors in every cell
	Dir_t dir;
	LCoord_t x, y;
	Wall_t wall;

	// dla scian z przodu
	dir = DIR_N + g_CurrDir;
	dir %= DIR_MODULO;
	wall = WALL_N << (g_CurrDir >> 1);
	if (wall > WALL_W)
		wall >>= 4;
	assert(wall <= WALL_W);

	if (g_SWD & SWD_FORWARD)
	{
		LabSetWall(g_CurrX, g_CurrY, wall);
	}
	else
	{
		if (g_SWD & SWD_FAR_FORWARD)
		{
			x = g_CurrX + LabGetDeltaX(dir);
			y = g_CurrY + LabGetDeltaY(dir);
			LabSetWall(x, y, wall);
		}

		// dla scian po skosie
		// kierunek jest ten sam co dla tych do przodu

		if (g_SWD & SWD_CROSS_LEFT)
		{
			wall = WALL_W << (g_CurrDir >> 1);
			if (wall > WALL_W)
				wall >>= 4;
			assert(wall <= WALL_W);
			x = g_CurrX + LabGetDeltaX(dir);
			y = g_CurrY + LabGetDeltaY(dir);
			LabSetWall(x, y, wall);
		}
		if (g_SWD & SWD_CROSS_RIGHT)
		{
			wall = WALL_W << (g_CurrDir >> 1);
			if (wall > WALL_W)
				wall >>= 4;
			assert(wall <= WALL_W);
			x = g_CurrX + LabGetDeltaX(dir);
			y = g_CurrY + LabGetDeltaY(dir);
			LabSetWall(x, y, wall);
		}
	}

	// dla scian z lewej
	dir = DIR_W + g_CurrDir;
	dir %= DIR_MODULO;
	wall = WALL_W << (g_CurrDir >> 1);
	if (wall > WALL_W)
		wall >>= 4;
	assert(wall <= WALL_W);

	if (g_SWD & SWD_LEFT)
	{
		LabSetWall(g_CurrX, g_CurrY, wall);
	}
	else if (g_SWD & SWD_FAR_LEFT)
	{
		x = g_CurrX + LabGetDeltaX(dir);
		y = g_CurrY + LabGetDeltaY(dir);
		LabSetWall(x, y, wall);
	}


	// dla scian z prawej
	dir = DIR_E + g_CurrDir;
	dir %= DIR_MODULO;
	wall = WALL_E << (g_CurrDir >> 1);
	if (wall > WALL_W)
		wall >>= 4;
	assert(wall <= WALL_W);

	if (g_SWD & SWD_RIGHT)
	{
		LabSetWall(g_CurrX, g_CurrY, wall);
	}
	else if (g_SWD & SWD_FAR_RIGHT)
	{
		x = g_CurrX + LabGetDeltaX(dir);
		y = g_CurrY + LabGetDeltaY(dir);
		LabSetWall(x, y, wall);
	}
}


void ConUpdateLabFromEdge()
{

}


void ConResetCosts()
{
	LCoordFull_t i;
	Cost_t* ptr = &g_Cost[0][0];
	Cost_t* end = ptr + LAB_SIZE*LAB_SIZE;

	while (ptr != end)
	{
		*ptr = MAX_COST;
		++ptr;
	}
}


void ConResetParents()
{
	LCoord_t x, y;

	for (x = 0; x < LAB_SIZE; ++x)
		for (y = 0; y < LAB_SIZE; ++y)
		{
			LabSetParent(x, y, WALL_PAR_N);
		}
}


void ConCheckOneCell(LCoord_t nx, LCoord_t ny, Wall_t parent, Cost_t nc)
{
	if (!LabIsIn(nx, ny))
		return;

	if (g_Cost[nx][ny] > nc)
	{
		g_Cost[nx][ny] = nc;
		//LabSetParent(nx, ny, parent);
		wqueue_push(&wq, LabCompress(nx, ny), nc);
	}
}


void ConCheckCellOrth(LCoord_t x, LCoord_t y, Cost_t c)
{
	// ortagonal
	if (!LabIsWall(x, y, WALL_N))
		ConCheckOneCell(x, y + 1, WALL_PAR_S, c + 2);
	if (!LabIsWall(x, y, WALL_E))
		ConCheckOneCell(x + 1, y, WALL_PAR_W, c + 2);
	if (!LabIsWall(x, y, WALL_S))
		ConCheckOneCell(x, y - 1, WALL_PAR_N, c + 2);
	if (!LabIsWall(x, y, WALL_W))
		ConCheckOneCell(x - 1, y, WALL_PAR_E, c + 2);
}


void ConCheckCelDiag(LCoord_t x, LCoord_t y, Cost_t c)
{
#define COST_DIAGONAL	3
#define COST_ORTHO		2
	// diagonal
	if (!LabIsWall(x, y, WALL_N))
	{
		ConCheckOneCell(x, y + 1, WALL_PAR_S, c + COST_ORTHO);
		if (!LabIsWall(x, y + 1, WALL_E))
			ConCheckOneCell(x + 1, y + 1, WALL_PAR_W, c + COST_DIAGONAL);
		if (!LabIsWall(x, y + 1, WALL_W))
			ConCheckOneCell(x - 1, y + 1, WALL_PAR_E, c + COST_DIAGONAL);
	}

	if (!LabIsWall(x, y, WALL_S))
	{
		ConCheckOneCell(x, y - 1, WALL_PAR_N, c + COST_ORTHO);
		if (!LabIsWall(x, y - 1, WALL_E))
			ConCheckOneCell(x + 1, y - 1, WALL_PAR_W, c + COST_DIAGONAL);
		if (!LabIsWall(x, y - 1, WALL_W))
			ConCheckOneCell(x - 1, y - 1, WALL_PAR_E, c + COST_DIAGONAL);
	}

	if (!LabIsWall(x, y, WALL_E))
	{
		ConCheckOneCell(x + 1, y, WALL_PAR_W, c + COST_ORTHO);
		if (!LabIsWall(x + 1, y, WALL_N))
			ConCheckOneCell(x + 1, y + 1, WALL_PAR_S, c + COST_DIAGONAL);
		if (!LabIsWall(x + 1, y, WALL_S))
			ConCheckOneCell(x + 1, y - 1, WALL_PAR_N, c + COST_DIAGONAL);
	}

	if (!LabIsWall(x, y, WALL_W))
	{
		ConCheckOneCell(x - 1, y, WALL_PAR_E, c + COST_ORTHO);
		if (!LabIsWall(x - 1, y, WALL_N))
			ConCheckOneCell(x - 1, y + 1, WALL_PAR_S, c + COST_DIAGONAL);
		if (!LabIsWall(x - 1, y, WALL_S))
			ConCheckOneCell(x - 1, y - 1, WALL_PAR_N, c + COST_DIAGONAL);
	}
}


//diagonal
// from current to target position
uint8_t ConFloodTo(int8_t dest_x, int8_t dest_y)
{
	assert_xy(dest_x, dest_y);

	LCoord_t x, y;
	Cost_t c; // cost
	wqueue_i i;

	ConResetCosts();
	g_Cost[g_CurrX][g_CurrY] = 0;
	wqueue_delete(&wq);

	wqueue_push(&wq, LabCompress(g_CurrX, g_CurrY), 0);
	g_Cost[g_CurrX][g_CurrY] = 0;

	do
	{
		i = wqueue_min(&wq); // get the cheapest one
		x = LabDecompressX(wq.bufValue[i]);
		y = LabDecompressY(wq.bufValue[i]);
		c = wq.bufCost[i];
		wqueue_remove(&wq, i);
		ConCheckCelDiag(x, y, c);

	} while (!(wqueue_empty(&wq) || (x == dest_x && y == dest_y)));

	// powiedz czy labirynt jest rozwiazywalny, czy nie
	if (x == dest_x && y == dest_y)
		return 1;
	else
		return 0;
}


//diagonal
// from target to current position
uint8_t ConFloodFrom(int8_t dest_x, int8_t dest_y)
{
	assert_xy(dest_x, dest_y);

	LCoord_t x, y;
	Cost_t c; // cost
	wqueue_i i;

	ConResetCosts();
	wqueue_delete(&wq);

	wqueue_push(&wq, LabCompress(dest_x, dest_y), 0);
	g_Cost[dest_x][dest_y] = 0;

	do
	{
		i = wqueue_min(&wq); // get the cheapest one
		x = LabDecompressX(wq.bufValue[i]);
		y = LabDecompressY(wq.bufValue[i]);
		c = wq.bufCost[i];
		wqueue_remove(&wq, i);
		ConCheckCelDiag(x, y, c);

	} while (!(wqueue_empty(&wq) || (x == g_CurrX && y == g_CurrY)));

	// powiedz czy labirynt jest rozwiazywalny, czy nie
	if (x == g_CurrX && y == g_CurrY)
		return 1;
	else
		return 0;
}

Wall_t ConFindPathByParent(int8_t dest_x, int8_t dest_y)
{
	int8_t dx, dy;
	LCoordFull_t counter{ 1 };
	Cell_t parent{ 0 };
	Move_t move;

	list_delete(&list);

	do
	{
		parent = LabGetParent(dest_x, dest_y);

		// dx and dy may by {-1, 0, +1}
		dx = (parent == WALL_PAR_E) - (parent == WALL_PAR_W);
		dy = (parent == WALL_PAR_N) - (parent == WALL_PAR_S);

		// dx dy dir => move

		dest_x += dx;
		dest_y += dy;

		if (dy != -1)
			move = 2 + dx; // bo w prawo to 2, w dol 2, a lewo 3 ale my jedziemy od konca! czyli odwracamy strony
		else
			move = 0; // 0 to do gory, ale odwracamy strony bo jedziemy od konca

		// sprawdz czy jeszcze jest miejsce w liscie
		// czy moze trzeba je zrobic usuwajac ruchy do wykonania najpozniej
		if (counter < LIST_BUFF_SIZE)
			++counter;
		else
			list_pop_front(&list);

		list_push_front(&list, move);
	} while (dest_x != g_CurrX || dest_y != g_CurrY);

	return parent;
}

Wall_t ConFindPathByCostTo(int8_t dest_x, int8_t dest_y)
{
	int8_t dx, dy;
	LCoordFull_t counter{ 1 };
	Move_t minMove = 0;
	Cost_t minCost = MAX_COST;

	list_delete(&list);

	while ((dest_x != g_CurrX || dest_y != g_CurrY))
	{
		// tutaj jest sprawdzanie 1 warunku oraz ostra nierownosc
		// aby robot domyslnie jechal w tym samym kierunku co przedtem
		// jezeli 2 pola maja ten sam koszt - to sie czesto zdarza przy
		// polach docelowych oraz startowych.
		// jezeli o to nie zadbamy, to algorytm sie zapetli i bedzie
		// probowac pojecha np. o jedna komorke w gore, a potem o 1 w dol
		if (minCost > LabGetCost(dest_x, dest_y + 1) && !LabIsWall(dest_x, dest_y, WALL_N))
		{
			minCost = LabGetCost(dest_x, dest_y + 1);
			dx = 0;
			dy = 1;
			minMove = MOVE_B; // ruch jest w przeciwna strone, bo jedziemy od tylu
		}
		if (minCost > LabGetCost(dest_x + 1, dest_y) && !LabIsWall(dest_x, dest_y, WALL_E))
		{
			dx = 1;
			dy = 0;
			minMove = MOVE_L;
			minCost = LabGetCost(dest_x + 1, dest_y);
		}
		if (minCost > LabGetCost(dest_x, dest_y - 1) && !LabIsWall(dest_x, dest_y, WALL_S))
		{
			dx = 0;
			dy = -1;
			minMove = MOVE_F;
			minCost = LabGetCost(dest_x, dest_y - 1);
		}
		if (minCost > LabGetCost(dest_x - 1, dest_y) && !LabIsWall(dest_x, dest_y, WALL_W))
		{
			dx = -1;
			dy = 0;
			minMove = MOVE_R;
			minCost = LabGetCost(dest_x - 1, dest_y);
		}

		dx = (minMove == MOVE_R) - (minMove == MOVE_L);
		dy = (minMove == MOVE_F) - (minMove == MOVE_B);

		dest_x -= dx;
		dest_y -= dy;

		assert_xy(dest_x, dest_y);

		// sprawdz czy jeszcze jest miejsce w liscie
		// czy moze trzeba je zrobic usuwajac ruchy do wykonania najpozniej
		if (counter < LIST_BUFF_SIZE)
			++counter;
		else
			list_pop_back(&list);

		// dodaj nowy ruch do listy ruchow
		list_push_front(&list, minMove);
	} 

	return minMove;
}

Wall_t ConFindPathByCostFrom(int8_t dest_x, int8_t dest_y)
{
	int8_t dx, dy;
	LCoordFull_t counter{ 1 }; // licznik znalezionych ruchow + 1
	LCoord_t x, y;
	Move_t minMove = 0;
	Cost_t minCost;

	list_delete(&list);
	x = g_CurrX;
	y = g_CurrY;

	do
	{
		if (!LabIsWall(x, y, WALL_N))
		{
			minCost = LabGetCost(x, y + 1);
			dx = 0;
			dy = 1;
			minMove = MOVE_F; // ruch jest w przeciwna strone, bo jedziemy od tylu
		}
		else
			minCost = MAX_COST;

		if (minCost > LabGetCost(x + 1, y) && !LabIsWall(x, y, WALL_E))
		{
			dx = 1;
			dy = 0;
			minMove = MOVE_R;
			minCost = LabGetCost(x + 1, y);
		}
		if (minCost > LabGetCost(x, y - 1) && !LabIsWall(x, y, WALL_S))
		{
			dx = 0;
			dy = -1;
			minMove = MOVE_B;
			minCost = LabGetCost(x, y - 1);
		}
		if (minCost > LabGetCost(x - 1, y) && !LabIsWall(x, y, WALL_W))
		{
			dx = -1;
			dy = 0;
			minMove = MOVE_L;
			minCost = LabGetCost(x - 1, y);
		}

		dx = ((minMove == MOVE_R) - (minMove == MOVE_L));
		dy = (minMove == MOVE_F) - (minMove == MOVE_B);

		x += dx;
		y += dy;

		list_push_back(&list, minMove);
		++counter;
	} while ((x != dest_x || y != dest_y) && counter < LIST_BUFF_SIZE);

	return minMove;
}

void ConConvertMove()
{
	Move_t lastMove = 0, rot;
	list_s* li = &list;// ConGetMoves();
	list_iterator i = list_begin(li);

	for (i = list_begin(li); i != list_end(li); i = list_next(li, i))
	{
		rot = rot = ConConvertMoveLoc(lastMove, *i);

		// pozamienianie zmiennych w pamieci
		lastMove = *i;
		*i = rot;
	}
}

Move_t ConConvertMoveLoc(Move_t lastMove, Move_t move)
{
	Move_t rot = move - lastMove;

	if (rot < 0)
		rot += 4;

	rot %= 4;
	return rot;
}


/// @brief return global direction of next move
Dir_t ConNextDir(Dir_t dir, int8_t m)
{
	return DIR_NONE;
}


void ConMove()
{
}

Dir_t ConGetMouseDir()
{
	return g_CurrDir;
}

LCoord_t ConGetX()
{
	return g_CurrX;
}

LCoord_t ConGetY()
{
	return g_CurrY;
}

list_s * ConGetMoves()
{
	return &list;
}
