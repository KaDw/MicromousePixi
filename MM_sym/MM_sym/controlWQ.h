#ifndef __CONTROLWQ_H__
#define __CONTROLWQ_H__
#include "wqueue.h"
#include "list.h"
#include "lab.h"


// global map construction
// coordinates after translation
// or with g_mapMirror = 0
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

#ifdef __cplusplus
extern "C"
{
#endif

#define MODE_TO_TARGET	0
#define MODE_TO_START	1
#define MODE_FAST_TARGET 2
#define MODE_FAST_START	3
#define MODE_STOP		4


	typedef uint8_t Mode_t;
	//extern Mode_t g_Mode;

	/// external function from Sensor module - not implemented here
	void SensorUpdateSWD();

	/// init variables
	void ConInit();

	/// call Flood, FindPath and control g_Mode
	void ConNextStep();

	/// take data from path list and call motor control functions
	void ConMakeMove();

	/// update lab map
	/// check WALL_FAR_FORWARD, WALL_FAR_RIGHT, WALL_FAR_LEFT
	///		WALL_LEFT, WALL_RIGHT, WALL_FORWARD
	/// This function based on current data from sensors
	/// should be called in a ?middle? of a cell
	void ConUpdateLabFromCenter();

	/// update lab map
	/// This function based on current data from sensors
	/// should be called in a ?edge? of a cell
	void ConUpdateLabFromEdge();


	/// it is used only in a flood algorithm
	void ConCheckOneCell(LCoord_t nx, LCoord_t ny, Wall_t parent, Cost_t nc);

	/// it is used only in a flood algorithm
	void ConCheckCellOrth(LCoord_t x, LCoord_t y, Cost_t c);

	/// it is used only in a flood algorithm
	void ConCheckCelDiag(LCoord_t x, LCoord_t y, Cost_t c);

	/// assign parent and cost to cells
	/// pointer moves from g_Curr position to dest pos
	/// @return 1 if it is possible to achieve target x, y and 0 otherwise
	uint8_t ConFloodTo(int8_t dest_x, int8_t dest_y);

	/// assign parent and cost to cells
	/// pointer moves to dest pos from g_Curr position
	/// @return 1 if it is possible to achieve target x, y and 0 otherwise
	uint8_t ConFloodFrom(int8_t dest_x, int8_t dest_y);

	/// create path in list
	/// this function base on data from g_Map (parents)
	Wall_t ConFindPathByParent(int8_t dest_x, int8_t dest_y);

	/// create path in list
	/// this function base on data from g_Cost
	Wall_t ConFindPathByCostTo(int8_t dest_x, int8_t dest_y);

	/// create path in list
	/// this function base on data from g_Cost
	Wall_t ConFindPathByCostFrom(int8_t dest_x, int8_t dest_y);



	/// convert 2 global moves to robot order
	Move_t ConConvertMoveLoc(Move_t lastMove, Move_t move); 

	/// convert path in list to robot orders (by ConConvertMoveLoc)
	void ConConvertMove();


	Dir_t ConGetMouseDir();
	LCoord_t ConGetX();
	LCoord_t ConGetY();
	list_s* ConGetMoves();


#ifdef __cplusplus
}
#endif

#endif

