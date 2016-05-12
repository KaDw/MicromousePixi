#ifndef __LAB_H__
#define __LAB_H__
#define WIN32
#include "keil_to_vs.h"

typedef uint16_t Cost_t; // lab cost
typedef uint8_t LCoord_t; // lab coordinate
typedef uint8_t LCoordFull_t; // lab coordinate

// MAX_Cost can not be 0xff because we sometimes make something like this f(cost + 2)
#define MAX_COST		2030
#define LAB_SIZE		10 /* in cells */
#define CELL_SIZE		100 /*in mm*/

#ifdef NDEBUG
#define assert_xy(x,y) ((void)0)
#else
#define assert_xy(x,y)	assert(x>=0&&y>=0); \
						assert(x<LAB_SIZE&&y<LAB_SIZE)
#endif

extern const int8_t LabDeltaX[8];
extern const int8_t LabDeltaY[8];

#ifdef __cplusplus
extern "C"
{
#endif

	#define DIR_NONE	0xff
	#define DIR_N		0
	#define DIR_NE		1
	#define DIR_E		2
	#define DIR_SE		3
	#define DIR_S		4
	#define DIR_SW		5
	#define DIR_W		6
	#define DIR_NW		7
	#define DIR_MODULO	8
	typedef uint8_t Dir_t;

#define ConIsOrt() !(g_CurrDir%2)

	// Wall_t definition
	// | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	// 7 - set if i were there
	// 6 - set if it is blind alley
	// 5:4 - move enum
	// 3:0 - walls
	/*typedef enum {
		WALL_N = 1,
		WALL_E = 2,
		WALL_S = 4,
		WALL_W = 8,
		WALL_BLIND = 0x40,
		WALL_VISITED = 0x80
	} Wall_t;*/
	#define WALL_N 1
	#define WALL_E 2
	#define WALL_S 4
	#define WALL_W 8
	#define WALL_MASK 0x0f
	#define WALL_PAR_N 0x00
	#define WALL_PAR_E 0x10
	#define WALL_PAR_S 0x20
	#define WALL_PAR_W 0x30
	#define WALL_PAR_MASK 0x30
	#define	WALL_BLIND 0x40
	#define WALL_VISITED 0x80
	typedef uint8_t Wall_t;

	typedef uint8_t Cell_t;


	//
	//
	//             -FF-
	//        |CL|  -F-  |CR|
	//  |FL|   |L|  /A\  |R|   |FR|
	//
	// sensor wall data
	#define SWD_FAR_LEFT	0x80
	#define SWD_LEFT		0x40
	#define SWD_CROSS_LEFT	0x20
	#define SWD_FAR_FORWARD	0x10
	#define SWD_FORWARD		0x08
	#define SWD_CROSS_RIGHT	0x04
	#define SWD_RIGHT		0x02
	#define SWD_FAR_RIGHT	0x01
	typedef uint8_t Sensor_t;

	/*typedef enum {
		MOVE_F = 0,
		MOVE_R = 1,
		MOVE_L = 2,
		MOVE_B = 3,
		MOVE_FC,
		MOVE_LCL, // left cross and justify to left
		MOVE_LCR, // left cross and justify to right
		MOVE_RCR,
		MOVE_RCL,
	} Move_t;*/

	#define MOVE_F 0
	#define MOVE_R 1
	#define MOVE_B 2
	#define MOVE_L 3
	typedef int8_t Move_t;

	void		LabInit();

	LCoordFull_t LabCompress(LCoord_t x, LCoord_t y);
	LCoord_t	LabDecompressX(LCoordFull_t c);
	LCoord_t	LabDecompressY(LCoordFull_t c);

	Cost_t		LabCompressCost(Cost_t real, Cost_t imag);
	Cost_t		LabDecompressCostReal(Cost_t);
	Cost_t		LabDecompressCostImag(Cost_t);


	Cell_t*		LabGetCell(int8_t x, int8_t y);
	uint8_t		LabIsVisited(uint8_t x, uint8_t y);
	uint8_t		LabIsBlind(uint8_t x, uint8_t y);
	uint8_t		LabIsWall(int8_t x, int8_t y, Wall_t w);
	uint8_t		LabIsIn(int8_t x, int8_t y);
	Wall_t		LabGetParent(LCoord_t x, LCoord_t y);
	Cost_t		LabGetCostReal(LCoord_t x, LCoord_t y);
	Cost_t		LabGetCostImag(LCoord_t x, LCoord_t y);

	void		LabSetBlind(uint8_t x, uint8_t y);
	void		LabSetVisited(uint8_t x, uint8_t y);
	void		LabClearVisited(uint8_t x, uint8_t y);
	void		LabSetWall(int8_t x, int8_t y, Wall_t w);
	void		LabClearWall(int8_t x, int8_t y, Wall_t w);
	void		LabSetParent(LCoord_t x, LCoord_t y, Wall_t p);


	/// return 1 when dir is one of {NE, E, SE}  return -1 when dir is one of {NW, W, SW} and 0 otherwise
	#define LabGetDeltaX(d) LabDeltaX[d]

	/// return 1 when dir is one of {NW, N, NE} return -1 when dir is one of {SW, S, SE} and 0 otherwise
	#define LabGetDeltaY(d) LabDeltaY[d]


	extern Sensor_t g_SWD; // sensor wall data


#ifdef __cplusplus
}
#endif

#endif