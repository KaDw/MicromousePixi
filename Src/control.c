#include "control.h"


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


// ==== file global variable ====
Cell_t	g_local[3][3]; // local cell map
Dir_t	g_currDir = DIR_N;
uint8_t	g_currX = 0;
uint8_t g_currY = 0;
uint8_t g_mapMirror = 0;


void ConInit(){
}


int ConGaugeDir(DIR_t){
	int value = 0;
	
	if(ten sam dir)
		value += 10;
	if(do centrum)
		value += 15;
	if(brak sciany)
		value += 100;
	if(nowa komorka)
		value += 5;
}


int ConChooseDir(){
	// translate map to local
	// controlGauageDir for each
	// choose the best one
}


void ConlUpdateSensor(){
	//call ADC measurement
	
	// if mirror then change left and right
	// rotate measurements to current direction
	// update global table
}


void ConUpdateLocal(){
	uint8_t x,y;
	
	x = g_currX - 1;
	y = g_currY - 1;
	
	for(uint8_t i = )
}



Wall_t ConWallLoc2Glo(Dir_t currDir, Wall_t localWall){
	assert(currDir <= DIR_W); //ortagonal mode
	Wall_t wall = 0x0f & localWall;
	localWall = localWall&(~0x0f) | 0x0f&(((wall<<currDir) | (wall>>(4-currDir)));
	return localWall;
}

Wall_t ConWallGlo2Loc(Dir_t currDir, Wall_t localWall){
	return ConWallLoc2Glo(4-currDir, localWall);
}



void CTurn(int angle, int radius, int vel, int(*driver)(Motors_t*)){
	if(g_mapMirror)
		angle = -angle;
	
	TurnA(&g_Motors, angle, radius, vel, driver);
}



void CGo(int left, int right, int vel, int(*driver)(Motors_t*)){
	if(g_mapMirror)
		GoA(&g_Motors, right, left, vel, driver);
	else
		GoA(&g_Motors, left, right, vel, driver);
	
}