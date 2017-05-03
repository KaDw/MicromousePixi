#include <stdbool.h>
#include "map.h"
#include "tim.h"
#include "motor.h"
//#include "usart.h"
//#include "UI.h"
#include "queue.h"
#include "sensor.h"

uint8_t map[MAZE_X][MAZE_Y];

uint8_t direction;
uint8_t x;
uint8_t y;
uint16_t last_pos = 0;
uint16_t pos;

void updateMap(){
	//uint8_t cell_x = motors.mot[0].enc/ONE_CELL_DISTANCE;
	static uint16_t last_pos = 0;
	uint16_t pos = ((EncL + EncR)/2)/ONE_CELL_DISTANCE;
	if(pos > last_pos){
		if(direction == 0)
			y++;
		else if(direction == 2)
			x++;
		else if(direction == 4)
			y--;
		else if(direction == 6)
			x--;
		
		last_pos = pos;
		}
		if(!(map[x][y] & VISITED)){ // not visited
			updateWalls();
			//Floodfill();
			//RightHand();
		}
}

inline bool checkBounds(x, y){
	if(x >= MAZE_X || y >= MAZE_Y || x < 0 || y < 0)
		return false;
	return true;
}


//void addWalls(uint8_t wall){
//	if(wall & WALL_N && y < MAZE_Y-1){
//		map[x][y] |= WALL_N;
//		map[x][y+1] |= WALL_N;
//	}
//	if(wall & WALL_E && x < MAZE_X-1){
//		map[x][y] |= WALL_E;
//		map[x+1][y] |= WALL_E;
//	}
//	if(wall & WALL_S && y >= 0){
//		map[x][y] |= WALL_S; 
//		map[x][y-1] |= WALL_S;	
//	}
//	if(wall & WALL_W && x >= 0){
//		map[x][y] |= WALL_W; 
//		map[x-1][y] |= WALL_W;
//	}
//}


void addWalls(uint8_t wall){
	
	map[x][y] |= wall;
	
	if(wall & WALL_N && y < MAZE_Y-1){
		map[x][y+1] |= WALL_S;
	}
	if(wall & WALL_E && x < MAZE_X-1){
		map[x+1][y] |= WALL_W;
	}
	if(wall & WALL_S && y >= 0){
		map[x][y-1] |= WALL_N;	
	}
	if(wall & WALL_W && x >= 0){
		map[x-1][y] |= WALL_E;
	}
}
	
void updateWalls(){
	uint8_t wall = 0;
	if(SENS_LF > HAS_FRONT_WALL && SENS_RF > HAS_FRONT_WALL){
		// wall |= direction/2
		if(direction == 0){
			wall |= WALL_N;
		}
		else if(direction == 2){
			wall |= WALL_E;
		}
		else if(direction == 4){
			wall |= WALL_S;
		}
		else if(direction == 6){
			wall |= WALL_W;
		}
	}
	if(SENS_RS > HAS_RIGHT_WALL){
		//wall = ((direction/2) & 0x0f);
		if(direction == 0){
			wall |= WALL_E;
		}
		else if(direction == 2){
			wall |= WALL_S;
		}
		else if(direction == 4){
			wall |= WALL_W;
		}
		else if(direction == 6){
			wall |= WALL_N;
		}
	}
	if(SENS_LS > HAS_LEFT_WALL){
		if(direction == 0){
			wall |= WALL_W;
		}
		else if(direction == 2){
			wall |= WALL_N;
		}
		else if(direction == 4){
			wall |= WALL_E;
		}
		else if(direction == 6){
			wall |= WALL_S;
		}
	}
	addWalls(wall);
}

//wall_t getLeft(uint8_t x, uint8_t y, uint8_t direction)
wall_t getLeft(uint8_t direction){
	if(direction == 0)
		return map[x][y] & WALL_W;
	else if(direction == 2)
		return map[x][y] & WALL_N;
	else if(direction == 4)
		return map[x][y] & WALL_E;
	else if(direction == 6)
		return map[x][y] & WALL_S;
}

wall_t getRight(uint8_t direction){
	if(direction == 0)
		return map[x][y] & WALL_E;
	else if(direction == 2)
		return map[x][y] & WALL_S;
	else if(direction == 4)
		return map[x][y] & WALL_W;
	else if(direction == 6)
		return map[x][y] & WALL_N;
}

wall_t getFront(uint8_t direction){
	if(direction == 0)
		return map[x][y] & WALL_N;
	else if(direction == 2)
		return map[x][y] & WALL_E;
	else if(direction == 4)
		return map[x][y] & WALL_S;
	else if(direction == 6)
		return map[x][y] & WALL_W;
}



// bez skosow
void rightHand(){
	if(!getRight(direction))
		q_push(&queue, MotorTurnA, -90, 0, 100);
		//add to queue motor turn right
	else if(!getFront(direction))
		q_push(&queue, MotorGoA, 180, 180, 100);
		// motor go
	else if(!getLeft(direction))
		q_push(&queue, MotorTurnA, 90, 0, 100);
		// motor turn left
	else
		q_push(&queue, MotorTurnA, 180, 0, 100);
		// turn 180, calibrate sensors
		
	// if(!(map[x][y] & direction/2)){
		
	// }

	// if(!(map[x][y] & WALL_E)){
	// 	//dodaj do kolejki skret w prawo
	// 	return;
	// }
	
	
}

void run(){
	updateMap();
	rightHand();
}

// uint8_t getDir(dir){
// 	uint8_t temp_dir = abs(dir);
// 	if(dir == 0){
// 		return 0;
// 	}
// 	else if(dir == 2){
// 		return 1;
// 	}
// 	else if(dir == 4){
// 		return 2;
// 	}
// 	else if(dir == 6){
// 		return 3;
// 	}
// }





