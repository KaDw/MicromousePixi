//
//
//
//

#include "motor.h"


int mmToTicks(int mm)
{
	return TICKS_PER_REVOLUTION*mm/(PI*WHEEL_DIAMETER);
}


int getEncL()
{
	//return hTim3->Instance->CNT;
	return __HAL_TIM_GetCounter(hTim3);
}

int getEncR()
{
	//return hTim4->Instance->CNT;
	return __HAL_TIM_GetCounter(hTim4);
}

void MotorSetVel(Motors* m)
{
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, m->velL>=0 ? 1 : 0);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, m->velL>=0 ? 0 : 1);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, m->velR>=0 ? 1 : 0);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, m->velR>=0 ? 0 : 1);

	//todo: set PWM
}


int MotorNormalizedVel(int vel)
{
	if(vel >= 1000)
		return 1000;
	else if(vel <= -1000)
		return -1000;
	else
		return vel;
}


int MotorEnd(Motors* m)
{
	if( abs(getEncL() - m->ePosL) < MOTOR_EPSILON &&
		  abs(getEncR() - m->ePosR) < MOTOR_EPSILON )
		return 1; //true
	else
		return 0; //false
}



void MotorDriverP(Motors* m)
{
	int trackL, trackR; // current track
	
	// calc track length
	trackL = m->ePosL - getEncL();
	trackR = m->ePosR - getEncR();
	
	// system of equations:
	//   velL+velR = 2*vel
	//   velL/velR = trackL/trackL
	m->velR += KP * MotorNormalizedVel( 2*m->vel*trackR / (abs(trackL)+abs(trackR)) );
	m->velL += KP * MotorNormalizedVel( 2*m->vel - m->velR );
}



void MotorDriverD(Motors* m)
{
	//todo: driver D
	static int counter = 0;
	
	if(++counter < MOTOR_DRIVER_D_CNT)
		return ;
}



int MotorUpdate(Motors* m)
{
	// check additional condition to end
	if( (m->condition!=0 && m->condition()) ||
		!MotorEnd(m) )
	{
		// check and set status
		return 0;
	}
		
	m->velL = m->velR = 0;
	MotorDriverP(m); 
	//MotorDriverD(&velL, &velR, vel);
		
	// normalise
	m->velL = MotorNormalizedVel(m->velL);
	m->velR = MotorNormalizedVel(m->velR);
	
	// set velocity
	MotorSetVel(m);
	
	return 1;
}



int MotorGoTo(Motors* m)
{
	//int ePosL, ePosR; // end position l/r
	//int sPosL, sPosR; //start position left/right
	int cPosL, cPosR; // current position l/r
	int trackL, trackR; // current track
	int velL, velR; // velocity l/r
	
	m->vel = MotorNormalizedVel(m->vel);
	
	//sPosL = cPosL = getEncL();
	//sPosR = cPosR = getEncR();
	
	
	while(!MotorEnd(m))
	{
		// check additional condition
		if(condition!=0 && condition())
			break;
		
		MotorDriverP(&velL, &velR, ePosL, ePosR, vel);
		//MotorDriverD(&velL, &velR, vel);
		
		// normalise
		velL = MotorNormalizedVel(velL);
		velR = MotorNormalizedVel(velR);
		
		// set velocity
		MotorSetVel(velL, velR);
	}
}



void Go(int left, int right, int vel, int(*condition)())
{
	Motors m = {.vel=vel, .condition=condition, .status=MOTOR_RUNNING_STOP};
	m.ePosL = mmToTicks(left) + getEncL();
	m.ePosR = mmToTicks(right) + getEncR();
	
	MotorGoTo(&m);
}



void Turn(int angle, int radius, int vel, int(*condition)())
{
	int trackL, trackR; // mm
	
	trackL = PI*(radius+HALF_WHEELBASE)*angle/180;
	trackR = PI*(radius-HALF_WHEELBASE)*angle/180;
	
	Go(trackL, trackR, vel, condition);
}
