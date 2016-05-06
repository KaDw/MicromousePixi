//
//
//
//

#include "motor.h"


int abs(int x)
{
	if(x < 0)
		return -x;
	else
		return x;
}

int sgn(int x)
{
	return (x>0)-(x<0);
}


int mmToTicks(int mm)
{
	return TICKS_PER_REVOLUTION*mm/(PI*WHEEL_DIAMETER);
}



void MotorInit()
{
	//todo: turn on timers
	__HAL_TIM_PRESCALER(&MOTOR_HTIM, 180);
	__HAL_TIM_SetAutoreload(&MOTOR_HTIM, MOTOR_MAX_VEL);
	//power 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	
	//MOTOR_HTIM_ENC_L.Instance->
	
	// filtracja sygnalu
	MOTOR_HTIM_ENC_L.Instance->SMCR |= TIM_SMCR_ETF_3;
	MOTOR_HTIM_ENC_R.Instance->SMCR |= TIM_SMCR_ETF_3;
	
	HAL_TIM_PWM_Start(&MOTOR_HTIM, MOTOR_CH_L);
	HAL_TIM_PWM_Start(&MOTOR_HTIM, MOTOR_CH_R);
	HAL_TIM_Encoder_Start(&MOTOR_HTIM_ENC_L, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&MOTOR_HTIM_ENC_R, TIM_CHANNEL_ALL);
}



int getEncL()
{
	//return hTim3->Instance->CNT;
	return __HAL_TIM_GetCounter(&htim3);
}



int getEncR()
{
	//return hTim4->Instance->CNT;
	return __HAL_TIM_GetCounter(&htim4);
}



void MotorSetPWMRaw(int left, int right)
{
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, left>=0 ? 	GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, left>=0 ? 	GPIO_PIN_RESET 	: GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, right<=0 ? 	GPIO_PIN_SET 		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, right<=0 ? 	GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, left);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, right);
}


void MotorSetPWM(Motors_t* m)
{
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, m->velL>=0 ? GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, m->velL>=0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, m->velR<=0 ? GPIO_PIN_SET 	: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, m->velR<=0 ? GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, m->velL >> 1);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, m->velR >> 1);
}



void MotorStop(Motors_t* m)
{
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_RESET);
	
	//todo: set PWM 100% when IN=00
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, MOTOR_MAX_VEL);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, MOTOR_MAX_VEL);
	
	if(m != 0) {
		m->status = MOTOR_STOPPED;
		m->driver = 0;
	}
}



void MotorFloat(Motors_t* m) // standby
{
	m->velL = MotorTruncVel(m->velL);
	m->velR = MotorTruncVel(m->velR);
	
	//todo: chrck value
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_SET);
	
	//todo: set PWM 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	
	if(m != 0){
		m->status = MOTOR_FLOATING;
		m->driver = 0;
	}
}



int MotorTruncVel(int vel)
{
	if(vel >= MOTOR_MAX_VEL)
		return MOTOR_MAX_VEL;
	else if(vel <= -MOTOR_MAX_VEL)
		return -MOTOR_MAX_VEL;
	else
		return vel;
}



int MotorEnd(Motors_t* m)
{
	if( abs(getEncL() - m->ePosL) < MOTOR_EPSILON &&
		  abs(getEncR() - m->ePosR) < MOTOR_EPSILON )
		return 1; //true
	else
		return 0; //false
}



void MotorDriverP1(Motors_t* m)
{
	int16_t trackL, trackR; // current track
	
	// calc track length
	trackL = (m->ePosL - getEncL());
	trackR = (m->ePosR - getEncR());
	
	// system of equations:
	//   velL+velR = 2*vel
	//   velL/velR = trackL/trackL
	m->velR += MotorTruncVel( KP * 2*m->vel*trackR / (abs(trackL)+abs(trackR)) ); // there is no div by 0
	m->velL += MotorTruncVel( KP * (2*m->vel - m->velR) );
	
	
	printf("E: l:%d p:%d   V: l:%d  p:%d   Track: lewy:%d  prawy:%d  -1:%d\n\r", getEncL(), getEncR(), m->velL, m->velR, trackL, trackR, -1);
	//todo: check end condition of reference call
	
	//sprawdz czy nic nie wychodzi poza zakres predkosci, ew. przelicz jeszcze raz
	/*if(m->velR>MOTOR_MAX_VEL || m->velR<-MOTOR_MAX_VEL)
	{
		m->vel *= (float)MOTOR_MAX_VEL / (float)m->velR; // there is no div by 0
		// ze wzgledu na zaokroglenia odejmnij jeszcze troche
		m->vel -= 10;
		
		MotorDriverP(m);
	}
	
	if(m->velL>MOTOR_MAX_VEL || m->velL<MOTOR_MAX_VEL)
	{
		m->vel *= (float)MOTOR_MAX_VEL / (float)m->velL; // there is no div by 0
		// ze wzgledu na zaokroglenia odejmnij jeszcze troche
		m->vel -= 10;
		
		MotorDriverP(m);
	}*/
}

void MotorDriverPraw(int errL, int errR)
{
	
}

void MotorDriverP(Motors_t* m)
{
	/*static int16_t startL, startR; // pozycja poczatku ruchu
	static int16_t lastEndL=0, lastEndR=0; // pozycja konca ruchu*/
	int16_t trackL, trackR;
	int VL, VR;
	int error;
	
	
	trackL = m->ePosL - getEncL();
	trackR = m->ePosR - getEncR();
	error = ((trackL*(m->wTrackR)/(m->wTrackL)) - abs(trackR)) >> 2; //in this place never wTrackL==0
	
	// where there is new target defined
	/*if(m->ePosL!=lastEndL || m->ePosR!=lastEndR)
	{
		startL = getEncL();
		startR = getEncR();
		lastEndL = m->ePosL;
		lastEndR = m->ePosR;
	}*/
	
	// if left wheel is faster
	if(abs(trackL) > abs(trackR))
	{
		// calc error of slower wheel corresponding to faster wheel
		
		// then write vel to velL with correct sign
		VL = abs(trackL)>MOTOR_SLOW_TICK ? sgn(trackL)*m->vel : sgn(trackL)*MOTOR_SLOW_VEL;
		
		// velR is proportional smaller plus error coefficient
		if(abs(trackR)>MOTOR_SLOW_TICK) //                    wholeTrackR * % of complete left track  - current trackR  
			VR = m->vel*trackR/abs(trackL) + error; 
		else
			VR = MOTOR_SLOW_VEL*sgn(trackR);
	}
	// right velocity is bigger or this same
	else
	{
		// calc error of slower wheel corresponding to faster wheel
		error = trackR*(m->wTrackL)/(m->wTrackR) - abs(trackL); //in this place never lastEndL=startL
		
		// then write vel to velR with correct sign
		VR = abs(trackR)>MOTOR_SLOW_TICK ? sgn(trackR)*m->vel : sgn(trackR)*MOTOR_SLOW_VEL;
		
		// velR is proportional smaller plus error coefficient
		if(abs(trackL)>MOTOR_SLOW_TICK) //                    wholeTrackR * % of complete left track  - current trackR  
			VL = m->vel*trackL/abs(trackR) - error;
		else
			VL = MOTOR_SLOW_VEL*sgn(trackL);
	}
	
	if(abs(trackL)>MOTOR_EPSILON && abs(VL)<MOTOR_SLOW_TICK)
		VL = sgn(VL)*MOTOR_SLOW_TICK;
	
	if(abs(trackR)>MOTOR_EPSILON && abs(VR)<MOTOR_SLOW_TICK)
		VR = sgn(VR)*MOTOR_SLOW_TICK;
	
	m->velL += KP * VL;
	m->velR += KP * VR;
	
	printf("V: L:%d  P:%d   T: L:%d  P:%d  E: l:%d p:%d   -1:%d\n\r", m->velL, m->velR, trackL, trackR, getEncL(), getEncR(), -1);
}


void MotorDriverP3(Motors_t* m)
{
	const float k = 1.0f;
	int16_t trackL, trackR;
	int error;
	
	trackL = abs(m->ePosL - getEncL());
	trackR = abs(m->ePosR - getEncR());
	
	// first phase of movement - accelerating
	if(trackL < (m->wTrackL>>1))
	{
		int vel = k*m->wTrackL + MOTOR_SLOW_VEL;
	}
	else
	{
	}
}

void MotorDriverD(Motors_t* m)
{
	//todo: driver D
	static int counter = 0;
	static int last_velL = 0, last_velR = 0;
	static int diffL = 0, diffR = 0;
	
	// calculate actual differencial
	if(++counter > MOTOR_DRIVER_D_CNT)
	{
		diffL = MotorTruncVel( 0.5f*diffL + 0.5f*(m->velL-last_velL) );
		diffR = MotorTruncVel( 0.5f*diffR + 0.5f*(m->velR-last_velR) );
		last_velL = m->velL;
		last_velR = m->velR;
	}
	
	m->velL += KD * diffL;
	m->velR += KD * diffR;
}



int MotorUpdate(Motors_t* m)
{	
	int errL, errR;
	
	// check condition to end
	if(MotorEnd(m))
	{
		// check and set status
		if(m->status!=MOTOR_STOPPED && m->status!=MOTOR_FLOATING)
		{
			if		(m->status == MOTOR_RUNNING_STOP)				MotorStop(m);
			else if (m->status == MOTOR_RUNNING_FLOAT) 		MotorFloat(m);
			//todo: else Error("Motor corrupted status");
		}
		return 0;
	}
		
	m->velL = m->velR = 0;
	MotorDriverP(m); 
	//MotorDriverD(&velL, &velR, vel);
	
	// call! additional driver and eventually break function
	// in this plce current velocity can be modified as well
	if(m->driver!=0 && m->driver(m))
		return 0;
	
	// normalise
	m->velL = MotorTruncVel(m->velL);
	m->velR = MotorTruncVel(m->velR);
	
	// set velocity
	//MotorSetPWM(m);
	
	return 1;
}



void GoA(Motors_t* m, int left, int right, int vel, int(*driver)(Motors_t*))
{
	vel=MotorTruncVel(abs(vel));
	
	m->vel = vel;
	m->driver = driver;
	m->status = MOTOR_RUNNING_STOP;
	m->wTrackL = mmToTicks(left);
	m->wTrackR = mmToTicks(right);
	m->ePosL = m->wTrackL + getEncL();
	m->ePosR = m->wTrackR + getEncR();
}



void Go(int left, int right, int vel, int(*driver)(Motors_t*))
{
	Motors_t m;
	GoA(&m, left, right, vel, driver);	
	while(MotorUpdate(&m))
	{
	}
}



void TurnA(Motors_t* m, int angle, int radius, int vel, int(*driver)(Motors_t*))
{
	int trackL, trackR; // mm
	
	trackL = PI*(radius+HALF_WHEELBASE)*angle/180;
	trackR = PI*(radius-HALF_WHEELBASE)*angle/180;
	
	GoA(m, trackL, trackR, vel, driver);
}



void Turn(int angle, int radius, int vel, int(*driver)(Motors_t*))
{ 
	int trackL, trackR; // mm
	
	trackL = PI*(radius+HALF_WHEELBASE)*angle/180;
	trackR = PI*(radius-HALF_WHEELBASE)*angle/180;
	
	Go(trackL, trackR, vel, driver);
}
