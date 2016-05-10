//
//
//
//

#include "motor.h"


Motors_t motors;


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
	__HAL_TIM_PRESCALER(&MOTOR_HTIM, 4);
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
	
	//TIM3->CNT = 0;
	//TIM4->CNT = 0;
	motors.mot[0].enc = getEncL();
	motors.mot[1].enc = getEncR();
	motors.mot[0].lastV = 0;
	motors.mot[1].lastV = 0;
	motors.mot[0].errI = 0;
	motors.mot[1].errI = 0;
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
	left = MotorTruncVel(left);
	right = MotorTruncVel(right);
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, left>=0 ? 	GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, left>=0 ? 	GPIO_PIN_RESET 	: GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, right<=0 ? 	GPIO_PIN_SET 		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, right<=0 ? 	GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, left);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, right);
}


void MotorSetPWM()
{
	int VL = MotorTruncVel(motors.mot[0].PWM);
	int VR = MotorTruncVel(motors.mot[1].PWM);
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, VL>=0 ? GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, VL>=0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, VR<=0 ? GPIO_PIN_SET 	: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, VR<=0 ? GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, VL);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, VR);
}



void MotorStop()
{
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_RESET);
	
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, MOTOR_MAX_VEL);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, MOTOR_MAX_VEL);
	
		motors.status = MOTOR_STOPPED;
		motors.driver = 0;
}



void MotorFloat() // standby
{	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_SET);
	
	//todo: set PWM 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	
	motors.status = MOTOR_FLOATING;
	motors.driver = 0;
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



int MotorEnd()
{
	if( abs(getEncL() - motors.mot[0].S3) < MOTOR_EPSILON &&
		  abs(getEncR() - motors.mot[1].S3) < MOTOR_EPSILON )
		return 1; //true
	else
		return 0; //false
}




void MotorDriverP(Motors_t* m)
{
	/*static int16_t startL, startR; // pozycja poczatku ruchu
	static int16_t lastEndL=0, lastEndR=0; // pozycja konca ruchu*/
	int16_t trackL, trackR;
	int VL, VR;
	int error;
	
	trackL = motors.mot[0].S3 - getEncL();
	trackR = motors.mot[1].S3 - getEncR();
	error = (( motors.mot[0].wS*(motors.mot[1].wS)/(motors.mot[0].wS)) - abs(trackR)) >> 2; //in this place never wTrackL==0
	
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
		error = trackR*(motors.mot[0].wS)/(motors.mot[1].wS) - abs(trackL); //in this place never lastEndL=startL
		
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
	
	motors.mot[0].PWM = KP * VL;
	motors.mot[1].PWM = KP * VR;
	
	//printf("V: L:%d  P:%d   T: L:%d  P:%d  E: l:%d p:%d   -1:%d\n\r", m->velL, m->velR, trackL, trackR, getEncL(), getEncR(), -1);
}


void MotorDriver(_Motor_t* m)
{
	pos_t s;
	posDif_t e;
	++motors.t;
	int t = motors.t;
	
	if(t < m->T1)
	{
		if(m->V > m->lastV)
			s = m->lastV*t + ((t*t*MOTOR_ACC)>>1);
		else
			s = m->lastV*t - ((t*t*MOTOR_ACC)>>1);
	}
	else if(t < m->T2)
		s = m->S1 + m->V * (t-m->T1);
	else if(t < m->T3)
	{
		t = m->T3-t; // yes, T3 -> just draw it to undertand
		s = m->S3 - (MOTOR_ACC*t*t>>1);
	}
	else
	{
		s = m->S3;
	}
	
	// error calc
	e = s - m->enc;
	
	if(abs(m->err) < 600 || abs(m->err+e)<abs(m->err))
		m->errI += e;
	
	m->errP = e - m->err;
	m->err = e;
	
	// I is not active during movement
	// only when you have been stopped by nonlinearity
	//if(abs(m->errP) > 3)
		//m->errI = 0;
	
	m->PWM = KI*m->errI;
	
	if(abs(m->PWM) > 350)
		m->PWM = sgn(m->PWM)*350;
	
	m->PWM = KP*e + KD*m->errP;
	
	if(t < 6)
	{
		if(m->PWM > 0)
			m->PWM += 300;
		else if(m->PWM < 0)
			m->PWM -= 300;
	}
	
	static int i = 1;
	if(i<20)
	{
		printf("Err:%d  ErrP:%d  ErrI:%d  PWM:%d  s:%d\n", e, m->errP, m->errI, m->PWM, s);
		//printf("t:%d t:%d s:%d\n", motors.t, t, s);
		
		/*if(e > 1000 || m->PWM > 700)
		{
			MotorStop();
			while(1){
				//printf("Err:%d  ErrP:%d  errI:%d PWM:%d  s:%d\n", e, m->errP, m->errI, m->PWM, s);
				UI_LedOn(UI_LED_L);
				UI_LedOn(UI_LED_R);
				HAL_Delay(500);
			}
		}*/
		++i;
	}
}

void MotorDriverD(Motors_t* m)
{
	//todo: driver D
	//static int counter = 0;
	//static int last_velL = 0, last_velR = 0;
	//static int diffL = 0, diffR = 0;
	
	// calculate actual differencial
	//if(++counter > MOTOR_DRIVER_D_CNT)
	//{
	//	diffL = MotorTruncVel( 0.5f*diffL + 0.5f*(m->velL-last_velL) );
	//	diffR = MotorTruncVel( 0.5f*diffR + 0.5f*(m->velR-last_velR) );
	//	last_velL = m->velL;
	//	last_velR = m->velR;
	//}
	
	//m->velL += KD * diffL;
	//m->velR += KD * diffR;
}



int MotorUpdate()
{
	motors.mot[0].enc = getEncL();
	motors.mot[1].enc = getEncR();
	
	// check condition to end
	if(MotorEnd())
	//if(motors.t >= motors.mot[0].T3 && motors.t >= motors.mot[1].T3)
	{
		// check and set status
		if(motors.status!=MOTOR_STOPPED && motors.status!=MOTOR_FLOATING)
		{
			if (motors.status == MOTOR_RUNNING_STOP)
			{
				MotorStop();
				motors.mot[0].lastV = 0;
				motors.mot[1].lastV = 0;
			}
			else if (motors.status == MOTOR_RUNNING_FLOAT)
			{
				MotorFloat();
				motors.mot[0].lastV = motors.mot[0].V;
				motors.mot[1].lastV = motors.mot[1].V;
			}
			//todo: else Error("Motor corrupted status");
		}
		
		return 0;
	}
		
	MotorDriver(&motors.mot[0]); 
	MotorDriver(&motors.mot[1]); 
	
	// call! additional driver and eventually break function
	// in this plce current velocity can be modified as well
	if(motors.driver!=0 && motors.driver(&motors))
		return 0;
	
	// normalise
	//m->velL = MotorTruncVel(m->velL);
	//m->velR = MotorTruncVel(m->velR);
	//m->velL = m->mot[0].PWM;
	//m->velR = m->mot[1].PWM;
	
	// set velocity
	MotorSetPWM();
	
	return 1;
}


void MotorFillStruct(_Motor_t* m, uint16_t track, int V, int b_break)
{
	m->wS = mmToTicks(track);
	m->V = V;
	
	int dV = V - m->lastV;
	
	// accelerate
	m->T1 = dV/MOTOR_ACC;
	int Sacc =  m->lastV*m->T1 + ((dV*m->T1)>>1);
	m->S1 = m->enc;
	m->S1 = m->enc + Sacc;
	
	// deccelerate
	int Tbr = 0;
	uint16_t Sbr = 0;
	if(b_break)
	{
		Tbr = V/MOTOR_ACC;
		Sbr = (V*V/MOTOR_ACC)>>1;
	}
	
	// const Velocity
	uint16_t Sm = m->wS - Sacc - Sbr;
	int Tm = Sm / V;
	
	m->T2 = m->T1 + Tm;
	m->S2 = m->S1 + Sm;
	
	m->T3 = m->T2 + Tbr;
	m->S3 = m->enc + m->wS;
	
	// when V is high and wS is low
	// then for precise calculating you need sqrt
	// so we approximate it with const a
	if(Sacc + Sbr > m->wS)
	{
		m->S1 = m->enc;
		m->S2 = m->S1 + m->wS;
		m->S3 = m->S2;
		m->T1 = 0;
		m->T2 = m->wS / m->V;
		m->T3 = m->T2;
	}
	
	printf("V:%d  lastV:%d\n", m->V, m->lastV);
	printf("T1:%d  T2:%d  T3:%d\n", m->T1, m->T2, m->T3);
	printf("S1:%d  S2:%d  S3:%d  wS:%d\n", m->S1, m->S2, m->S3, m->wS);
}


void GoA(int left, int right, int vel, int(*driver)(Motors_t*))
{
	int brake = 1;
	vel=MotorTruncVel(abs(vel));
	
	if(left==0 && right==0)
	{
		MotorStop();
		return;
	}
	
	vel = mmToTicks(vel)/MOTOR_DRIVER_FREQ; // /f
	motors.t = 0;
	motors.vel = vel;
	motors.driver = driver;
	motors.status = MOTOR_RUNNING_STOP;
	
	//m->mot[0].enc = (int16_t*)(&TIM3->CNT);
	//m->mot[1].enc = (int16_t*)(&TIM4->CNT);
	motors.mot[0].enc = getEncL();
	motors.mot[1].enc = getEncR();
	
	if(left > right)
	{
		MotorFillStruct(&motors.mot[0], left, vel, brake);
		MotorFillStruct(&motors.mot[1], right, vel*right/left, brake); // left!=0
	}
	else
	{
		MotorFillStruct(&motors.mot[0], left, vel*left/right, brake); // right!=0
		MotorFillStruct(&motors.mot[1], right, vel, brake);
	}
}



void Go(int left, int right, int vel, int(*driver)(Motors_t*))
{
	int delay = 1000/MOTOR_DRIVER_FREQ;
	GoA(left, right, vel, driver);	
	
	while(MotorUpdate())
		HAL_Delay(delay);
}



void TurnA(int angle, int radius, int vel, int(*driver)(Motors_t*))
{
	int trackL, trackR; // mm
	
	trackL = PI*(radius+HALF_WHEELBASE)*angle/180;
	trackR = PI*(radius-HALF_WHEELBASE)*angle/180;
	
	GoA(trackL, trackR, vel, driver);
}



void Turn(int angle, int radius, int vel, int(*driver)(Motors_t*))
{ 
	int trackL, trackR; // mm
	
	trackL = PI*(radius+HALF_WHEELBASE)*angle/180;
	trackR = PI*(radius-HALF_WHEELBASE)*angle/180;
	
	Go(trackL, trackR, vel, driver);
}
