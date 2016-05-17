//
//
//
//

#include "motor.h"
#include "sensor.h"
#include "fxas21002c.h"
const float HALF_WHEELBASE				= (66/2); /* mm*/
const float TICKS_PER_MM					= (TICKS_PER_REVOLUTION/(PI*WHEEL_DIAMETER));
const float MOTOR_DRIVER_FREQ			= 1000; // Hz
const float MOTOR_DRIVER_T				= 1.f/MOTOR_DRIVER_FREQ;
const int MOTOR_EPSILON 					= 15; /* acceptable position error - enc tick 15~1mm*/
const int ONE_CELL_DISTANCE				= 2725; // ticks

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

float fast_sqrt(float x)
{
	// Heron from Alexandria
	float p = 0.5f*x;
	float lastP = x;
	
	while(lastP-p > 0.1f)
	{
		lastP = p;
		p = 0.5f*(x/p+p);
	}
	
	return p;
}


int mmToTicks(int mm)
{
	return mm*TICKS_PER_REVOLUTION/(PI*WHEEL_DIAMETER);
}


int MotorTruncPWM(int vel)
{
	if(vel >= MOTOR_MAX_PWM)
		return MOTOR_MAX_PWM;
	else if(vel <= -MOTOR_MAX_PWM)
		return -MOTOR_MAX_PWM;
	else
		return vel;
}


//========================
//====== VELOCITY ========
//========================

MotorsV motors;
int _motor_flag = FLAG_ENCODER;

void MotorReset()
{
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	EncL = 0;
	EncR = 0;
	memset(&motors.mot[0], 0, sizeof(_MotorV));
	memset(&motors.mot[1], 0, sizeof(_MotorV));
	//memset(&motors, 0, sizeof(MotorsV));
	//motors.targetV = motors.currentV = motors.PosErrV = motors.lastPosErrV = 0;
	//motors.targetW = motors.currentW = motors.PosErrW = motors.lastPosErrW = 0;
	motors.targetV = motors.currentV = motors.errVI = motors.previousV = 0;
	motors.targetW = motors.currentW = motors.errWI = motors.previousW = 0;
}

void MotorInit()
{
	//todo: turn on timers
	__HAL_TIM_PRESCALER(&MOTOR_HTIM, 4);
	__HAL_TIM_SetAutoreload(&MOTOR_HTIM, MOTOR_MAX_PWM);
	//power 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	
	// filtracja sygnalu
	//MOTOR_HTIM_ENC_L.Instance->SMCR |= TIM_SMCR_ETF_3;
	//MOTOR_HTIM_ENC_R.Instance->SMCR |= TIM_SMCR_ETF_3;
	
	HAL_TIM_PWM_Start(&MOTOR_HTIM, MOTOR_CH_L);
	HAL_TIM_PWM_Start(&MOTOR_HTIM, MOTOR_CH_R);
	HAL_TIM_Encoder_Start(&MOTOR_HTIM_ENC_L, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&MOTOR_HTIM_ENC_R, TIM_CHANNEL_ALL);
	
	MotorReset();
}
	
void MotorUpdateEnc()
{
	int16_t delta;
	
	int16_t el = MOTOR_HTIM_ENC_L.Instance->CNT;
	int16_t er = MOTOR_HTIM_ENC_R.Instance->CNT;
	
	delta = el - motors.mot[0].lastEnc;
	motors.mot[0].encChange = delta;
	motors.mot[0].enc += delta;
	
	delta = er - motors.mot[1].lastEnc;
	motors.mot[1].encChange = delta;
	motors.mot[1].enc += delta;
	
	motors.mot[0].lastEnc = el;
	motors.mot[1].lastEnc = er;
}

void MotorUpdateVelocity()
{
	// == translation velocity ==
	if(motors.currentV < motors.targetV) // accelerate
	{		
		motors.currentV += MOTOR_ACC_V*MOTOR_DRIVER_T;
		
		if(motors.currentV > motors.targetV)
			motors.currentV = motors.targetV;
	}
	else if(motors.currentV > motors.targetV)
	{
		motors.currentV -= MOTOR_ACC_V*MOTOR_DRIVER_T;
		if(motors.currentV < motors.targetV)
			motors.currentV = motors.targetV;
	}
	
	// breaking
	if(motors.distLeft < motors.Sbreak) // decelerate
	{
		// we use distance to calc velocity to stop right where we want
		// and whe override previous de/acceleration block based on MOTOR_ACC_V and MOTOR_DRIVER_T
		motors.currentV = motors.targetV * motors.distLeft/(motors.Sbreak);
		if(motors.currentV > 0 && abs(motors.distLeft) > MOTOR_EPSILON)
			motors.currentV += 20;
		else if(motors.currentV < 0 && abs(motors.distLeft) > MOTOR_EPSILON)
			motors.currentV -= 20;
	}
	
	
	// == rotation velocity ==
	if(motors.currentW < motors.currentW) // accelerate
	{
		motors.currentW += MOTOR_ACC_W;
		if(motors.currentW > motors.targetW)
			motors.currentW = motors.targetW;
	}
	else if(motors.currentW > motors.currentW) // deceleration
	{
		motors.currentW -= MOTOR_ACC_W;
		if(motors.currentW < motors.targetW)
			motors.currentW = motors.targetW;
	}
	
	//printf("Vel:%f  Dist:%d", motors.currentV, motors.distLeft);
}

void MotorDriver()
{
	float rotationalFeedback = 0;
	float encoderVFeedback;
	float errorV, errorW;
	int PwmV, PwmW;
	int leftCh = motors.mot[0].encChange;
	int rightCh = motors.mot[1].encChange;
	
	motors.distLeft -= leftCh + rightCh; // imp
	
  // == PID ==
	// based on speed from 2 encoders
	encoderVFeedback = leftCh + rightCh; // [imp/T] use abs to measure wheel speed, or don't use abs (and change TurnA) and measure robot speed
	
	if(_motor_flag & FLAG_ENCODER)
	{
		if(leftCh != rightCh)
		{
			float alpha = 0.5f*(rightCh-leftCh)/HALF_WHEELBASE; // rad
			float encoderWFeedback = alpha*MOTOR_DRIVER_FREQ; // rad / s
			rotationalFeedback += encoderWFeedback;
		}
	}
	
	if(_motor_flag & FLAG_GYRO)
	{
		rotationalFeedback += sensorGyroW;
	}
	
	if(_motor_flag & FLAG_SENSOR)
	{ // 1100 LMiddleValue 920 RMiddleValue
		float sensorFeedback = 0;//sensorError/a_scale;
		if(sens[2] > 1100 && sens[3] < 920)
			//sensorFeedback = sens[2] - sens[3];
			sensorFeedback = 1100 - sens[2];
		else if(sens[3] > 920 && sens[2] < 1100)
			sensorFeedback = sens[3] - 920;
		else
			sensorFeedback = 0;
		rotationalFeedback += 0.01*sensorFeedback;
	}
	
	//printf("PWM_W: %f\r\n", rotationalFeedback);

	// errorV = cV*0.017 - encoderVFeedback
	errorV = motors.currentV*MOTOR_DRIVER_T*TICKS_PER_MM - encoderVFeedback; // [mm/s]*T -> imp
	errorW = motors.currentW - rotationalFeedback;
	
	// == error in PID == 
	motors.errVD = errorV - motors.errVP;
	motors.errWD = errorW - motors.errWP;
	motors.errVI += errorV;
	motors.errWI += errorW;
	motors.errVP = errorV;
	motors.errWP = errorW;
	
	PwmV = (int)(MOTOR_VELV_KP*errorV + MOTOR_VELV_KI*motors.errVI + MOTOR_VELV_KD*motors.errVD);
	PwmW = MOTOR_VELW_KP*errorW + MOTOR_VELW_KI*motors.errWI + MOTOR_VELW_KD*motors.errWD;
	
	// == I limitation == 
	//if((PwmV < -MOTOR_MAX_PWM) || (MOTOR_MAX_PWM < PwmV))
	//	motors.errVI -= errorV;
	//if((PwmW < -MOTOR_MAX_PWM) || (MOTOR_MAX_PWM < PwmW))
	//	motors.errWI -= errorW;
	
	motors.mot[0].PWM = PwmV - PwmW;
	motors.mot[1].PWM = PwmV + PwmW;
	
	//printf("u");
	//printf("cV:%f\tD:%d\terrV:%f\terrVI:%f\tPWM:%d\t\r\n", motors.currentV, motors.distLeft, errorV, motors.errVI, PwmV);
}

void MotorSetPWM()
{
	int VL = MotorTruncPWM(motors.mot[0].PWM)/2;
	int VR = MotorTruncPWM(motors.mot[1].PWM)/2;
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, VL>=0 ? GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, VL>=0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, VR<=0 ? GPIO_PIN_SET 	: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, VR<=0 ? GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, abs(VL));
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, abs(VR));
}

void MotorUpdate()
{
	//while(1) {MotorUpdateEnc(); printf("L:%d\t R:%d\r\n", motors.mot[0].enc, motors.mot[1].enc); HAL_Delay(50); }
	//GyroReadData();
	MotorUpdateEnc();
	MotorUpdateVelocity();
	MotorDriver();
	MotorSetPWM();
	//printf("tarV:%f errV:%d errR:%d PwmL:%d PwmR:%d ledtf:%d\r\n", motors.targetV, rotationalFeedback, motors.mot[0].PWM, motors.mot[1].PWM, motors.distLeft);
	//printf("L:%d  R:%d\r\n", motors.mot[0].PWM, motors.mot[0].PWM);
	//static int lastDistLeft;
	//printf("%d\r\n", -(motors.distLeft - lastDistLeft));
	//lastDistLeft = motors.distLeft;
}

void MotorGoA(int left, int right, float vel) // [mm] [mm] [mm/s]
{	
	float w = 0;
	float dV = vel - motors.previousV; // [mm/s]
	float Sbreak = 2.0f*0.5f*vel*vel/MOTOR_ACC_V;// [mm] = mm/s * mm/s * s*s/mm		double S because we accumulate dX from 2 wheels
	float Tacc = dV/MOTOR_ACC_V; // [s]
	int Sacc = 2.0f*(motors.previousV*Tacc + 0.5f*dV*dV/MOTOR_ACC_V);// mm = mm/s * mm/s * s*s/mm		double S because we accumulate dX from 2 wheels
	
	// when vel is big and dist is short
	// then deccelerate has higher priority in
	if(Sacc + Sbreak > left + right)
	{
		vel = fast_sqrt(0.01f*(MOTOR_ACC_V*(left+right) + 0.5f*motors.previousV*motors.previousV)); // mm/s/s*mm + mm*mm/s/s
		dV = vel - motors.previousV;
		Sbreak = 2.0f*0.5f*vel*vel/MOTOR_ACC_V;
		Tacc = dV/MOTOR_ACC_V;
		Sacc = 2.0f*(motors.previousV*Tacc + 0.5f*dV*dV/MOTOR_ACC_V);
	}
	
	// calc W speed
	if(left != right)
	{
		float a = 0.5f*(left-right)/HALF_WHEELBASE; // rad
		w = a * vel / (abs(left)+abs(right)); // [rad/s] = rad * [mm/s] / mm
	}
		
	motors.previousV = vel;
	motors.Sbreak = mmToTicks(Sbreak);// mm/s * mm/s * s*s/mm = mm
	motors.distLeft = mmToTicks(left + right); // 2 wheel distance
	motors.targetV = 2*vel;// becouse we accumutade dX from 2 wheels
	motors.targetW = w;
	MotorUpdateEnc(); // to delete EncChange
	
	//printf("D:%d\t Tacc:%dT\t Sacc:%d\t Sbr:%d\r\n", motors.distLeft, (int)(Tacc*MOTOR_DRIVER_FREQ), mmToTicks(Sacc), motors.Sbreak);
}

void MotorTurnA(int angle, int r, float vel)
{
	int dist = abs(r+HALF_WHEELBASE) + abs(r-HALF_WHEELBASE);
	motors.distLeft = mmToTicks(2.0f*PI*dist);
	motors.targetV = vel;
	motors.targetW = vel/(abs(r)+HALF_WHEELBASE);
	motors.Sbreak = mmToTicks(2.0f*0.5f*vel*vel/MOTOR_ACC_V);
	//MotorGoA(2*PI*(r+HALF_WHEELBASE)*angle*0.0027777778f, 2*PI*(r-HALF_WHEELBASE)*angle*0.0027777778f, vel);
}

void MotorGo(int left, int right, float vel)
{
	MotorGoA(left, right, vel);
	while(motors.distLeft > MOTOR_EPSILON)
	{
		MotorUpdate();
		UI_DelayUs(990);
	}	
	MotorStop();
	//ADCreadChannel(CH9, vbat);
}

void MotorTurn(int angle, int r, float vel)
{
	MotorTurnA(angle, r, vel);
	while(motors.distLeft > MOTOR_EPSILON)
	{
		MotorUpdate();
		UI_DelayUs(990);
	}
	
}

void MotorSetPWMRaw(int left, int right)
{
	left = MotorTruncPWM(left);
	right = MotorTruncPWM(right);
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, left>=0 ? 	GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, left>=0 ? 	GPIO_PIN_RESET 	: GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, right<=0 ? 	GPIO_PIN_SET 		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, right<=0 ? 	GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, abs(left));
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, abs(right));
}


void MotorStop()
{
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_RESET);
	
	//todo: set PWM 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);

}

//========================
//====== POSITION ========
//========================
/*
Motors_t motors;

void MotorInit()
{
	//todo: turn on timers
	__HAL_TIM_PRESCALER(&MOTOR_HTIM, 4);
	__HAL_TIM_SetAutoreload(&MOTOR_HTIM, MOTOR_MAX_PWM);
	//power 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	
	//MOTOR_HTIM_ENC_L.Instance->
	
	// filtracja sygnalu
	//MOTOR_HTIM_ENC_L.Instance->SMCR |= TIM_SMCR_ETF_3;
	//MOTOR_HTIM_ENC_R.Instance->SMCR |= TIM_SMCR_ETF_3;
	
	HAL_TIM_PWM_Start(&MOTOR_HTIM, MOTOR_CH_L);
	HAL_TIM_PWM_Start(&MOTOR_HTIM, MOTOR_CH_R);
	HAL_TIM_Encoder_Start(&MOTOR_HTIM_ENC_L, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&MOTOR_HTIM_ENC_R, TIM_CHANNEL_ALL);
	
	//TIM3->CNT = 0;
	//TIM4->CNT = 0;
	MOTOR_HTIM_ENC_L.Instance->CNT = 0;
	MOTOR_HTIM_ENC_R.Instance->CNT = 0;
	motors.mot[0].enc = 0;
	motors.mot[1].enc = 0;
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
	left = MotorTruncPWM(left);
	right = MotorTruncPWM(right);
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, left>=0 ? 	GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, left>=0 ? 	GPIO_PIN_RESET 	: GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, right<=0 ? 	GPIO_PIN_SET 		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, right<=0 ? 	GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, abs(left));
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, abs(right));
}


void MotorSetPWM()
{
	int VL = MotorTruncPWM(motors.mot[0].PWM)/4;
	int VR = MotorTruncPWM(motors.mot[1].PWM)/4;
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, VL>=0 ? GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, VL>=0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, VR<=0 ? GPIO_PIN_SET 	: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, VR<=0 ? GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, abs(VL));
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, abs(VR));
}



void MotorStop()
{
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_RESET);
	
	//todo: set PWM 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	
		motors.status = MOTOR_STOPPED;
		motors.driver = 0;
}



void MotorFloat() // standby
{	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_RESET);
	
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, MOTOR_MAX_PWM);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, MOTOR_MAX_PWM);
	
	motors.status = MOTOR_FLOATING;
	motors.driver = 0;
}



int MotorEnd()
{
	//if( abs(getEncL() - motors.mot[0].S3) < MOTOR_EPSILON &&
	//	  abs(getEncR() - motors.mot[1].S3) < MOTOR_EPSILON )
	if( abs(getEncL()-motors.mot[0].S3) + abs(getEncR()-motors.mot[1].S3) < 2*MOTOR_EPSILON)
		return 1; //true
	else
		return 0; //false
}


void MotorUpdateEnc()
{
	int16_t delta;
	int16_t el = MOTOR_HTIM_ENC_L.Instance->CNT;
	int16_t er = MOTOR_HTIM_ENC_R.Instance->CNT;
	
	delta = el - motors.mot[0].lastEnc;
	motors.mot[0].encChange = delta;
	motors.mot[0].enc += delta;
	
	delta = er - motors.mot[1].lastEnc;
	motors.mot[1].encChange = delta;
	motors.mot[1].enc += delta;
	
	motors.mot[0].lastEnc = el;
	motors.mot[1].lastEnc = er;
}


int MotorGetS(_Motor_t* m) // return absolute ticks
{
	int s;
	float t = MOTOR_DRIVER_T * motors.t; // s
	
	if(motors.t < m->T1)
	{
		if(m->V > m->lastV)
			s = TICKS_PER_MM * (m->lastV*t + 0.5f*t*t*MOTOR_ACC_V); // ticks
		else
			s = TICKS_PER_MM * (m->lastV*t - 0.5f*t*t*MOTOR_ACC_V); // ticks
		
		s += m->S0;
	}
	else if(motors.t < m->T2)
		s = m->S1 + TICKS_PER_MM*m->V*(t-MOTOR_DRIVER_T*m->T1); // ticks
	else if(motors.t < m->T3)
	{
		t = MOTOR_DRIVER_T*m->T3 - t; // yes, T3 -> just draw it to undertand
		s = m->S3 - 0.5f*TICKS_PER_MM*MOTOR_ACC_V*t*t;
	}
	else
		s = m->S3;
	
	return s;
}

void MotorDriverV(_Motor_t* m)
{
	int s = MotorGetS(m);
	int e = s - m->enc;
	
	m->errD = e - m->errP;
	m->errI += e;
	m->errP = e;
	
	m->PWMV = MOTOR_VELV_KP*e + MOTOR_VELV_KI*m->errI + MOTOR_VELV_KD*m->errD;
	
	// prevent integral from windapping 
	//if(abs(m->PWMV) > MOTOR_MAX_PWM)
	//	m->errI -= e;
}



int MotorUpdate()
{
	if(motors.t < motors.mot[0].T3)
		++motors.t;
	
	MotorUpdateEnc();
	
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
		
	
	// == position PID ==
	MotorDriverV(&motors.mot[0]);
	MotorDriverV(&motors.mot[1]);
	
	// == rotation PID ==
	
	// call! additional driver and eventually break function
	// in this plce current velocity can be modified as well
	if(motors.driver!=0 && motors.driver(&motors))
		return 0;
	
	// == sum ==
	motors.mot[0].PWM = motors.mot[0].PWMV;
	motors.mot[1].PWM = motors.mot[1].PWMV;
	
	if(motors.mot[0].PWMV < 0)
	{
		MotorStop();
		//printf("Zwrot! t:%d[T] %f[s]  d:%d[tick] %f[mm]\r\n", motors.t, MOTOR_DRIVER_T*motors.t, MotorGetS(&motors.mot[0]), MotorGetS(&motors.mot[0])/TICKS_PER_MM);
		while(1);
	}
	
	// set velocity
	MotorSetPWM();
	
	//printf("ErrP:%f  ErrI:%f  PWM:%d  s:%d ds:%d\r\n", motors.mot[0].errP, motors.mot[0].errI, motors.mot[0].PWM, MotorGetS(&motors.mot[0]), MotorGetS(&motors.mot[0]) - motors.mot[0].enc);
	//printf_("ErrP:%d  ErrI:%d  PWM:%d  s:%d ds:%d\r\n", (int)(motors.mot[0].errP), (int)(motors.mot[0].errI), motors.mot[0].PWM, MotorGetS(&motors.mot[0]), MotorGetS(&motors.mot[0]) - motors.mot[0].enc);
	
//	static int i = 1;
//	if(i<20)
//	{
//		printf("ErrP:%f  ErrI:%f  PWM:%d  s:%d\r\n", motors.mot[0].errP, motors.mot[0].errI, motors.mot[0].PWM, MotorGetS(&motors.mot[0]));
//		++i;
//	}
	
	return 1;
}


void MotorFillStruct(_Motor_t* m, int track, float V, int b_break)
{
	m->V = V; // [mm/s]
	float dV = V - m->lastV; // [mm/s]
	
	// accelerate
	float Tacc = dV/MOTOR_ACC_V; //[s]
	float Sacc = m->lastV*Tacc + 0.5f*dV*Tacc; // [mm] = [mm/s] * s + [mm/s] * [s]
	
	// deccelerate
	float Tbr = 0;
	float Sbr = 0;
	if(b_break)
	{
		Tbr = V/MOTOR_ACC_V; //[s]
		Sbr = 0.5f*(V*V/MOTOR_ACC_V); // [mm]
	}
	
	
	// when V is high and wS is low
	// then for precise calculating you need sqrt
	// so we approximate it with a linear 
	if(Sacc + Sbr > track)
	{
		V = fast_sqrt(track*MOTOR_ACC_V + 0.5f*m->lastV*m->lastV);
		dV = V - m->lastV;
		Tacc = dV / MOTOR_ACC_V;
		Sacc = m->lastV*Tacc + 0.5f*dV*Tacc;
		if(b_break)
		{
			Tbr = V/MOTOR_ACC_V; //[s]
			Sbr = 0.5f*(V*V/MOTOR_ACC_V); // [mm]
		}
	}
	
	// const Velocity
	float Sm = track - Sacc - Sbr; //[mm]
	float Tm = Sm / V; // [s]
	
	m->T1 = (int)(MOTOR_DRIVER_FREQ*Tacc); // T - here [ms]
	m->T2 = m->T1 + (int)(MOTOR_DRIVER_FREQ*Tm); // T - here [ms]
	m->T3 = m->T2 + (int)(MOTOR_DRIVER_FREQ*Tbr); // T - here [ms]
	m->S0 = m->enc; // ticks
	m->S1 = m->S0 + (int)(TICKS_PER_MM*Sacc); // [tick]
	m->S2 = m->S1 + (int)(TICKS_PER_MM*Sm); // [tick]
	m->S3 = m->S0 + (int)(TICKS_PER_MM*track); // [tick]
	m->wS = mmToTicks(track); // [ticks]
	
	printf("V:%d  lastV:%d track:%d\r\n", m->V, m->lastV, track);
	printf("T1:%d\t  T2:%d\t  T3:%d\t [T]\r\n", m->T1, m->T2, m->T3);
	printf("T1:%f\t  T2:%f\t  T3:%f\t [s]\r\n", Tacc, Tacc+Tm, Tacc+Tm+Tbr);
	printf("S0:%d\t  S1:%d\t  S2:%d\t  S3:%d\t  wS:%d\t [ticks]\r\n", m->S0, m->S1, m->S2, m->S3, m->wS);
	printf("Sacc:%d\t  Sm:%d\t  Sbr:%d\t  Sum:%d\t [mm]\r\n", (int)(Sacc), (int)(Sm), (int)(Sbr), (int)(Sacc+Sm+Sbr));
}


void MotorGoA(int left, int right, float vel, int(*driver)(Motors_t*)) // mm mm [mm/s]
{
	int brake = 1;
	
	if(left==0 && right==0)
	{
		MotorStop();
		return;
	}

	motors.t = 0; // T
	motors.vel = vel; // [mm/s]
	motors.driver = driver;
	motors.status = MOTOR_RUNNING_STOP;
	
	//motors.mot[0].enc = getEncL();
	//motors.mot[1].enc = getEncR();
	MotorUpdateEnc();
	
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



void Go(int left, int right, float vel, int(*driver)(Motors_t*))
{
	int delay = 1000/MOTOR_DRIVER_FREQ;
	MotorGoA(left, right, vel, driver);	
	
	while(MotorUpdate())
		UI_DelayUs(990);
}



void TurnA(int angle, int radius, int vel, int(*driver)(Motors_t*))
{
	int trackL, trackR; // mm
	
	trackL = PI*(radius+HALF_WHEELBASE)*angle/180;
	trackR = PI*(radius-HALF_WHEELBASE)*angle/180;
	
	MotorGoA(trackL, trackR, vel, driver);
}



void Turn(int angle, int radius, int vel, int(*driver)(Motors_t*))
{ 
	int trackL, trackR; // mm
	
	trackL = PI*(radius+HALF_WHEELBASE)*angle/180;
	trackR = PI*(radius-HALF_WHEELBASE)*angle/180;
	
	MotorGoA(trackL, trackR, vel, driver);
}*/
