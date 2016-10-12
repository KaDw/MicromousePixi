#include "motor.h"
#include "sensor.h"
#include "fxas21002c.h"
const float 	WHEELBASE							= 66;
const float 	HALF_WHEELBASE				= (WHEELBASE/2); /* mm*/
const float 	TICKS_PER_MM					= (TICKS_PER_REVOLUTION/(PI*WHEEL_DIAMETER));
const float 	MOTOR_DRIVER_FREQ			= 1000.f; // Hz
const float 	MOTOR_DRIVER_T				= 1.f/MOTOR_DRIVER_FREQ;
const float 	MOTOR_EPSILON_W				= 0.08; /* acceptable rotation error in radians*/
const int 		MOTOR_EPSILON 					= 15; /* acceptable position error - enc tick 15~1mm*/
const int 		ONE_CELL_DISTANCE				= 2725; // ticks

int MotorUpdateStatus(void);
void MotorUpdateEncoder(void);
void MotorUpdateVariable(void);

int abs(int x)
{
	if(x < 0)
		return -x;
	else
		return x;
}

float _fabs(float x)
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


int MotorTruncPWM(float vel)
{
	if(vel >= MOTOR_MAX_PWM)
		return MOTOR_MAX_PWM;
	else if(vel <= -MOTOR_MAX_PWM)
		return -MOTOR_MAX_PWM;
	else
		return vel;
}

void MotorStepResponse(uint16_t PwmL, uint16_t PwmR, uint16_t time)
{
		static uint16_t test_mode = 0;
		uint8_t buf[2];
		uint32_t enc;
			
		if(!test_mode){
			MotorSetPWMRaw(PwmL, PwmR); // make mouse go straight
		}
		if(test_mode > time){
			MotorSetPWMRaw(0, 0);
			HAL_UART_DeInit(&huart1); // stop sending data
		}
			//_itoa(37, aTxBuffer);
		enc = EncL;
		TIM3->CNT = 0;
		buf[0] = enc >> 8;
		buf[1] = enc;
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, 2);
		test_mode++;
}

//========================
//====== VELOCITY ========
//========================

MotorsV motors;
int _motor_flag = 0;

void MotorReset()
{
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	EncL = 0;
	EncR = 0;
	memset(&motors.mot[0], 0, sizeof(_MotorV));
	memset(&motors.mot[1], 0, sizeof(_MotorV));
}

void MotorResetEnc(_MotorV* m)
{
	m->enc = 0;
	m->encChange = 0;
	m->lastEnc = 0;
	m->idealEnc = 0;
}

void MotorInit()
{
	//todo: turn on timers
	__HAL_TIM_PRESCALER(&MOTOR_HTIM, 7);
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
	// te zmienne powinny byz liczbami calkowitymi ze znakiem
	// oraz miec taka sama wielkosc jak rejestr przechowujacy
	// impulsy z eknkodera - patrz przejscia CNT przez 0!
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

int MotorUpdateStatus()
{
	if(_fabs(motors.mot[0].vel) > 0.0001f || _fabs(motors.mot[0].targetVel) > 0.0001f ||
		_fabs(motors.mot[1].vel) > 0.0001f || _fabs(motors.mot[1].targetVel) > 0.0001f)
		return 0; // motor running
	else
		return 1; // motor stopped
}


/// increase/decrease 'temp' by 'by' to get closer to 'to'
void _stepTo(float* temp, float to, float step)
{
	if(*temp < to) // accelerate
	{
		*temp += step;
		if(*temp > to)
			*temp = to;
	}
	else if(*temp > to) // deccelerate
	{
		*temp -= step;
		if(*temp < to)
			*temp = to;
	}
}


void MotorUpdateVariable()
{
	// velocity
	_stepTo(&motors.mot[0].vel, motors.mot[0].targetVel, MOTOR_ACC_V*MOTOR_DRIVER_T);
	_stepTo(&motors.mot[1].vel, motors.mot[1].targetVel, MOTOR_ACC_V*MOTOR_DRIVER_T);
	
	// timing
	if(motors.time > 1) // decrement translation velocity timer
		--motors.time;
	else if(motors.time == 1)
	{
		--motors.time;
		motors.mot[0].targetVel = 0;
		motors.mot[1].targetVel = 0;
	}
	
	// position
	motors.mot[0].idealEnc += (int)(motors.mot[0].vel*MOTOR_DRIVER_T*TICKS_PER_MM);
	motors.mot[1].idealEnc += (int)(motors.mot[1].vel*MOTOR_DRIVER_T*TICKS_PER_MM);
}


void MotorDriver()
{
	int err;
	for(int i = 0; i < 2; ++i)
	{
		// oblicz uchyba jako roznice pomiedzy pozycja idelana i ta rzeczywista
		err = motors.mot[i].idealEnc - motors.mot[i].enc;
		motors.mot[i].errD = motors.mot[i].errP - err;
		motors.mot[i].errI+= err;
		motors.mot[i].errP = err;
		// sprzezenie od bledu pozycji
		motors.mot[i].PWM = (int)(5.f*(float)(motors.mot[i].errP) // 500
														+ 0.f*(float)(motors.mot[i].errI) // 1000
														+ 0.8f*(float)(motors.mot[i].errD)); // 56
		// sprzezenie od zadanej predkosci
		motors.mot[i].PWM += (int)(0.15f*motors.mot[i].vel);
	}
	
	// sprzezenie od bledu pozycji 2. silnika
//	motors.mot[0].PWM += -(int)(3.f*motors.mot[1].errP);
//	motors.mot[1].PWM += -(int)(3.f*motors.mot[0].errP);
}


void MotorUpdate()
{
	if(MOTOR_FREEZE())
		return;
	MotorUpdateEnc();
	MotorUpdateVariable();
	MotorDriver();
	MotorSetPWM();
}

float _MotorCalcTime(float lastV, float vel, float s)
{
	float Tacc = abs(lastV-vel)/MOTOR_ACC_V;
	float Sacc = (lastV+vel)*0.5f*Tacc;
	float Tcon = abs((s-Sacc)/vel);
	float T = 0;
	//gdy tylko przyspieszamy, to czas jest opisany zaleznoscia
	//t^2 + t*2*v0/a - s*s/a = 0
	if(abs(Sacc) > abs(s)){
		s = abs(s);
		T = (-lastV + fast_sqrt(lastV*lastV+2*s*MOTOR_ACC_V))/MOTOR_ACC_V;
	}
	//gdy przyspieszamy oraz poruszamy sie ze stala predkoscia
	//T = Tacc + Tconst
	else if(_fabs(vel) > 0.001f)
		T = Tacc+Tcon;
		
	return T;
}

float MotorCalcS(float lastV, float vel, float t)
{
	 float Tacc = abs((vel-lastV)/MOTOR_ACC_V);
   float Sacc = (lastV+vel)*0.5f*Tacc;
   float Scon = vel*(t-Tacc);
		 
   if(t > Tacc)
			return Sacc+Scon;
    else
			return Sacc;
}
		

float _MototorCalcTime(int s, int previuousV, int vel, float acc) // [mm], [mm/s], [mm/s], [mm/s/s]
{
	// returns time needed for acceleration+movement with constat velocity
	float Tacc = abs(previuousV-vel)/(float)(acc);
	int   Sacc = (previuousV+vel)*Tacc*0.5f;
	float T = 0;
	if(vel != 0)
		T = Tacc + abs(s-Sacc)/(float)(vel);
	return T;
}

int _MotorCalcVel(int s, int previousV, float T, float acc) // [mm], [mm/s], [s], [mm/s/s]
{
	// rozwiazania rowniania kwadratowego
	// zapewniajacego spelnienie podanych
	// warunkow (parametrow)
	// zakladamy ze vel > previousV
	float a = 1.f;
	float b = -2.0f * acc * T;
	float c = 2.0f * acc * s - previousV*previousV;
	float delta = b*b - 4.f*a*c;
	delta = fast_sqrt(delta);
	float vel = (-b-delta) / (4.f*a*c);
	if(vel < previousV) // gdy przyjecte zalozenia nie sa spelnjione
		vel = (-b+delta) / (4.f*a*c);
	
	if(vel < previousV) // gdy podane zalozenia dalej nie sa spelnione
	{ // to zamien znaki w funkcji kwadratowej (abs)
		a = 1.f;
		b = 2.f*acc*T;
		c = -2.f*acc - previousV*previousV;
		delta = b*b - 4.f*a*c;
		delta = fast_sqrt(delta);
		vel = (-b-delta) / (4.f*a*c);
		if( vel > previousV) // gdy pierwiastek nie spelnia zalozen
			vel = (-b+delta) / (4.f*a*c);
	}
	
	return vel;
}


float MotorCalcVel(float lastV, float s, float t)
{
	//ale gdy mamy rownierz poruszanie sie z predkoscia stala,
	//to musimy rozwiazac ponizsze rownanie z zalozeniami:
	float a = MOTOR_ACC_V;	
		
	if(lastV*t > s)
		a = -a;
	//zakladamy, ze v > lastV
	float p = (a*t*t + 2*lastV*t - 2*s)/a;
	float S = 0;
	float V;
	if( p >= 0){
		p = fast_sqrt(p);
		float V = lastV - a*p + a*t;
		S = MotorCalcS(lastV, V, t);
		
		
		if(_fabs(S - s) > 5)
			V = lastV + a * p + a * t;
	}
	// w akcie rozpaczy (gdy nie idzie liczenie tego) - to uprosc
	// obliczenia dobrane metoda doswiadczalna
	// inne opcje to:
	// a) V = s/t //gdy t!=0
	// b) p = math.sqrt(a*t*t + 2*lastV*t + 2*s)/a)
	//    V = lastV + a * p + a * t
	// c) wesola tworczosc
	else
		V = lastV + a*t;
	
	return V;
}


void MotorGoA(int left, int right, float vel) // [mm] [mm] [mm/s]
{
//	float time = (abs(left)+abs(right))*0.5f/vel;
//	MOTOR_FREEZE_EN();
//	motors.mot[0].targetVel = (int)(left/time);
//	motors.mot[1].targetVel = (int)(right/time);
//	motors.time = (int)(time*MOTOR_DRIVER_FREQ); //convert [s] to [T]
//	MOTOR_FREEZE_DIS();
//	// now MotorUpdate polling in interrupt will handle this query
	float Vl = motors.mot[0].vel;
	float Vr = motors.mot[1].vel;
	float Tl = _MototorCalcTime(left,  Vl, vel, MOTOR_ACC_V);
	float Tr = _MototorCalcTime(right, Vr, vel, MOTOR_ACC_V);
	float T;
	
	// wyjustuj do wolniejszego silnika
	if(Tl > Tr)
	{
		T = Tl;
		Vl = vel;
		Vr = _MotorCalcVel(right, Vr, T, MOTOR_ACC_V); //wyrownujemy, do dluzszego czasu 
	}
	else
	{
		T = Tr;
		Vl = _MotorCalcVel(left, Vl, T, MOTOR_ACC_V);
		Vr = vel;
	}
	
	// ustaw obliczone parametry ruchu
	MOTOR_FREEZE_EN();
	motors.mot[0].vel = Vl;
	motors.mot[1].vel = Vr;
	motors.time = T*MOTOR_DRIVER_FREQ;
	MOTOR_FREEZE_DIS();
}

void MotorTurnA(int angle, int r, float vel)
{
	int left  = PI*(r+HALF_WHEELBASE)*angle/180;
	int right = PI*(r-HALF_WHEELBASE)*angle/180;
	
	MotorResetEnc(&motors.mot[0]);
	MotorResetEnc(&motors.mot[1]);
	MotorGoA(left, right, vel);
}

void MotorRotR90A()
{
	MotorTurnA(90, 0, 500);
}

void MotorSetVel(int velocity, int omega)
{
	motors.time = 0;
	motors.mot[0].targetVel = velocity + omega*180.f/PI*HALF_WHEELBASE;
	motors.mot[1].targetVel = velocity - omega*180.f/PI*HALF_WHEELBASE;
}


void MotorGo(int left, int right, float vel)
{ 
	MotorGoA(left, right, vel);
	while(!MotorUpdateStatus())
	{
	}
}

void MotorTurn(int angle, int r, float vel)
{
	MotorTurnA(angle, r, vel);
	while(!MotorUpdateStatus())
	{
	}
}

void MotorSetPWM()
{
	int VL = MotorTruncPWM(motors.mot[0].PWM);
	int VR = MotorTruncPWM(motors.mot[1].PWM);
	
	// power limiter
	int thres = 500;
	if(VL*VL > thres*thres)
		VL = thres*sgn(VL);
	if(VR*VR > thres*thres)
		VR = thres*sgn(VR);
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, VL>=0 ? GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, VL>=0 ? GPIO_PIN_RESET  : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, VR<=0 ? GPIO_PIN_SET 	  : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, VR<=0 ? GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, abs(VL));
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, abs(VR));
}

void MotorSetPWMRaw(int left, int right)
{
	motors.mot[0].PWM = left;
	motors.mot[1].PWM = right;
	MotorSetPWM();
}


void MotorStop()
{
	MOTOR_FREEZE_EN();
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_RESET);
	
	//todo: set PWM 0%
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, 0);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, 0);
	
	motors.status = MOTOR_STOPPED;
	motors.mot[0].targetVel = 0;
	motors.mot[0].targetVel = 0;
}

void MotorFloat() // standby
{
	MOTOR_FREEZE_EN();
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, GPIO_PIN_RESET);
	
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, MOTOR_MAX_PWM);
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, MOTOR_MAX_PWM);
	
	motors.status = MOTOR_FLOATING;
	motors.mot[0].targetVel = 0;
	motors.mot[0].targetVel = 0;
}

