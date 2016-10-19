#include "motor.h"
#include "sensor.h"
#include "fxas21002c.h"
#include <math.h>
const float 	WHEELBASE							= 66;
const float 	HALF_WHEELBASE				= (WHEELBASE/2); /* mm*/
const float 	TICKS_PER_MM					= (TICKS_PER_REVOLUTION/(PI*WHEEL_DIAMETER));
float const 	MOTOR_DRIVER_FREQ			= 1000.f; // Hz
float					MOTOR_DRIVER_T				= 1.f/MOTOR_DRIVER_FREQ;
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

//float _fabs(float x)
//{
//	if(x < 0)
//		return -x;
//	else
//		return x;
//}

int sgn(int x)
{
	return (x>0)-(x<0);
}

int _roundf(float x)
{
	// zaszumianie
	static float szum = 0;
	szum += 0.013f;
	if(szum > 0.5f)
		szum -= 1.0f;
	x += szum;
	
	if(x < 0)
		return (int)(x+0.5f);
	else
		return (int)(x+0.5f);
}

//float fast_sqrt(float x)
//{
//	// Heron from Alexandria
//	float p = 0.5f*x;
//	float lastP = x;
//	
//	while(lastP-p > 0.1f)
//	{
//		lastP = p;
//		p = 0.5f*(x/p+p);
//	}
//	
//	return p;
//}


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

void MotorPrintData()
{
		uint8_t buf[2];
		uint32_t enc;
		//_itoa(37, aTxBuffer);
		enc = EncL;
		TIM3->CNT = 0;
		buf[0] = enc >> 8;
		buf[1] = enc;
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, 2);
}

//========================
//====== VELOCITY ========
//========================

MotorsV motors;
int _motor_flag = 0;
int motor_error = 0;

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
	MOTOR_FREEZE_EN();
	m->enc = 0;
	m->encChange = 0;
	m->lastEnc = 0;
	m->idealEnc = 0;
	MOTOR_FREEZE_DIS();
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
	if(fabs(motors.mot[0].targetVel) > 0.1f || fabs(motors.mot[1].targetVel) > 0.01f)
		return 0; // motor running
	else
		return 1; // motor stopped
}


/// update accelerate and velocity
/// increase/decrease 'temp' by 'by' to get closer to 'to'
void _MotorUpdateAV(_MotorV* mot)
{
	if(mot->vel < mot->targetVel) // accelerate
	{
		mot->a = MOTOR_ACC_V;
		mot->vel += mot->a*MOTOR_DRIVER_T;
		if(mot->vel > mot->targetVel)
			mot->vel = mot->targetVel;
	}
	else if(mot->vel > mot->targetVel) // deccelerate
	{
		mot->a = -MOTOR_ACC_V;
		mot->vel += mot->a*MOTOR_DRIVER_T;
		if(mot->vel < mot->targetVel)
			mot->vel = mot->targetVel;
	}
	else{ 
		mot->a = 0;
	}
}


void MotorUpdateVariable()
{
	// velocity
	_MotorUpdateAV(&motors.mot[0]);
	_MotorUpdateAV(&motors.mot[1]);
	
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
	if(!motors.mot[0].vel){
		++motors.mot[0].vel;
		--motors.mot[0].vel;
	}
	
	motors.mot[0].idealEnc += _roundf(motors.mot[0].vel*MOTOR_DRIVER_T*TICKS_PER_MM);
	motors.mot[1].idealEnc += _roundf(motors.mot[1].vel*MOTOR_DRIVER_T*TICKS_PER_MM);
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
		motors.mot[i].PWM = (int)(3.f*((float)(motors.mot[i].errP) // 500
														+ 0.0f*(float)(motors.mot[i].errI) // 1000
														+ 0.5f*(float)(motors.mot[i].errD))); // 56
		// sprzezenie od zadanej predkosci
		motors.mot[i].PWM += (int)(0.17f*(motors.mot[i].vel
														+ 0.3f*motors.mot[i].a)); // 0.03
	}
	
	// sprzezenie od bledu pozycji 2. silnika
	//motors.mot[0].PWM += -(int)(3.f*motors.mot[1].errP);
	//motors.mot[1].PWM += -(int)(3.f*motors.mot[0].errP);
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
	
	if(fabs(vel) < 0.001)
		return 0;
	// returns time needed for acceleration+movement with constat velocity
	float a = MOTOR_ACC_V;
	float Tacc = fabs(vel-lastV) / a;
	float Sacc = (lastV+vel)*0.5f*Tacc;
	float Tcon = fabs((s-Sacc)/vel); // czas ruchu jednostajnego
	float T;
	//gdy tylko przyspieszamy, to czas jest opisany zaleznoscia
	//t^2 + t*2*v0/a - s*s/a = 0
	if(fabs(Sacc) > fabs(s)){
		s = fabs(s);
		T = (-lastV + sqrtf(lastV*lastV+2*s*a)) / a;
		if(T < 0)
			T = (lastV + sqrtf(lastV*lastV+2*s*a)) / a;
	//gdy przyspieszamy oraz poruszamy sie ze stala predkoscia
	//T = Tacc + Tconst
	}
	else
		T = Tacc + Tcon;
	
	return T;
}


float _MotorCalcS(float lastV, float vel, float t)
{
	 float a = MOTOR_ACC_V;
	 float Tacc = fabs((vel-lastV) / a);
   float Sacc = (lastV+vel)*0.5f*Tacc;
   float Scon = vel*(t-Tacc);
		 
   if(t > Tacc)
			return Sacc+Scon;
    else
			return Sacc;
}


float _MotorCalcVel(float lastV, float s, float t) // [mm], [mm/s], [s]
{
	// sprawdzamy, czy mamy przyspieszac, czy zwalniac
	float a = (lastV*t)>s ? -MOTOR_ACC_V : MOTOR_ACC_V;
	float V, p;
	float S = 0;
	
	p = (a*t*t + 2*lastV*t -2*s) / a;
	if(p < 0){
		p = (a*t*t + 2*lastV*t + 2*s)/a;
    p = sqrtf(p);
	}
	if(p >= 0)
	{
		p = sqrtf(p);
		V = lastV - (a*p) + (a*t);
		S = _MotorCalcS(lastV, V, t);
		if(fabs(S-s) > 5) // gdy wyilczona predkosc jest zla
			V = lastV + a*p + a*t; // to probujemy 2 rozwiazanie
	}
	else
	{ // obliczenia nie ida za dobrze
		// w akcie rozpaczy upraszczamy sobie rownania
		// doswiadczalnie zostalo dobrane to rownanie
		// inne opcje to: V = s/t (gdy t!=0), lub cos zupelnie innego
		MOTOR_ERR(MOTOR_ERR_CALC_VEL);
		V = lastV + a*t;
	}
	return V;
}


void MotorGoA(int left, int right, float vel) // [mm] [mm] [mm/s]
{
	vel = fabs(vel);
	float lVl = motors.mot[0].vel; //last vel left
	float lVr = motors.mot[1].vel;
	float Vl = vel;
	float Vr = vel;
	float T;
	
	if(left < 0)
			Vl = -vel;
	if(right < 0)
			Vr = -vel;
	
	// oczlicz czas potrzebny na ruch kazdego z kol
	// i wybierz to ktore potrzebuje go wiecej
	// jako czas calego ruchu
	float Tl = _MotorCalcTime(lVl, Vl, left);
	float Tr = _MotorCalcTime(lVr, Vr, right);
	if(Tl > Tr) // takie same jesli jedziemy prosto
			T = Tl;
	else
			T = Tr;
	
	// gdy juz znasz czas, to dopasuj predkosci kazdego z kol
	// i wpisz rezultaty do struktury, aby zaczely byc obslugiwane
	// przez MotorDriver()
	Vl = _MotorCalcVel(lVl, left,  T);
	Vr = _MotorCalcVel(lVr, right, T);
	MOTOR_FREEZE_EN();
	motors.mot[0].targetVel = Vl;
	motors.mot[1].targetVel = Vr;
	motors.time = T*MOTOR_DRIVER_FREQ;
	MOTOR_FREEZE_DIS();
}


void MotorTurnA(int angle, int r, float vel)
{
	int left, right;
	
	if(angle < 0 && r > 0) // left forward
	{
		left  = -PI*(r-HALF_WHEELBASE)*angle/180;
		right = -PI*(r+HALF_WHEELBASE)*angle/180;
	}
	else if( angle > 0 && r > 0) // right forward
	{
		left  = PI*(r+HALF_WHEELBASE)*angle/180;
		right = PI*(r-HALF_WHEELBASE)*angle/180;
	}
	else if( angle < 0 && r < 0) // left backward
	{
		left  = -PI*(r+HALF_WHEELBASE)*angle/180;
		right = -PI*(r-HALF_WHEELBASE)*angle/180;
	}
	else // right backward
	{
		left  = PI*(r-HALF_WHEELBASE)*angle/180;
		right = PI*(r+HALF_WHEELBASE)*angle/180;
	}
	
	//MotorResetEnc(&motors.mot[0]);
	//MotorResetEnc(&motors.mot[1]);
	MotorGoA(left, right, vel);
}


void MotorSetVel(int velocity, int omega)
{
	motors.time = 0;
	motors.mot[0].targetVel = velocity + omega*180.f/PI*HALF_WHEELBASE;
	motors.mot[1].targetVel = velocity - omega*180.f/PI*HALF_WHEELBASE;
}


void MotorGo(int left, int right, float vel)
{ 
	int32_t end;
	MotorGoA(left, right, vel);
	while(!MotorUpdateStatus())
	{
		end = UI_Timestamp();
		MotorUpdate();
		ADCreadAmbient();
		//MotorPrintData();
		while(UI_TimeElapsedUs(end) < MOTOR_DRIVER_T*1000000);
	}
	MotorSetPWMRaw(0,0);
}

void MotorTurn(int angle, int r, float vel)
{
	int32_t end;
	MotorTurnA(angle, r, vel);
	while(!MotorUpdateStatus())
	{
		end = UI_Timestamp();
		MotorUpdate();
		ADCreadAmbient();
		while(UI_TimeElapsedUs(end) < MOTOR_DRIVER_T*1000000);
	}
	MotorSetPWMRaw(0,0);
}

void MotorSetPWM()
{
	int VL = MotorTruncPWM(motors.mot[0].PWM);
	int VR = MotorTruncPWM(motors.mot[1].PWM);
	
	// power limiter
	int thres = 250;
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

