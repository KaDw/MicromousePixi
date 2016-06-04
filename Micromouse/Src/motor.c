//
//
//
//

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
void MotorUpdateVelocity(void);

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
int _motor_flag = 0;

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

int MotorUpdateStatus()
{
	if(motors.currentV != 0 || motors.targetV != 0 ||
		motors.currentW != 0 || motors.targetW != 0)
		return 0; // motor running
	else
		return 1; // motor stopped
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
	
	
	// == rotation velocity ==
	if(motors.currentW < motors.targetW) // accelerate
	{
		motors.currentW += MOTOR_ACC_W*MOTOR_DRIVER_T;
		if(motors.currentW > motors.targetW)
			motors.currentW = motors.targetW;
	}
	else if(motors.currentW > motors.targetW) // deceleration
	{
		motors.currentW -= MOTOR_ACC_W*MOTOR_DRIVER_T;
		if(motors.currentW < motors.targetW)
			motors.currentW = motors.targetW;
	}
}

void MotorDriver()
{
	float rotationalFeedback = 0; // right+ left-
	float encoderVFeedback;
	float errorV, errorW;
	int PwmV, PwmW;
	int leftCh = motors.mot[0].encChange;
	int rightCh = motors.mot[1].encChange;
	
  // == PID ==
	// based on speed from 2 encoders
	encoderVFeedback = leftCh + rightCh; // [imp/T] use abs to measure wheel speed, or don't use abs (and change TurnA) and measure robot speed
	
	if(_motor_flag & FLAG_ENCODER)
	{
		if(leftCh != rightCh)
		{
			float alpha = 0.5f*(leftCh-rightCh)/(TICKS_PER_MM*HALF_WHEELBASE); // rad
			float encoderWFeedback = alpha;  // rad
			rotationalFeedback += encoderWFeedback;
		}
	}
	
	if(_motor_flag & FLAG_GYRO)
	{
		//rotationalFeedback += (-sensorGyroW)*MOTOR_DRIVER_T*PI*0.00277777777777777777777777777778f; // /360
		motors.desAlpha += motors.currentW*MOTOR_DRIVER_T*180.f/PI - (-sensorGyroA);
	}
	
	if(_motor_flag & FLAG_SENSOR)
	{ // 1100 LMiddleValue 920 RMiddleValue
		float sensorFeedback = 0;//sensorError/a_scale;
		if(SENS_RS < 60)
				sensorFeedback = (float)(SENS_RS) - 30;
//		else if(sens[2] > 1100)
//			sensorFeedback = 1100 - sens[2];
//		else if(sens[3] > 920)
//			sensorFeedback = sens[3] - 920;
//		else
//			sensorFeedback = sens[2] - sens[3];
		rotationalFeedback -= 1*sensorFeedback;
	}

	// == decrement movement timer ==
	if(motors.timev > 0)
		--motors.timev;
	else
		motors.targetV = 0;
	
	if(motors.timew > 0)
		--motors.timew;
	else
		motors.targetW = 0;
	
	
	// == error calc == 
	errorV = motors.currentV*MOTOR_DRIVER_T*TICKS_PER_MM - encoderVFeedback; // [mm/s]*T -> imp
	errorW = motors.currentW*MOTOR_DRIVER_T							 - rotationalFeedback;
	
	motors.errVD = errorV - motors.errVP;
	motors.errWD = errorW - motors.errWP;
	motors.errVI += errorV;
	motors.errWI += errorW;
	motors.errVP = errorV;
	motors.errWP = errorW;
	
	PwmV = (int)(MOTOR_VELV_KP*errorV + MOTOR_VELV_KI*motors.errVI + MOTOR_VELV_KD*motors.errVD);
	PwmW = (int)(MOTOR_VELW_KP*errorW + MOTOR_VELW_KI*motors.errWI + MOTOR_VELW_KD*motors.errWD);
	
	// == I limitation == 
	//if((PwmV < -MOTOR_MAX_PWM) || (MOTOR_MAX_PWM < PwmV))
	//	motors.errVI -= errorV;
	//if((PwmW < -MOTOR_MAX_PWM) || (MOTOR_MAX_PWM < PwmW))
	//	motors.errWI -= errorW;
	
	//printf("%d\t %d\t snes:%f\t feedW:%f\t rot:%f\r\n",SENS_L, SENS_R, sensorFeedback, rotationalFeedback, errorW);
	//printf("%d\t %d\r\n",SENS_L, SENS_R);
		
	motors.mot[0].PWM = PwmV + PwmW;
	motors.mot[1].PWM = PwmV - PwmW;
	
	//printf("tV:%f\t cV:%f\tDV:%d\tDW:%f\tcW:%f\ttW:%f\r\n", motors.targetV, motors.currentV, motors.distLeftV, motors.distLeftW, motors.currentW, motors.targetW);
}

void MotorUpdate()
{
	GyroGetAngle(0.001); // time in secnods
	MotorUpdateEnc();
	//if(MotorUpdateStatus()) return;// return 1 when motor driver should be called
	MotorUpdateVelocity();
	MotorDriver();
	MotorSetPWM();
}

void MotorFindParams(float previousV, float* v, float dist, float acc, int* time)
{
	float vel = *v;
	float dV = vel - previousV; // [mm/s]
	float Tacc = dV/acc; // [s]
	if(Tacc < 0) Tacc = -Tacc;
	float Sbreak = 0.5f*vel*vel/acc;// [mm] = mm/s * mm/s * s*s/mm
	float Sacc = (previousV*Tacc + 0.5f*dV*dV/acc);// mm = mm/s * mm/s * s*s/mm
	
	if(Sacc + Sbreak > dist)
	{
		vel = fast_sqrt(0.5f*(acc*dist + previousV*previousV)); // mm/s/s*mm + mm*mm/s/s
		dV = vel - previousV;
		Tacc = dV/acc;
		Sacc = 2.0f*(0.5f*(previousV*Tacc + dV*dV/acc));
		Sbreak = 2.0f*0.5f*vel*vel/acc;
	}
	
	*time = (Tacc + 0.5f*(dist - Sacc - Sbreak) / vel) * MOTOR_DRIVER_FREQ; // [T]
}

void MotorGoA(int left, int right, float vel) // [mm] [mm] [mm/s]
{		
	MotorFindParams(motors.previousV, &vel, 0.5f*(left+right), MOTOR_ACC_V, &motors.timev);
	motors.targetV = vel;
	
	// calc W speed
	if(left != right)
	{
		float a = 0.5f*(left-right)/HALF_WHEELBASE; // rad
		float w = a * vel * 2.f / (abs(left)+abs(right)); // [rad/s] = rad * [mm/s] / mm
		MotorFindParams(motors.previousW, &w, a, MOTOR_ACC_W, &motors.timew);
		motors.targetW = w;
	}
	else
		motors.targetW = 0;
	
	if(left+right >= 0)
	{
		motors.previousV = vel;
		motors.targetV = 2*vel;// becouse we accumutade dX from 2 wheels
	}
	else
	{
		motors.previousV = -vel;
		motors.targetV = -2*vel;
	}
	
	MotorUpdateEnc(); // to delete EncChange
}

void MotorTurnA(int angle, int r, float vel)
{
	//int dist = r+HALF_WHEELBASE + r-HALF_WHEELBASE;
	motors.targetV = 0;
	motors.timev = 0;
	motors.targetW = vel/(abs(r)+HALF_WHEELBASE);
	
	MotorUpdateEnc(); // to delete EncChange
}

void MotorRotR90A()
{
	motors.targetV = 0;
	motors.timev = 0;
	motors.targetW = 50.f;
	motors.timew = 3*0.5605f*MOTOR_DRIVER_FREQ;
	motors.desAlpha = sensorGyroA;
	MotorUpdateEnc(); // to update start position
}


/*int MotorUpdateStatus()
{
	if(abs(motors.distLeftV) > MOTOR_EPSILON || abs(motors.distLeftW) > MOTOR_EPSILON_W 
		|| motors.status == MOTOR_CONST_VEL)
		return 0; // motor running
	else
	{
		if(motors.status == MOTOR_STOPPED || motors.status == MOTOR_RUNNING_STOP)
			MotorStop();
		else if(motors.status == MOTOR_FLOATING || motors.status == MOTOR_RUNNING_FLOAT)
			MotorFloat();
		// else motor MOTOR_ELSE
	}
	return 1; // motor stopped
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
	if(abs(motors.distLeftV) < motors.SbreakV) // decelerate
	{
		// we use distance to calc velocity to stop right where we want
		// and whe override previous de/acceleration block based on MOTOR_ACC_V and MOTOR_DRIVER_T
		if(motors.SbreakV != 0)
			motors.currentV = abs(motors.targetV) * motors.distLeftV/(motors.SbreakV);
		else
			motors.currentV = 0;
		
		if(motors.currentV > 0 && abs(motors.distLeftV) > MOTOR_EPSILON)
			motors.currentV += 20;
		else if(motors.currentV < 0 && abs(motors.distLeftV) > MOTOR_EPSILON)
			motors.currentV -= 20;
	}
	
	
	// == rotation velocity ==
	if(motors.currentW < motors.targetW) // accelerate
	{
		motors.currentW += MOTOR_ACC_W*MOTOR_DRIVER_T;
		if(motors.currentW > motors.targetW)
			motors.currentW = motors.targetW;
	}
	else if(motors.currentW > motors.targetW) // deceleration
	{
		motors.currentW -= MOTOR_ACC_W*MOTOR_DRIVER_T;
		if(motors.currentW < motors.targetW)
			motors.currentW = motors.targetW;
	}
	
//	//breaking
//	// w*w/MOTOR_ACC_W >= distleftW
//	if(((motors.currentW*motors.currentW)/MOTOR_ACC_W) >= motors.distLeftW){
//		
//	}
//	if(-motors.SbreakW < motors.distLeftW &&  motors.distLeftW < motors.SbreakW)
//	{
//		// we use distance to calc velocityW to stop right where we want
//		// and whe override previous de/acceleration block based on MOTOR_ACC_W and MOTOR_DRIVER_T
//		if(motors.SbreakW != 0)
//			motors.currentW = abs(motors.targetW) * motors.distLeftW/(motors.SbreakW);
//		else if(-MOTOR_EPSILON_W < motors.SbreakW && motors.SbreakV < MOTOR_EPSILON_W)
//			motors.currentW = 0;
//		else
//			motors.currentW = motors.targetW;
//	}
}

void MotorDriver()
{
	float rotationalFeedback = 0; // right+ left-
	float encoderVFeedback;
	float errorV, errorW;
	int PwmV, PwmW;
	int leftCh = motors.mot[0].encChange;
	int rightCh = motors.mot[1].encChange;
	
  // == PID ==
	// based on speed from 2 encoders
	encoderVFeedback = leftCh + rightCh; // [imp/T] use abs to measure wheel speed, or don't use abs (and change TurnA) and measure robot speed
	
	if(_motor_flag & FLAG_ENCODER)
	{
		if(leftCh != rightCh)
		{
			float alpha = 0.5f*(leftCh-rightCh)/(TICKS_PER_MM*HALF_WHEELBASE); // rad
			float encoderWFeedback = alpha*MOTOR_DRIVER_FREQ;  // rad / s
			rotationalFeedback += encoderWFeedback;
		}
	}
	
	if(_motor_flag & FLAG_GYRO)
	{
		rotationalFeedback += 0.8f*(-sensorGyroW)*PI*0.002778f; // /360
	}
	
//	if(_motor_flag & FLAG_SENSOR)
//	{ // 1100 LMiddleValue 920 RMiddleValue
//		float sensorFeedback = 0;//sensorError/a_scale;
//		if(SENS_RS < 60)
//				sensorFeedback = (float)(SENS_RS) - 30;
////		else if(sens[2] > 1100)
////			sensorFeedback = 1100 - sens[2];
////		else if(sens[3] > 920)
////			sensorFeedback = sens[3] - 920;
////		else
////			sensorFeedback = sens[2] - sens[3];
//		rotationalFeedback -= 1*sensorFeedback;
//	}

	
	motors.distLeftV -= encoderVFeedback; // imp
	motors.distLeftW -= rotationalFeedback*MOTOR_DRIVER_T;
	
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
	PwmW = (int)(MOTOR_VELW_KP*errorW + MOTOR_VELW_KI*motors.errWI + MOTOR_VELW_KD*motors.errWD);
	
	// == I limitation == 
	//if((PwmV < -MOTOR_MAX_PWM) || (MOTOR_MAX_PWM < PwmV))
	//	motors.errVI -= errorV;
	//if((PwmW < -MOTOR_MAX_PWM) || (MOTOR_MAX_PWM < PwmW))
	//	motors.errWI -= errorW;
	
	//printf("%d\t %d\t snes:%f\t feedW:%f\t rot:%f\r\n",SENS_L, SENS_R, sensorFeedback, rotationalFeedback, errorW);
	//printf("%d\t %d\r\n",SENS_L, SENS_R);
		
	motors.mot[0].PWM = PwmV + PwmW;
	motors.mot[1].PWM = PwmV - PwmW;
	
	//printf("tV:%f\t cV:%f\tDV:%d\tDW:%f\tcW:%f\ttW:%f\r\n", motors.targetV, motors.currentV, motors.distLeftV, motors.distLeftW, motors.currentW, motors.targetW);
}

void MotorUpdate()
{
	//while(1) {MotorUpdateEnc(); printf("L:%d\t R:%d\r\n", motors.mot[0].enc, motors.mot[1].enc); HAL_Delay(50); }
	//GyroReadData(); // angular velocity
	GyroGetAngle(0.001);
	MotorUpdateEnc();
	if(MotorUpdateStatus()) return;// return 1 when motor driver should be called
	MotorUpdateVelocity();
	MotorDriver();
	MotorSetPWM();
	//printf("tarV:%f errV:%d errR:%d PwmL:%d PwmR:%d ledtf:%d\r\n", motors.targetV, rotationalFeedback, motors.mot[0].PWM, motors.mot[1].PWM, motors.distLeftV);
	//printf("L:%d  R:%d\r\n", motors.mot[0].PWM, motors.mot[0].PWM);
	//static int lastDistLeft;
	//printf("%d\r\n", -(motors.distLeftV - lastDistLeft));
	//lastDistLeft = motors.distLeftV;
}

void MotorGoA(int left, int right, float vel) // [mm] [mm] [mm/s]
{	
	float w = 0;
	float dV = vel - motors.previousV; // [mm/s]
	float Sbreak = 2.0f*0.5f*vel*vel/MOTOR_ACC_V;// [mm] = mm/s * mm/s * s*s/mm		double S because we accumulate dX from 2 wheels
	float Tacc = dV/MOTOR_ACC_V; // [s]
	float Sacc = 2.0f*(motors.previousV*Tacc + 0.5f*dV*dV/MOTOR_ACC_V);// mm = mm/s * mm/s * s*s/mm		double S because we accumulate dX from 2 wheels
	
	// when vel is big and dist is short
	// then deccelerate has higher priority in
	if(Sacc + Sbreak > abs(left) + abs(right))
	{
		vel = fast_sqrt(0.5f*(MOTOR_ACC_V*(abs(left)+abs(right)) + 0.5f*motors.previousV*motors.previousV)); // mm/s/s*mm + mm*mm/s/s
		dV = vel - motors.previousV;
		Tacc = dV/MOTOR_ACC_V;
		Sacc = 2.0f*(0.5f*(motors.previousV*Tacc + dV*dV/MOTOR_ACC_V));
		Sbreak = 2.0f*0.5f*vel*vel/MOTOR_ACC_V;
	}
	
	// calc W speed
	if(left != right)
	{
		float a = 0.5f*(left-right)/HALF_WHEELBASE; // rad
		w = a * vel / (abs(left)+abs(right)); // [rad/s] = rad * [mm/s] / mm
	}
		
	motors.SbreakV = mmToTicks(Sbreak);// mm/s * mm/s * s*s/mm = mm
	motors.distLeftV = mmToTicks(left + right); // 2 wheel distance
	motors.targetW = w;
	
	if(motors.distLeftV > 0)
	{
		motors.previousV = vel;
		motors.targetV = 2*vel;// becouse we accumutade dX from 2 wheels
	}
	else
	{
		motors.previousV = - vel;
		motors.targetV = -2*vel;
	}
	
	
	MotorUpdateEnc(); // to delete EncChange
	
}

void MotorTurnA(int angle, int r, float vel)
{
	while(motors.mot[0].encChange < 1760){
		MotorSetPWMRaw(300, -300);
	}
	MotorStop();
	//int dist = r+HALF_WHEELBASE + r-HALF_WHEELBASE;
<<<<<<< HEAD
//	motors.distLeftV = 0;
//	motors.distLeftW = angle*PI*0.0056f; // /180
//	motors.targetV = 0;
//	motors.targetW = vel/(abs(r)+HALF_WHEELBASE);
//	motors.SbreakV = 0;//mmToTicks(2.0f*0.5f*vel*vel/MOTOR_ACC_V);
//	motors.SbreakW = 0;//0.5f*motors.targetW*motors.targetW/MOTOR_ACC_W;
}
=======
	motors.distLeftV = 0;
	motors.distLeftW = angle*PI*0.00555555555555555555555555555556f; // /180
	motors.targetV = 0;
	motors.targetW = vel/(abs(r)+HALF_WHEELBASE);
	motors.SbreakV = 0;//mmToTicks(2.0f*0.5f*vel*vel/MOTOR_ACC_V);
	motors.SbreakW = 0;//0.5f*motors.targetW*motors.targetW/MOTOR_ACC_W;
}*/

void MotorGo(int left, int right, float vel)
{ 
	MotorGoA(left, right, vel);
	while(!MotorUpdateStatus())
	{
		MotorUpdate();
		UI_DelayUs(990);
	}	
	MotorStop();
}



void Turn90C(){
	// ang_vel to ticks/ms
	//float ticks = WHEELBASE*vel*PI*ONE_CELL_DISTANCE/180/180/1000;
	UI_TimerUs(1e6f*MOTOR_DRIVER_T);
	if(TIM7->CNT < 180) // T1 + T2
		motors.targetW = MAX_ANGULAR_VEL;
	else if(TIM7->CNT > 180) // T3
		motors.targetW = 0;
}

void MotorTurn(int angle, int r, float vel)
{
	motors.targetV = vel;
	if(sensorGyroA < 30){
		MotorSetPWMRaw(300, -300);
	}
	else if(sensorGyroA > 30 && sensorGyroW < 60){
		MotorSetPWMRaw(400, -400);
	}
	else if(sensorGyroA > 60)
	{
		MotorSetPWMRaw(300, -300);
	}
	else if(sensorGyroA > angle)
		MotorStop();
//	motors.status = MOTOR_ELSE;
//	if(angle > 0)
//		MotorSetPWMRaw(300, -300);
//	else
//		MotorSetPWMRaw(-300, 300);
//	
//	UI_DelayUs(55000);
//	
//	if(angle > 0)
//		MotorSetPWMRaw(100, -100);
//	else
//		MotorSetPWMRaw(-100, 100);
//	
//	UI_DelayUs(60000);
//	MotorSetPWMRaw(0, 0);
//	UI_DelayUs(60000);
//	UI_DelayUs(60000);
//	MotorStop();
//	motors.status = MOTOR_STOPPED;
//	MotorTurnA(angle, r, vel);
//	while(motors.distLeftV > MOTOR_EPSILON)
//	{
//		MotorUpdate();
//		UI_DelayUs(990);
//	}
	
}

void MotorSetPWM()
{
	int VL = MotorTruncPWM(motors.mot[0].PWM)/3;
	int VR = MotorTruncPWM(motors.mot[1].PWM)/3;
	
	//if(VL < 0)
//		VL -= 30;
//	else if(VL > 0)
//		VL = 30;
	
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN1_Pin, VL>=0 ? GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, ML_IN2_Pin, VL>=0 ? GPIO_PIN_RESET  : GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN1_Pin, VR<=0 ? GPIO_PIN_SET 	  : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_GPIO, MR_IN2_Pin, VR<=0 ? GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_L, abs(VL));
	__HAL_TIM_SetCompare(&MOTOR_HTIM, MOTOR_CH_R, abs(VR));
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
	
	motors.status = MOTOR_STOPPED;
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
