//call speedProfile(void) in systick handle to make it execute every one milli second
//this controller sample code doesn't include 

//here are the code you need to call periodically to make the mouse sample and excute and a constant rate
//in order to make sample and pwm updating at a constant rate, you'd better call the code in the handler of a timer periodically
//the period for intterupt handler is usually 1ms, which means you have to finish all code excuting within 1ms
//I simply call the controller code in systick handler that intterupts every 1ms, notice systick timer is also used to
//keep track the system time in millisecond
/*
//test the timing make sure everything finishes within 1ms and leave at least 30% extra time for other code to excute in main routing
void systickHandler(void)
{
	Millis++;  //keep track of the system time in millisecond
	if(bUseIRSensor)
		readIRSensor();
	if(bUseGyro)
		readGyro();
	if(bUseSpeedProfile)
		speedProfile
}
*/

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "sensor.h"
#include "controller.h"

//ps. if you want to change speed, you simply change the targetSpeedX and targetSpeedW in main routing(int main) at any time.

float curSpeedX = 0;
float curSpeedW = 0;
int targetSpeedX = 1000;
int targetSpeedW = 1000;
int encoderFeedbackX = 0;
int encoderFeedbackW = 0;
float pidInputX = 0;
float pidInputW = 0;
float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
int posPwmX = 0;
int posPwmW = 0;
float kpX = 2, kdX = 4;
float kpW = 1, kdW = 12;//used in straight
float kpW1 = 1;//used for T1 and T3 in curve turn
float kdW1 = 26;
float kpW2 = 1;//used for T2 in curve turn
float kdW2 = 36;
float accX = 600;//6m/s/s  
float decX = 600; 
float accW = 1; //cm/s^2
float decW = 1;
int moveSpeed = 15141; // 15141  
int turnSpeed = 15141; // 15141
int returnSpeed = 15141; // 15141
int stopSpeed = 3028; // 3028
int maxSpeed = 60565; // 60565 

int oldEncoderCount;
int leftEncoder;
int rightEncoder;
int leftEncoderChange;
int rightEncoderChange;
int leftEncoderOld;
int rightEncoderOld;
int encoderChange;
int leftEncoderCount;
int rightEncoderCount;
int encoderCount;
int distanceLeft;
int leftBaseSpeed;
int rightBaseSpeed;
	
uint8_t useSensor;
uint8_t useGyro;
uint8_t usePID;
uint8_t onlyUseEncoderFeedback;
uint8_t onlyUseGyroFeedback;

int sensorError;
int a_scale = 1;
int aSpeed;

uint32_t DRMiddleValue = 1000; // side sensor value
uint32_t DLMiddleValue = 1000;
int gyroFeedbackRatio = 5700;//5900;

int oneCellDistance = 1514; // ticks, ~100mm


uint32_t LFvalue1 = 1000;
uint32_t RFvalue1 = 1000;
uint32_t LFvalue2 = 1000; // front sensor threshold
uint32_t RFvalue2 = 1000; // 



int speed_to_counts(int mm)
{
	return mm*1760/(3.14*37);
}

int counts_to_speed(int ticks)
{
	return (ticks/1760)*(3.14*37);
}

int abs(int x)
{
	if(x < 0)
		return -x;
	else
		return x;
}

void getEncoderStatus()
{
	leftEncoder = htim3.Instance->CNT;//read current encoder ticks from register of 32 bit general purpose timer 2
	rightEncoder = htim4.Instance->CNT;//read current encoder ticks from register of 32 bit general purpose timer 5

	leftEncoderChange = leftEncoder - leftEncoderOld; // rozniczkojemy droge lewego kola zeby dostac jego predkosc
	rightEncoderChange = rightEncoder - rightEncoderOld; 
	encoderChange = (leftEncoderChange + rightEncoderChange)/2; 

	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;
					
	leftEncoderCount += leftEncoderChange; // calkujemy zeby znowu dostac droge?
	rightEncoderCount += rightEncoderChange;
	encoderCount = (leftEncoderCount+rightEncoderCount)/2;	// i mamy zmiane 
	
	distanceLeft -= encoderChange;// update distanceLeft	
}



void updateCurrentSpeed(void)
{
	if(curSpeedX < targetSpeedX)
	{
		curSpeedX += (float)(speed_to_counts(accX*2)/100);
		if(curSpeedX > targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	else if(curSpeedX > targetSpeedX)
	{
		curSpeedX -= (float)speed_to_counts(decX*2)/100;
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	if(curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}
	else if(curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}	
}


void calculateMotorPwm(void) // encoder PD controller
{	
	int gyroFeedback;
	int rotationalFeedback;
	int sensorFeedback;
	
    /* simple PD loop to generate base speed for both motors */	
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;	
	
	gyroFeedback = aSpeed/gyroFeedbackRatio; //gyroFeedbackRatio mentioned in curve turn lecture	
	sensorFeedback = sensorError/a_scale;//have sensor error properly scale to fit the system
	
	if(onlyUseGyroFeedback)
		rotationalFeedback = gyroFeedback;
	else if(onlyUseEncoderFeedback)
		rotationalFeedback = encoderFeedbackW;
	else
		rotationalFeedback = encoderFeedbackW + gyroFeedback;
	    //if you use IR sensor as well, the line above will be rotationalFeedback = encoderFeedbackW + gyroFeedback + sensorFeedback;
	    //make sure to check the sign of sensor error.

	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW += curSpeedW - rotationalFeedback;
	
	posPwmX = kpX * posErrorX + kdX * (posErrorX - oldPosErrorX);
	posPwmW = kpW * posErrorW + kdW * (posErrorW - oldPosErrorW);
	
	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;
	
	leftBaseSpeed = posPwmX - posPwmW;
	rightBaseSpeed = posPwmX + posPwmW;

	
	HAL_GPIO_WritePin(GPIOC, ML_IN1_Pin, leftBaseSpeed>=0 ? GPIO_PIN_SET		: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, ML_IN2_Pin, leftBaseSpeed>=0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, MR_IN1_Pin, rightBaseSpeed<=0 ? GPIO_PIN_SET 	: GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, MR_IN2_Pin, rightBaseSpeed<=0 ? GPIO_PIN_RESET	: GPIO_PIN_SET);

	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, leftBaseSpeed);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, rightBaseSpeed);
}


int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd)//speed are in encoder counts/ms, dist is in encoder counts 
{
	if (curSpd<0) curSpd = -curSpd;
	if (endSpd<0) endSpd = -endSpd;
	if (dist<0) dist = 1;//-dist;
	if (dist == 0) dist = 1;  //prevent divide by 0

	return (abs(counts_to_speed((curSpd*curSpd - endSpd*endSpd)*100*(double)dist/4/2))); //dist_counts_to_mm(dist)/2);
	//calculate deceleration rate needed with input distance, input current speed and input targetspeed to determind if the deceleration is needed
	//use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/2/S
	//because the speed is the sum of left and right wheels(which means it's doubled), that's why there is a "/4" in equation since the square of 2 is 4
}

void resetSpeedProfile(void)
{
	//resetEverything;
	
	//disable sensor data collecting functions running in 1ms interrupt
 	useSensor = 0;
 	useGyro = 0;
	//no PID calculating, no motor lock
	usePID = 0;	

	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0);
	
	pidInputX = 0;
	pidInputW = 0;
	curSpeedX = 0;
	curSpeedW = 0;
	targetSpeedX = 0;
	targetSpeedW = 0;
	posErrorX = 0;
	posErrorW = 0;
	oldPosErrorX = 0;
	oldPosErrorW = 0;
    leftEncoderOld = 0;
	rightEncoderOld = 0;	
	leftEncoder = 0;
	rightEncoder = 0;
	leftEncoderCount = 0;
	rightEncoderCount = 0;
	encoderCount = 0;	
	oldEncoderCount = 0;
	leftBaseSpeed = 0;
	rightBaseSpeed = 0;

	TIM2->CNT = 0;//reset left encoder count
	TIM5->CNT = 0;//reset right encoder count
}

void getSensorEror(void)//the very basic case
{
//	if(sens[2] > DLMiddleValue && sens[3] < DRMiddleValue)
//		sensorError = DLMiddleValue - sens[2];
//	else if(sens[3] > DRMiddleValue && sens[3] < DLMiddleValue)
//		sensorError = sens[3] - DRMiddleValue;
//	else
		sensorError = 0;
}


/*
sample code for straight movement
*/
void moveOneCell()
{
//	enable_sensor(),
//	enable_gyro();
//	enable_PID();
	
	targetSpeedW = 0;
	targetSpeedX = moveSpeed;
	
	do
	{
		/*you can call int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd) 
		here with current speed and distanceLeft to decide if you should start to decelerate or not.*/
		/*sample*/
		if(needToDecelerate(distanceLeft, curSpeedX, moveSpeed) < decX)
			targetSpeedX = maxSpeed;
		else
			targetSpeedX = moveSpeed;	
		
		//there is something else you can add here. Such as detecting falling edge of post to correct longitudinal position of mouse when running in a straight path
	}
	while((encoderCount-oldEncoderCount) < oneCellDistance);
//	while( ( (encoderCount-oldEncoderCount) < oneCellDistance && sens[0] < LFvalue2 && sens[1] < RFvalue2)//use encoder to finish 180mm movement if no front walls	
//	|| (sens[0] < LFvalue1 && sens[0] > LFvalue2)//if has front wall, make the mouse finish this 180mm with sensor threshold only
//	|| (sens[1] < RFvalue1 && sens[1] > RFvalue2)
//	);
	//LFvalues1 and RFvalues1 are the front wall sensor threshold when the center of mouse between the boundary of the cells.
	//LFvalues2 and RFvalues2 are the front wall sensor threshold when the center of the mouse staying half cell farther than LFvalues1 and 2
	//and LF/RFvalues2 are usually the threshold to determine if there is a front wall or not. You should probably move this 10mm closer to front wall when collecting
	//these thresholds just in case the readings are too weak.
	
	oldEncoderCount = encoderCount;	//update here for next movement to minimized the counts loss between cells.
}

void speedProfile(void)
{	
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();      
}

//void PID(void)       
//{    
//    if((leftSensor > hasLeftWall && rightSensor > hasRightWall))//has both walls
//    {  //ccw direction is positive
//        errorP = rightSensor – leftSensor – 63;//63 is the offset between left and right sensor when mouse in the middle of cell
//        errorD = errorP – oldErrorP;
//    }        
//    else if((leftSensor > hasLeftWall))//only has left wall
//    {
//        errorP = 2 * (leftMiddleValue – leftSensor);
//        errorD = errorP – oldErrorP;
//    }
//    else if((rightSensor > hasRightWall))//only has right wall
//    {
//        errorP = 2 * (rightSensor – rightMiddleValue);
//        errorD = errorP – oldErrorP;
//    }
//    else if((leftSensor < hasLeftWall && rightSensor <hasRightWall))//no wall, use encoder or gyro
//    {
//        errorP = 0;//(leftEncoder – rightEncoder*1005/1000)*3;
//        errorD = 0;
//    }
//    totalError = P * errorP + D * errorD;
//    oldErrorP = errorP;
//    setLeftPwm(leftBaseSpeed – totalError);
//    setRightPwm(rightBaseSpeed + totalError);    
//}


//If(leftSensor > leftMiddleValue && rightSensor < rightMiddleValue)//case 1

//    errorP = leftMiddleValue – leftSensor;

//Else if(rightSensor > rightMiddleValue && leftSensor < leftMiddleValue)//case 2

//    errorP = rightSensor – rightMiddleValue;

//Else //case 3 do nothing, let encoder to align the mouse

//    errorP = 0;

//errorD = errorP – oldErrorP;

//oldErrorP = errorP;

//totalError = kp*errorP + kd*errorD;

//setLeftPwm(leftBaseSpeed – totalError);

//setRightPwm(rightBaseSpeed + totalError);






