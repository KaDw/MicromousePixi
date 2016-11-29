#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sensor.h"
#include "fxas21002c.h"
#include "motor.h"
#include "UI.h"


//uint32_t sens1[3];
//uint32_t sens2[3];
//uint32_t sens3[3];
//uint32_t sens4[3];
//uint32_t sens5[3];
//uint32_t sens6[3];

#define SENS_COUNT 6

uint8_t batcnt;
//_sensor sensor[6];
uint32_t cal[7];
uint32_t distance[6];
uint32_t sens[6];
uint32_t read[6];
uint32_t read_buf[2][SENS_COUNT];
uint32_t sens_buf[2][SENS_COUNT];
//uint32_t fuzzy[6] = {4, 2, 3, 4, 5, 6};
volatile uint32_t vbat;
uint8_t batError;
/*
sens[0] - LF - Left
sens[1] - RF - Right
sens[2] - L - Left Front
sens[3] - R - Right Front
sens[4] - LS - Left Side
sens[5] - RS - Right Side

	L		LF	RF		R
	\		|		|   /
	 \	|		|  /
		\	|		| /
LS----		 ---- RS

*/

/* 	
	@note WARNING 
		@arg1 DONT USE ADC_CHANNEL_XX, USE CHx INSTEAD
		*/


void LinLEDs(void)
{
	const float c[5] = {-3.89448230619322e-12,	2.39320612601111e-08,	-4.09318457411115e-05,	-0.0106170510758770,	115.341914847137};
	float x;
	float d;
	
	x = SENS_LS;
	d = 0;
	for(int i = 4; i >= 0; --i)
		d = d*x + c[i];
	distance[4] = d;
	
	x = SENS_RS;
	d = 0;
	for(int i = 4; i >= 0; --i)
		d = d*x + c[i];
	distance[5] = d;
}

void LinLEDleft(void)
{
	double x = SENS_LS; // aktualnie brana pod uwage dioda
	//distance[4] = ((-1.4161e-8*x + 8.2094e-5)*x - 0.16098)*x + 172.52; // ma zero gdzies w srodku skali :/
	distance[4] = (4210426483214.32	-194895074894.817/(x*x)) / (3469.50589670008 + 	102173560.704401*x) * 1.1364; //1.3

	//zlinearyzuj tez przod
	x = SENS_LF;
	distance[0] = (4210426483214.32	-194895074894.817/(x*x)) / (3469.50589670008 + 	102173560.704401*x);
}

void LinLEDright(void)
{	
	float x = SENS_RS; // aktualnie brana pod uwage dioda
	//distance[5] = (-439491.665036219/x/x + 9850.39739591126/x  + 46.2900271020770 -0.00738785453539922*x  ); // calkiem dobrze dziala, ale dla duzych odl sie malo zmienia
	distance[5] = (4210426483214.32	-194895074894.817/(x*x)) / (3469.50589670008 + 	102173560.704401*x) *  0.6250; // 0.7
}


int SensorFeedback(void)
{
	int l = distance[4];
	int r = distance[5];
	int offset;
	
	// todo:
	if(l > 60)
		offset = r > 60 ? 0 : r - (162-WHEELBASE)/2;
	else if(r > 60)
		offset = (162-WHEELBASE)/2 - l; // tutaj juz wiemy, ze l <= 60
	else
		offset = r - l;
	return offset;
}


void SensorCallback(void)
{
	static int count = 0;
	
	//static int COUNTER_MAX = MOTOR_DRIVER_T/0.00002;
	HAL_GPIO_WritePin(GPIOB, CS_A_Pin, GPIO_PIN_SET);
		switch(count){
		case 0: 
			ADCreadAmbient(); // read ambient light
			HAL_GPIO_WritePin(GPIOC, D_LF_Pin, GPIO_PIN_SET); 
			break;
		case 2: // case 3: // 40us 
			ADCreadChannel(CH2, &read[0]); // LF read
			HAL_GPIO_WritePin(GPIOC, D_LF_Pin, GPIO_PIN_RESET); 
			break;
		case 3: //120us case 6: // 100us 
			HAL_GPIO_WritePin(GPIOC, D_RF_Pin, GPIO_PIN_SET);
			break;
		case 5: //160us case 8: // 140us
			ADCreadChannel(CH11, &read[1]); // RF read
			HAL_GPIO_WritePin(GPIOC, D_RF_Pin, GPIO_PIN_RESET);
			break;
		case 7: // 240us case 11: // 200us 
			HAL_GPIO_WritePin(GPIOA, D_L_Pin, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOC, D_R_Pin, GPIO_PIN_SET); 
			break;
		case 8: // 280us case 13: // 240us 
			ADCread2Channel(CH13, CH12, &read[2]); // L, R read
			HAL_GPIO_WritePin(GPIOA, D_L_Pin, GPIO_PIN_RESET); 
			HAL_GPIO_WritePin(GPIOC, D_R_Pin, GPIO_PIN_RESET); 
			break;
		case 10: // 360us case 16: // 300us 
			HAL_GPIO_WritePin(GPIOC, D_LS_Pin, GPIO_PIN_SET); 
			HAL_GPIO_WritePin(GPIOC, D_RS_Pin, GPIO_PIN_SET);
			break;
		case 11: // case 18: // 340us
			ADCread2Channel(CH3, CH10, &read[4]); // LS, RS read
			HAL_GPIO_WritePin(GPIOC, D_LS_Pin, GPIO_PIN_RESET); // side sensors off
			HAL_GPIO_WritePin(GPIOC, D_RS_Pin, GPIO_PIN_RESET);
			break;
		case 12: 
			for(int i = 0; i < SENS_COUNT; ++i)
			{
				// sprawdz czy naswietlona sciana nie jest jasniejsza od nienaswietlonej
				if(read[i] < cal[i])
					read[i] = cal[i];
				
				//odejmij skladowa stala od sygnalu
				sens[i] = read[i] - cal[i];
				
				// oblicz licznik i mianownik do wyjscia
				// butter 100Hz/1000Hz, 2 stopnia
				float num = 0.00554271721028068 * read[i] - 177863177782459 * read_buf[0][i] + 0.800802646665708 * read_buf[1][i];
				float den = 0.00554271721028068 * sens[i] + 0.0110854344205614 * sens_buf[0][i] + 0.00554271721028068 * sens_buf[1][i];
				
				// sprawdz czy mozesz bezpiecznie dzielic
//				if(den != 0)
//					sens[i] = num/den;
//				else
					sens[i] = (sens_buf[1][i] + read[i] + 2*read_buf[0][i] + 4*read_buf[1][i]) * 0.125f;
				
				// zaktualizuj bufory
				read_buf[1][i]=read_buf[0][i]; read_buf[0][i]=read[i];
				sens_buf[1][i]=sens_buf[0][i]; sens_buf[1][i]=sens[i];
				
				// zlinearyzuj led'y
				LinLEDleft();
				LinLEDright();
				//LinLEDs();
			}
			
			break;
		case 13:
			GyroGetAngle(0.001);
			break;
		
		case 14:
			//MotorStepResponse(160, 150, 1500);
			MotorUpdate();
			break;
	}
			
		
	++count;
	
	if(count >= 25){
		//HAL_GPIO_WritePin(GPIOB, CS_A_Pin, 1);
		count = 0;
	}
	
	HAL_GPIO_WritePin(GPIOB, CS_A_Pin, GPIO_PIN_RESET);
}


void ADCreadAmbient(){
	ADC1->SQR1 = (ADC_SQR1_L_2|ADC_SQR1_L_1); // 7 conversions (0x06, count from 0)
	// channels order is important!
	ADC1->SQR3 = (CH2)|(CH11<<5)|(CH13<<10)|(CH12<<15)|(CH3<<20)|(CH10<<25); // channels to convert: 2, 11, 12, 13 and 9 for battery
	ADC1->SQR2 = CH9;
	//CH3, CH10
	HAL_ADC_Start_DMA(&hadc1, cal, 7);
	//vbat = cal[6]; // *0.0025 i dodac 0,067
	if(cal[6] < 3408 && cal[6] > 2650){ // 7,4-6,8
		vbat += cal[6];
		batcnt++;
		if(vbat){
			batcnt = 0;
			UI_LedOnAll();
			batError = 1;
		}
	}
}

void ADCreadChannel(uint8_t CHx, uint32_t *buf){
	/*
	ADC_ChannelConfTypeDef sConfig;
	
	sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	*/
	ADC1->SQR1 &= ~ADC_SQR1_L; // 1 conversion
	ADC1->SQR3 = CHx; // channel to be converted
	if(HAL_ADC_Start_DMA(&hadc1, buf, 1) != HAL_OK)
		UI_LedOnAll();
}

void ADCread2Channel(uint8_t CHx1, uint8_t CHx2, uint32_t *buf){
	ADC1->SQR1 = ADC_SQR1_L_0;
	ADC1->SQR3 = (CHx1)|(CHx2<<5);
	if(HAL_ADC_Start_DMA(&hadc1, buf, 2) != HAL_OK)
		UI_LedOnAll();
}


void SensorOff(){
	// set as input
	GPIOC->MODER &= ~(GPIO_MODER_MODER14_0|GPIO_MODER_MODER14_1|
										GPIO_MODER_MODER15_0|GPIO_MODER_MODER15_1|
										GPIO_MODER_MODER13_0|GPIO_MODER_MODER13_1|
										GPIO_MODER_MODER4_0|GPIO_MODER_MODER4_1|
										GPIO_MODER_MODER5_0|GPIO_MODER_MODER5_1);
	GPIOA->MODER &= ~(GPIO_MODER_MODER1_0|GPIO_MODER_MODER1_0);
	
	// pull down
	GPIOC->PUPDR &= ~54000500;
	GPIOC->PUPDR |= 0xA8000A00;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_1;
}

/* linearizing sensors*/
uint32_t LinADC(uint32_t *sens){
	float x = (float)(*sens);
	if(*sens < 3400)
		return (uint32_t)(((-1.14897e-8f*x + 8.18093e-5f)*x - 0.19299f)*x + 203.425f);
	else
		return (uint32_t)((-5.03451e-5f*x + 0.287246f)*x - 349.511f);
}
// To be continued    vvvvv
void ADCcalibrate(){
	
}

/*


*/
void UpdateCell(){
	//uint8_t cell_x = motors.mot[0].enc/ONE_CELL_DISTANCE;
	uint16_t pos = (EncL+EncR)/2;
	if(SENS_RS < HAS_RIGHT_WALL){
		
	}
	else if(SENS_LS < HAS_LEFT_WALL){
		
	}
	//else if(SENS_LF 
	
	
}

//void Move(_sensor *sensor){ // moving samples in buf
//	sensor->buf[0] = sensor->buf[1];
//	sensor->buf[1] = sensor->buf[2];
//}
///*Pepper noise filtering
//	Correct value will be stored in sensor.sens[1]
//	sens[3][6];
//	ADCread2channel(&hadc1, 2, &sens[3][0]);
//*/

//void Swap(uint32_t *s1, uint32_t *s2){
//	uint32_t temp = 0;
//	temp = (*s2);
//	(*s2) = (*s1);
//	(*s1) = temp;
//}

//uint32_t Sort(_sensor sensor){
//	// faster bubble sort
//	if (sensor.buf[0] > sensor.buf[1]) Swap(&sensor.buf[0], &sensor.buf[1]);
//	if (sensor.buf[1] > sensor.buf[2]) Swap(&sensor.buf[1], &sensor.buf[2]);
//	if (sensor.buf[0] > sensor.buf[1]) Swap(&sensor.buf[0], &sensor.buf[1]);
//	// bubble sort
////	for(uint8_t i = 0; i < 3; i++){ // size
////		for(uint8_t j = 0; j < 2; j++){ // size -1
////			if(sensor.buf[j] > sensor.buf[j+1]){
////				temp = sensor.buf[j+1];
////				sensor.buf[j+1] = sensor.buf[j];
////				sensor.buf[j] = temp;
////			}
////		}
////	}
//	
//	return sensor.buf[1]; // filtered value, sens[3] = {bad, good, bad};
//}
	

uint8_t FingerStart(){ // calibrate after finger start
while(sens[0] < 2000 && sens[2] < 2000) // LF and L sensor
	{}
		
	return 1;
}
//void readSensor()
//{
//	uint32_t curt;

// read without ir enabled
//	ADC_read_channel(ADC_CHANNEL_2, &LFSensor); 
//	ADC_read_channel(ADC_CHANNEL_11, &RFSensor); 
//	ADC_read_channel(ADC_CHANNEL_13, &LSensor); 
//	ADC_read_channel(ADC_CHANNEL_12, &RSensor); 

//	curt = micros();
//	
////left front sensor
//	LF_EM_ON;
//	elapseMicros(60,curt);
//	LFSensor = read_LF_Sensor - LFSensor;
//	LF_EM_OFF;
//	if(LFSensor < 0)//error check
//		LFSensor = 0;
// 	elapseMicros(140,curt);
////right front sensor	
//	RF_EM_ON;
//	elapseMicros(200,curt);	
//	RFSensor = read_RF_Sensor - RFSensor;
//	RF_EM_OFF;
//	if(RFSensor < 0)
//		RFSensor = 0;
// 	elapseMicros(280,curt);
////diagonal sensors
//	SIDE_EM_ON;
//	elapseMicros(340,curt);	
//	DLSensor = read_DL_Sensor - DLSensor;
//	DRSensor = read_DR_Sensor - DRSensor;
//  SIDE_EM_OFF;
//	if(DLSensor < 0)
//		DLSensor = 0;
//	if(DRSensor < 0)
//		DRSensor = 0;
//	
//	readVolMeter();
//	
//	LFSensor = LFSensor*reflectionRate/1000;
//	RFSensor = RFSensor*reflectionRate/1000;
//	DLSensor = DLSensor*reflectionRate/1000;
//	DRSensor = DRSensor*reflectionRate/1000;
//	
//	//delay_us(80);
//	//elapseMicros(500,curt);
//}
