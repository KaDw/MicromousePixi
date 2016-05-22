#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sensor.h"


uint32_t cal[7]; 
uint32_t sens[6];
uint32_t read[2];
uint32_t fuzzy[6];
uint32_t vbat;
uint8_t batError;
#define SENS_LF sens[0]
/*
sens[0] - LF - Left
sens[1] - RF - Right
sens[3] - L - Left Front
sens[4] - R - Right Front
sens[5] - LS - Left Side
sens[6] - RS - Right Side

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

void ADCreadAmbient(){
	ADC1->SQR1 = (ADC_SQR1_L_2|ADC_SQR1_L_1); // 7 conversions (0x06, count from 0)
	// TODO fix this hex
	//(CH2)|(CH11<<5)|(CH12<<10)|(CH13<<15)|(CH9<<20);
	ADC1->SQR3 = (CH2)|(CH11<<5)|(CH12<<10)|(CH13<<15)|(CH3<<20)|(CH10<<25); // channels to convert: 2, 11, 12, 13 and 9 for battery
	ADC1->SQR2 = CH9;
	//CH3, CH10
	HAL_ADC_Start_DMA(&hadc1, cal, 7);
	vbat = (cal[6]*0.0025)+0.067; // *0.0025 i dodac 0,067
	if(cal[6] < 2884 && cal[6] > 2650) // 7,4-6,8
		batError = 1;
}

void ADCreadChannel(uint8_t CHx, uint32_t *buf){
	ADC1->SQR1 &= ~ADC_SQR1_L; // 1 conversion
	ADC1->SQR3 = CHx; // channel to be converted
	HAL_ADC_Start_DMA(&hadc1, buf, 1);
}

void ADCread2Channel(uint8_t CHx1, uint8_t CHx2, uint32_t *buf){
	ADC1->SQR1 = ADC_SQR1_L_0;
	ADC1->SQR3 = (CHx1)|(CHx2<<5);
	HAL_ADC_Start_DMA(&hadc1, buf, 2);
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
