#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sensor.h"


uint32_t cal[5];
uint32_t sens[4];
uint32_t vbat;

/* 	
	@note WARNING 
		@arg1 DONT USE ADC_CHANNEL_XX, USE CHx INSTEAD
		*/

void ADCreadAmbient(){
	ADC1->SQR1 = ADC_SQR1_L_2; // 5 conversions (0x04, count from 0)
	// TODO fix this hex
	//(CH2)|(CH11<<5)|(CH12<<10)|(CH13<<15)|(CH9<<20);
	ADC1->SQR3 = (CH2)|(CH11<<5)|(CH12<<10)|(CH13<<15)|(CH9<<20); // channels to convert: 2, 11, 12, 13 and 9 for battery
	HAL_ADC_Start_DMA(&hadc1, cal, 5);
	vbat = cal[4];
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


// To be continued    vvvvv
void ADCcalibrate(){
	
}

uint8_t FingerStart(){ // calibrate after finger start
while(sens[0] > 2000 && sens[2] > 2000) // LF and L sensor
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
