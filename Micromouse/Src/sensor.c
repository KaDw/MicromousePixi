#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "sensor.h"

uint32_t LFSensor;
uint32_t RFSensor;
uint32_t LSensor;
uint32_t RSensor;

uint32_t cal_LFSensor;
uint32_t cal_RFSensor;
uint32_t cal_LSensor;
uint32_t cal_RSensor;

void ADC_read_channel(uint32_t channel, uint32_t *buf){
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Offset = 0;
	sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start_DMA(&hadc1, buf, 1);
}

void ADC_read_ambient(){
	ADC_read_channel(ADC_CHANNEL_2, &cal_LFSensor); 
	ADC_read_channel(ADC_CHANNEL_11, &cal_RFSensor); 
	ADC_read_channel(ADC_CHANNEL_12, &cal_RSensor); 
	ADC_read_channel(ADC_CHANNEL_13, &cal_LSensor); 
}

// To be continued    vvvvv
void Calibrate(){
	
}

uint8_t Finger_start(){ // calibrate after finger start
while(LSensor > 5000 && LFSensor > 5000)
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
