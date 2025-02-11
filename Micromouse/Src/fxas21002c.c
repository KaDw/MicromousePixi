#include "spi.h"
#include "fxas21002c.h"
#include "gpio.h"
#include "UI.h"

/*
TODO:
get dt using timer
*/


// SPI buffers
uint8_t SpiRxBuffer[2];
volatile uint16_t dt;
//int16_t x, y, z;
//raw_data raw;
int16_t raw_z;
int16_t cal_z;
int16_t old_cal_z;
float angle;
float prev_z;
float sensorGyroW;
float sensorGyroA;
float a1; // current angle
//angular_data angle ,x_pri, x_post, v_pri, v_post, alfa, beta;
// get time between readings
///* Captured Values */
//uint32_t uwIC2Value1;
//uint32_t uwIC2Value2;
///* Capture index */
//uint16_t uhCaptureIndex;




uint8_t SpiRead(uint8_t address, uint8_t size){
	uint8_t a = 0xff;
	/* Read mode mask: 0b1000000*/
	address |= 0x80;
	HAL_GPIO_WritePin(GPIOB, CS_G_Pin, GPIO_PIN_RESET); // CS LOW
	/* Setup time for SPI_CS_B signal 250ns*/
	HAL_SPI_TransmitReceive_DMA(&hspi3, &address, SpiRxBuffer, 1);
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY); // fancy HAL way
	// this is faster
	//  while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
	//	while ((SPI3->SR & SPI_FLAG_BSY));
	// for multi-read mode
	
		/* before entering HAL_SPI_Transmit we must esnure that SPI is ready 
		 (TXE buffer empty, BSY flag). Otherwise HAL will return error and the transmission never happens */
	for(int i = 0; i < size; i++){
		HAL_SPI_TransmitReceive_DMA(&hspi3, &a, &SpiRxBuffer[i], 1);
	  while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY); // fancy HAL way
		// this is faster
	//	  while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
	//		while ((SPI3->SR & SPI_FLAG_BSY)); // shouldnt be here
	}
	HAL_GPIO_WritePin(GPIOB, CS_G_Pin, GPIO_PIN_SET); // CS HIGH
	return *SpiRxBuffer;
}


void SpiWrite(uint8_t address, uint8_t value){
	/* Setup time for SPI_CS_B signal 250ns*/
	HAL_GPIO_WritePin(GPIOB, CS_G_Pin, GPIO_PIN_RESET); // CS LOW
	/* Write mode mask: 0b0000000 */
	HAL_SPI_TransmitReceive_DMA(&hspi3, &address, SpiRxBuffer, 1);
  //while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
	//while ((SPI3->SR & SPI_FLAG_BSY));
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY); 
	HAL_SPI_TransmitReceive_DMA(&hspi3, &value, SpiRxBuffer, 1);
	while (HAL_SPI_GetState(&hspi3) != HAL_SPI_STATE_READY); 
	//while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
  //while ((SPI3->SR & SPI_FLAG_BSY)); // check BUSY flag
	HAL_GPIO_WritePin(GPIOB, CS_G_Pin, GPIO_PIN_SET); // CS HIGH
}



void GyroInit(void){
	/* Gyro is now in Standby mode every register 
			changes should be made in this mode */
	
	// this shouldnt be here, HAL messed up
	SpiRead(FXAS21002C_H_WHO_AM_I, 1); //  dummy read for clk synchronization
	
	// check connection by reading whoami register, it should always be 0xD7
	if(SpiRead(FXAS21002C_H_WHO_AM_I, 1) != 0xD7){
		return;
	}
  
		//SpiWrite(FXAS21002C_H_CTRL_REG0, 0x03); // High-pass filter disabled, +/-250 dps range -> 7.8125 mdps/LSB = 128 LSB/dps
		//SpiWrite(FXAS21002C_H_CTRL_REG2, 0x0C); // Enable DRDY interrupt, routed to INT1 - PTA5, push-pull, active low interrupt
		//SpiWrite(FXAS21002C_H_CTRL_REG1, GODR_200HZ); // set Output Data Rate 
		//SpiWrite(FXAS21002C_H_CTRL_REG2, 0x0E);
		//SpiWrite(FXAS21002C_H_CTRL_REG0, (GFS_250DPS | 0x3)); // set Full-scale range, enable high pass filter
		//SpiWrite(FXAS21002C_H_CTRL_REG1, 0x40); // Reset all registers to POR values
		//SpiRead(FXAS21002C_H_CTRL_REG1, 1);
		UI_DelayUs(1);
		//SpiWrite(FXAS21002C_H_CTRL_REG2, (1<<3)|(1<<4)); // interrupt active low, enable, INT1
	  SpiWrite(FXAS21002C_H_CTRL_REG0, (GFS_250DPS)); // set FSR  
		SpiWrite(FXAS21002C_H_CTRL_REG1, (GODR_800HZ |  MODE_ACTIVE));  // ODR = 800Hz, switch to active mode  
		UI_DelayUs(50);
}

void GyroReadData(void){
		prev_z = raw_z;
		SpiRead(FXAS21002C_H_OUT_Z_MSB, 2);
		raw_z = -((int16_t)(SpiRxBuffer[0] << 8 | SpiRxBuffer[1]));
		sensorGyroW = (float)(raw_z - cal_z);
}
/* Take 100 samples and average offset value*/
void GyroCalibrate(float dt, uint16_t samples){
	// for user comfort
	dt*=1000; // get miliseconds

	old_cal_z = cal_z;  
	for(int i = 0; i < samples; i++){
		GyroReadData();
		cal_z += raw_z;
		UI_DelayUs((uint16_t)dt);
	}
	cal_z = cal_z/samples;
	old_cal_z = cal_z;
}

/* Trapezoidal integration */

float GyroIntegrate(float dt){
	a1 = ((((float)(prev_z-cal_z)+(raw_z-cal_z))*0.5f)*dt*0.0078125f) + a1;
	return a1;
}

/* Call this function to get angle 
	This function takes 33us to complete */
float GyroGetAngle(float dt){
	GyroReadData();
	sensorGyroA = GyroIntegrate(dt); 
	return sensorGyroA;
	
}

/* not tested yet */
void GyroResetDrif(){
	GyroCalibrate(1, 10);
	cal_z+=old_cal_z;
}




/* Non-essentail functions */

//void GyroSelfTest(){
//	for(uint8_t i = 0; i < 50; i++)
//		GyroReadData();
//	
//	SpiRead(FXAS21002C_H_CTRL_REG1, 1);
//	SpiWrite(FXAS21002C_H_CTRL_REG1, SpiRxBuffer[0]|(1<<5));
//	SpiRead(FXAS21002C_H_CTRL_REG1, 1);
//	
//	for(uint8_t i = 0; i < 50; i++){
//		GyroReadData();
//		
//		if(!(raw.x && raw.y && raw.z > 7000) && (raw.x && raw.y && raw.z < 25000)){
//			printf("Self-Test failed. Reading is out of 7000-25000 range");
//			return;
//		}
//	}
//}

//void GyroSoftReset(){
//	
//	SpiWrite(FXAS21002C_H_CTRL_REG1, 0x40); // Reset all registers to POR values
//}

//void AlfaBetaFilter(void){
//	float dt;
//	/* Parametry pracy filtru */
//	dt = 0.01;

//	alfa.x = 0.5;
//	alfa.y = 0.5;
//	alfa.z = 0.5;

//	beta.x = 0.05;
//	beta.y = 0.05;
//	beta.z = 0.05;
//	
//	x_pri.x = x_post.x + dt * v_post.x;
//	x_pri.y = x_post.y + dt * v_post.y;
//	x_pri.z = x_post.z + dt * v_post.z;
//	v_pri.x = v_post.x;
//	v_pri.y = v_post.y;
//	v_pri.z = v_post.z;

//	x_post.x = x_pri.x + alfa.x * (x - x_pri.x);
//	x_post.y = x_pri.y + alfa.y * (y - x_pri.y);
//	x_post.z = x_pri.z + alfa.z * (z - x_pri.z);
//	v_post.x = v_pri.x + beta.x * (x - x_pri.x) / dt;
//	v_post.y = v_pri.y + beta.y * (y - x_pri.y) / dt;
//	v_post.z = v_pri.z + beta.z * (z - x_pri.z) / dt;
//}
