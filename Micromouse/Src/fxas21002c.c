#include "spi.h"
#include "fxas21002c.h"
#include "gpio.h"

#define DEBUG_MODE
// SPI buffers
uint8_t abc;
uint8_t SpiRxBuffer[6];
//int16_t x, y, z;
raw_data raw;
float angle_new;
float R0;
float post_z;
int16_t cal_x, cal_y, cal_z;
angular_data angle ,x_pri, x_post, v_pri, v_post, alfa, beta;


uint8_t SpiRead(uint8_t address, uint8_t size){
	uint8_t a = 0xff;
	/* Read mode mask: 0b1000000*/
	address |= 0x80;
	HAL_GPIO_WritePin(GPIOA, CS_A_Pin, 0); // CS LOW
	/* Setup time for SPI_CS_B signal 250ns*/
		
	HAL_SPI_TransmitReceive_IT(&hspi3, &address, SpiRxBuffer, 1);
  while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
	while ((SPI3->SR & SPI_FLAG_BSY));
	// for multi-read mode
	for(int i = 0; i < size; i++){
		HAL_SPI_TransmitReceive_IT(&hspi3, &a, &SpiRxBuffer[i], 1);
	
	  while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
		while ((SPI3->SR & SPI_FLAG_BSY)); // shouldnt be here
	}
	while ((SPI3->SR & SPI_FLAG_BSY)); // check BUSY flag
	HAL_GPIO_WritePin(GPIOA, CS_A_Pin, 1); // CS HIGH
	return *SpiRxBuffer;
}


void SpiWrite(uint8_t address, uint8_t value){
	/* Write: MSB is 0*/

	HAL_GPIO_WritePin(GPIOA, CS_A_Pin, 0); // CS LOW
	/* Setup time for SPI_CS_B signal 250ns*/
	// MSB is 0 + address
	HAL_SPI_TransmitReceive_IT(&hspi3, &address, SpiRxBuffer, 1);

  while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
	while ((SPI3->SR & SPI_FLAG_BSY));
	HAL_SPI_TransmitReceive_IT(&hspi3, &value, SpiRxBuffer, 1);
	
	while (!(SPI3->SR & SPI_FLAG_TXE)); // check if transmit buffer has been shifted out
  while ((SPI3->SR & SPI_FLAG_BSY)); // check BUSY flag

	HAL_GPIO_WritePin(GPIOA, CS_A_Pin, 1); // CS HIGH 
}



void GyroInit(void){
	/* Gyro is now in Standby mode every register 
			changes should be made in this mode
	*/
	// check connection by reading whoami register, it should always be 0xD7
	if(SpiRead(FXAS21002C_H_WHO_AM_I, 1) != 0xD7){
		#ifdef DEBUG_MODE
		abc = 1;
		//printf("Unable to communicate with gyro\r\n");
		#endif
		return;
	}
//		I2C_WriteRegister(FXAS21002C_I2C_ADDRESS, GYRO_CTRL_REG1, 0x40);     // Reset all registers to POR values
//     Pause(0x631);        // ~1ms delay
//     
//     I2C_WriteRegister(FXAS21002C_I2C_ADDRESS, GYRO_CTRL_REG0, 0x03);     // High-pass filter disabled, +/-250 dps range -> 7.8125 mdps/LSB = 128 LSB/dps
//     I2C_WriteRegister(FXAS21002C_I2C_ADDRESS, GYRO_CTRL_REG2, 0x0C);     // Enable DRDY interrupt, routed to INT1 - PTA5, push-pull, active low interrupt
//     I2C_WriteRegister(FXAS21002C_I2C_ADDRESS, GYRO_CTRL_REG1, 0x16);     // ODR = 25Hz, Active mode       

		
		//SpiWrite(FXAS21002C_H_CTRL_REG0, (GFS_250DPS | 0x3)); // set Full-scale range, enable high pass filter
		SpiWrite(FXAS21002C_H_CTRL_REG1, 0x40); // Reset all registers to POR values
		HAL_Delay(1);
		SpiWrite(FXAS21002C_H_CTRL_REG0, 0x03); // High-pass filter disabled, +/-250 dps range -> 7.8125 mdps/LSB = 128 LSB/dps
		SpiWrite(FXAS21002C_H_CTRL_REG2, 0x0C); // Enable DRDY interrupt, routed to INT1 - PTA5, push-pull, active low interrupt
		SpiWrite(FXAS21002C_H_CTRL_REG3, GODR_800HZ); // ODR = 800Hz, Active mode 
		//SpiWrite(FXAS21002C_H_CTRL_REG1, GODR_200HZ); // set Output Data Rate 
		//SpiWrite(FXAS21002C_H_CTRL_REG2, 0x0E);
	
		SpiWrite(FXAS21002C_H_CTRL_REG1, MODE_ACTIVE); // enable gyro
		#ifdef DEBUG_MODE
		//printf("Ready\r\n");
		#endif 
}

void GyroReadData(void){
		//uint8_t Data[6];
		post_z = angle.z;
	//for(int i =0; i < 4; i++){
		SpiRead(FXAS21002C_H_OUT_X_MSB, 6);
		raw.x = (int16_t)(SpiRxBuffer[0] << 8 | SpiRxBuffer[1]) >> 2;
		raw.y = (int16_t)(SpiRxBuffer[2] << 8 | SpiRxBuffer[3]) >> 2;
		raw.z = (int16_t)(SpiRxBuffer[4] << 8 | SpiRxBuffer[5]) >> 2;
		// zero te gyro and get angular velocity 15.625
		angle.z = ((float)(raw.z-cal_z)*0.0078125); //31.25
	//}
}

void GyroCalibrate(void){
	for(int i = 0; i < 100; i++){
		GyroReadData();
		cal_x += raw.x;
		cal_y += raw.y;
		cal_z += raw.z;
		HAL_Delay(10);
	}
	cal_x = cal_x/100;
	cal_y = cal_y/100;
	cal_z = cal_z/100;
}

float GetAngle(float dt, float a, float b){
	float angle2;
	angle2 = (((a+b)/2.0)*dt);
	angle_new +=angle2;
	//gyroRate = (a - 1) * 7.8125;
	//angle += gyroRate / 100.0;
	// sensitivity 7.8125
	return angle_new;
}






void AlfaBetaFilter(void){
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
}