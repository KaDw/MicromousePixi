/* FXAS21002C 3-Axis Digital Angular Rate Gyroscope*/
#ifndef FXAS21002C_H_
#define FXAS21002C_H_

#include "stm32f4xx_hal.h"


typedef struct
{
   float x;
   float y;
   float z;
} angular_data;

typedef struct
{
   int16_t x;
   int16_t y;
   int16_t z;
} raw_data;

#define FXAS21002C_H_STATUS           0x00
#define FXAS21002C_H_DR_STATUS        0x07
#define FXAS21002C_H_F_STATUS         0x08
#define FXAS21002C_H_OUT_X_MSB        0x01    
#define FXAS21002C_H_OUT_X_LSB        0x02
#define FXAS21002C_H_OUT_Y_MSB        0x03
#define FXAS21002C_H_OUT_Y_LSB        0x04
#define FXAS21002C_H_OUT_Z_MSB        0x05
#define FXAS21002C_H_OUT_Z_LSB        0x06
#define FXAS21002C_H_F_SETUP          0x09
#define FXAS21002C_H_F_EVENT          0x0A
#define FXAS21002C_H_INT_SRC_FLAG     0x0B
#define FXAS21002C_H_WHO_AM_I         0x0C
#define FXAS21002C_H_CTRL_REG0        0x0D  
#define FXAS21002C_H_RT_CFG       	  0x0E
#define FXAS21002C_H_RT_SRC       	  0x0F 
#define FXAS21002C_H_RT_THS       	  0x10
#define FXAS21002C_H_RT_COUNT         0x11
#define FXAS21002C_H_TEMP             0x12
#define FXAS21002C_H_CTRL_REG1        0x13
#define FXAS21002C_H_CTRL_REG2        0x14
#define FXAS21002C_H_CTRL_REG3        0x15


enum gyroODR {
	GODR_800HZ, 
	GODR_400HZ,
	GODR_200HZ,
	GODR_100HZ,
	GODR_50HZ,
	GODR_12_5HZ, 
	GODR_6_25HZ,
	GODR_1_56HZ
};

// Set initial input parameters
enum gyroFSR {
  GFS_2000DPS,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS
};

enum gyroMODE {
	MODE_STANDBY,
	MODE_READY,
	MODE_ACTIVE
};


	 
extern float post_z;
//extern int16_t x, y, z;
//extern int16_t cal_x, cal_y, cal_z;
extern angular_data angle, x_pri, x_post, v_pri, v_post, alfa, beta;


void GyroInit(void);
uint8_t SpiRead(uint8_t address, uint8_t size);
void SpiWrite(uint8_t address,uint8_t value);
void GyroReadData(void);
void GyroCalibrate(void);
void AlfaBetaFilter(void);
float GetAngle(float dt, float a, float b);


#endif
