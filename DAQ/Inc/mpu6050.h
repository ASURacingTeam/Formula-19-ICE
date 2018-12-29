/*
 * mpu6050.h
 *
 *  Created on: Nov 30, 2018
 *      Author: mohned
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "i2c.h"

/*----------------------------------Registers Map---------------------------*/
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6B


#define SLEEPMODE_OFF 0x00

typedef enum
{
	/*mpu6050 address is 110100x0*/

	MPU6050_ADDRESS_AD0LOW=0xD0,
	MPU6050_ADDRESS_AD0HIGH=0xD2,

}MPU6050_I2CAddress;

typedef enum
{
    /* due to DLPF_CFG bits is 0 in configuration register so gyroscope output rate is 8khz  */
	/*sample rate= gyroscope output rate / ( 1 + SMPLRT_DIV ) */

	/*SMPLRT_DIV*/
	SAMPLERATE_8KHz=0,
	SAMPLERATE_4KHz=1,
	SAMPLERATE_2KHz=3,
	SAMPLERATE_1KHz=7,
	SAMPLERATE_500Hz=15,
	SAMPLERATE_250Hz=31,
	SAMPLERATE_125Hz=63,
	SAMPLERATE_100Hz=79,

}MPU6050_SamplingRate;

typedef enum
{
	FULLSCALE_250=0x00, //full scale of gyroscope to 250 degree/s
	FULLSCALE_500=0x08,
	FULLSCALE_1000=0x10,
	FULLSCALE_2000=0x18,

}MPU6050_GyroFS;

typedef enum
{
	FULLSCALE_2g=0x00,  //full scale of accelerometer to +-2g
	FULLSCALE_4g=0x08,
	FULLSCALE_8g=0x10,
	FULLSCALE_16g=0x18,

}MPU6050_AccelFS;

typedef struct
{
	int16_t gyroscope_x;
	int16_t gyroscope_y;
	int16_t gyroscope_z;
	int16_t accelerometer_x;
	int16_t accelerometer_y;
	int16_t accelerometer_z;

}MPU6050_RawValues;



void MPU6050_Init(I2C_HandleTypeDef *hi2c,
	              MPU6050_I2CAddress address,
				  MPU6050_GyroFS gyro_FS,
				  MPU6050_AccelFS accel_FS,
				  MPU6050_SamplingRate sampling_rate);

void MPU6050_ReadRawData(I2C_HandleTypeDef *hi2c,
					  	 MPU6050_I2CAddress address,
						 MPU6050_RawValues* mpu);

#endif /* MPU6050_H_ */
