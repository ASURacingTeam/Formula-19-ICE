/*
 * mpu6050.c
 *
 *  Created on: Dec 1, 2018
 *      Author: mohned
 */

#include "mpu6050.h"

static float gyro_divider;
static float accel_divider;

void MPU6050_Init(I2C_HandleTypeDef *hi2c,
	              MPU6050_I2CAddress address,
				  MPU6050_GyroFS gyro_FS,
				  MPU6050_AccelFS accel_FS,
				  MPU6050_SamplingRate sampling_rate,
				  MPU6050_Readings* mpu)
{

	uint8_t data_buffer[2];
	/*wake up MPU from sleep mode*/
	data_buffer[0]=MPU6050_PWR_MGMT_1;
	data_buffer[1]=SLEEPMODE_OFF;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);
	HAL_Delay(1);

	data_buffer[0]=MPU6050_PWR_MGMT_2;
	data_buffer[1]=ACC_GYRO_ENABLE;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);
	HAL_Delay(1);

	//temp_scale = WriteReg(MPUREG_ACCEL_CONFIG | 0x80, 0x00);
	//HAL_Delay(1);

	data_buffer[0]=MPU6050_CONFIG;
	data_buffer[1]=DLPF_CFG_188HZ;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);
	HAL_Delay(1);

	/*set full scale of gyroscope*/
	data_buffer[0]= MPU6050_GYRO_CONFIG;
	data_buffer[1]=gyro_FS;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);
	HAL_Delay(1);

	/*set full scale of accelerometer*/
	data_buffer[0]= MPU6050_ACCEL_CONFIG;
	data_buffer[1]=accel_FS;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);
	HAL_Delay(1);

	/*select sampling rate*/
	data_buffer[0]=MPU6050_SMPLRT_DIV;
	data_buffer[1]=sampling_rate;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,2,1000);
	HAL_Delay(1);

	/*select multiplying factor that used to convert raw data to actual readings*/
	switch (gyro_FS)
		{
			case FULLSCALE_250:
				gyro_divider = MPU6050_GYRO_DIVIDER_250;
				break;
			case FULLSCALE_500:
				gyro_divider =  MPU6050_GYRO_DIVIDER_500;
				break;
			case FULLSCALE_1000:
				gyro_divider =  MPU6050_GYRO_DIVIDER_1000;
				break;
			case FULLSCALE_2000:
				gyro_divider =  MPU6050_GYRO_DIVIDER_2000;
				break;
			default:
				break;
		}

	/*select multiplying factor that used to convert raw data to actual readings*/
	switch (accel_FS)
		{
			case FULLSCALE_2g:
				accel_divider =  MPU6050_ACCE_DIVIDER_2;
				break;
			case FULLSCALE_4g:
				accel_divider =  MPU6050_ACCE_DIVIDER_4;
				break;
			case FULLSCALE_8g:
				accel_divider =  MPU6050_ACCE_DIVIDER_8;
				break;
			case FULLSCALE_16g:
				accel_divider =  MPU6050_ACCE_DIVIDER_16;
				break;
			default:
				break;
		}
}

void MPU6050_ReadData(I2C_HandleTypeDef *hi2c,
					  MPU6050_I2CAddress address,
					  MPU6050_Readings* mpu)
{
    uint8_t data_buffer[7];
	/*read raw data of gyroscope*/
	data_buffer[0]=MPU6050_GYRO_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,1,1000);
	//while(HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,1000)!=HAL_OK);
	HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,1000);
	int16_t raw_readings[3];
	raw_readings[0]=((int16_t)data_buffer[1]<<8)| data_buffer[2];
	raw_readings[1]=((int16_t)data_buffer[3]<<8)| data_buffer[4];
	raw_readings[2]=((int16_t)data_buffer[5]<<8)| data_buffer[6];
	mpu->gyroscope_x=(float)raw_readings[0]/gyro_divider;
	mpu->gyroscope_y=(float)raw_readings[1]/gyro_divider;
	mpu->gyroscope_z=(float)raw_readings[2]/gyro_divider;
	HAL_Delay(20);

	/*read data of accelerometer*/
	data_buffer[0]=MPU6050_ACCEL_XOUT_H;
	HAL_I2C_Master_Transmit(hi2c,(uint16_t)address,data_buffer,1,1000);
	HAL_I2C_Master_Receive(hi2c,(uint16_t)address,&data_buffer[1],6,1000);
	raw_readings[0]=(int16_t)(data_buffer[1]<<8)| data_buffer[2];
	raw_readings[1]=(int16_t)(data_buffer[3]<<8)| data_buffer[4];
	raw_readings[2]=(int16_t)(data_buffer[5]<<8)| data_buffer[6];
	mpu->accelerometer_x=(float)raw_readings[0]/accel_divider;
	mpu->accelerometer_y=(float)raw_readings[1]/accel_divider;
	mpu->accelerometer_z=(float)raw_readings[2]/accel_divider;
	HAL_Delay(20);

}


