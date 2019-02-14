/*
 * wheel_speed.c
 *
 *  Created on: Dec 29, 2018
 *      Author: mohned
 */

#include "wheel_speed.h"

//private variables
float g_wheelspeed[5] = {0,0,0,0,0};                /*SPEED OF THE WHEEL*/
float g_rotation_duration[5] = {0,0,0,0,0};         /* 1/4 of  ROTATION DURATION OF THE WHEEL*/

// functions definitions
float Wheel_speed(uint8_t channel_number)
{
	if (channel_number == 1)
	{
		// 1/4 duration of one rotation
		g_rotation_duration[0] = (float) ((g_capture_val[0] - g_old_capture_val[0])
										 +(updateCounter[0] * Max_TIMER_COUNT))
										 / F_ICU;
		// speed in km/h
		g_wheelspeed[0] = ((((60 / (4 * g_rotation_duration[0])) * (2 * PI / 60)) * WHEEL_RADIUS) * (18 / 5));
		return g_wheelspeed[0];
	}
	else if (channel_number == 2)
	{
		g_rotation_duration[1] = (float) ((g_capture_val[1] - g_old_capture_val[1])
				                         +(updateCounter[1] * Max_TIMER_COUNT))
				                         / F_ICU;

		g_wheelspeed[1] = ((((60 / (4 * g_rotation_duration[1])) * (2 * PI / 60)) * WHEEL_RADIUS) * (18 / 5));
		return g_wheelspeed[1];
	}
	else if (channel_number == 3)
	{
		g_rotation_duration[2] = (float) ((g_capture_val[2] - g_old_capture_val[2])
										 +(updateCounter[2] * Max_TIMER_COUNT))
										 / F_ICU;

		g_wheelspeed[2] = ((((60 / (4 * g_rotation_duration[2])) * (2 * PI / 60)) * WHEEL_RADIUS) * (18 / 5));
		return g_wheelspeed[2];
	}
	else if (channel_number == 4)
	{
		g_rotation_duration[3] = (float) ((g_capture_val[3] - g_old_capture_val[3])
										 +(updateCounter[3] * Max_TIMER_COUNT))
										 / F_ICU;

		g_wheelspeed[3] = ((((60 / (4 * g_rotation_duration[3])) * (2 * PI / 60)) * WHEEL_RADIUS) * (18 / 5));
		return g_wheelspeed[3];
	}

	return -1;
}

uint16_t Average_car_speed (void)
{
   return ( Wheel_speed(1) + Wheel_speed(2)+Wheel_speed(3)+Wheel_speed(4) )/4;
}

float Engine_Speed(uint8_t channel_number)
{
	if (channel_number == 1)
	{
		// 1/4 duration of one rotation
		g_rotation_duration[4] = (float) ((g_capture_val[4] - g_old_capture_val[4])
										 +(updateCounter[4] * Max_TIMER_COUNT))
										 / F_ICU;
		// speed in km/h
		g_wheelspeed[4] = ((((60 / (12 * g_rotation_duration[4])) * (2 * PI / 60)) * WHEEL_RADIUS) * (18 / 5));
		return g_wheelspeed[4];
	}
	return -1;
}


