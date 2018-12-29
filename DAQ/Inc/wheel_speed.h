/*
 * wheel_speed.h
 *
 *  Created on: Dec 29, 2018
 *      Author: mohned
 */

#ifndef WHEEL_SPEED_H_
#define WHEEL_SPEED_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#define F_ICU 1000000            /*FREQUENCY OF INPUT CAPTURE*/
#define WHEEL_RADIUS 0.323       // radius of the wheel
#define PI 3.1415
#define Max_TIMER_COUNT  65536   // maximum number that timer can count

extern uint32_t g_old_capture_val[5];      /*TIME OF FIRST EDGE*/
extern uint32_t g_capture_val[5];          /*TIME OF SECOND EDGE*/
extern uint8_t updateCounter[5];           /*counter to know how many overflows between 2 edges*/

float Wheel_speed(uint8_t channel_number);
uint16_t Average_car_speed (void);
float Engine_Speed(uint8_t channel_number);


#endif /* WHEEL_SPEED_H_ */
