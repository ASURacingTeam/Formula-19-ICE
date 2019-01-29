/*
 * steering.h
 *
 *  Created on: Dec 29, 2018
 *      Author: mohned
 */

#ifndef STEERING_H_
#define STEERING_H_

#include "stm32f1xx_hal.h"
#include "main.h"

#define INPUT_PORT GPIOB

uint32_t Steering_InputReading();
uint32_t Steering_GrayToDecimalConversion(uint32_t graycode,int size);
uint32_t Steering_ActualReading();

#endif /* STEERING_H_ */
