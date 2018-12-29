/*
 * steering.c
 *
 *  Created on: Dec 29, 2018
 *      Author: mohned
 */

#include "steering.h"

//private variables

uint32_t lastValue;
//uint32_t binary=0;
//uint32_t decimalNumber=0;


//functions definitions
uint32_t Steering_InputReading()
{
	uint32_t input=0;
	uint32_t value;
	value=INPUT_PORT->IDR;
    input =value & 0x0000001111111111;
	/*if(input>lastValue)
	{
		printf("moving ClockWise");
	}
		else
	{
		printf("moving CounterClockWise");
	} if used in lcd*/
	lastValue= input;
	return input;
}
uint32_t Steering_GrayToDecimalConversion(uint32_t graycode,int size)
{
	uint32_t Number;
	uint32_t gray[size];
	uint32_t binary[size];
	for(int i=0;i<size;i++)
	{
		gray[size-1-i]=graycode%10 ;
		graycode/=10;
	}
    binary[0] = gray[0];
    for (int i = 1; i <size; i++)
    {
        if (gray[i] == 0)
        {
            binary[i]= binary[i - 1];
        }
        else
        {
            if(binary[i-1]==0)
            {
            	binary[i] = 1;
            }
            else
            {
            	binary[i]=0;
            }
        }
    }
    Number=0;
    for (int i=0;i<size;i++)
    {
    	Number= Number<<1;
    	Number|=binary[i];
    }
     /* int remainder;
        decimalNumber=0;
        while (trial!=0)
        {
            remainder = trial%10;
            trial /= 10;
            decimalNumber += remainder*pow(2,i);
            i++;
        }*/
    return Number;
}
uint32_t Steering_ActualReading(uint32_t steering_decimal_reading)
{
	return (steering_decimal_reading*360)/1024;
}












