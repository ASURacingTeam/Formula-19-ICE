/*
 * steering.c
 *
 *  Created on: Dec 29, 2018
 *      Author: mohned
 */

#include "steering.h"

//private variables

uint32_t input=0;
uint32_t binary=0;
//uint32_t decimalNumber=0;
uint32_t Number=0;



//functions definitions
uint32_t Steering_InputReading()
{ uint32_t value=0;
  uint32_t lastValue=0;
  uint32_t input1=0;
  uint32_t readingbit13=0;
  uint32_t readingbit14=0;
	value=INPUT_PORT->IDR;
    input1 =value & 0x0000000011111111;
    readingbit13=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)<<9;
    readingbit14=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)<<10;
    input= input1|readingbit13|readingbit14;

	/*if(input>lastValue)
	 * {
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
	uint32_t gray[size];
		uint32_t binary[size];
	    int i = 0, remainder;
	for(int i=0;i<size;i++)
	{
		gray[size-1-i]=graycode%10 ;
		graycode/=10;

	}
	    binary[0] = gray[0];

	    for (int i = 1; i <size; i++) {
	        if (gray[i] == 0)
	            binary[i]= binary[i - 1];
	        else
	        {
	            if(binary[i-1]==0)
	            binary[i] = 1;
	            else
	            binary[i]=0;
	        }

	    }
	       Number=0;
	    for (int i=0;i<size;i++)
	    {
	    	Number= Number<<1;
	    	Number|=binary[i];
	    }
	     /* decimalNumber=0;
	        while (trial!=0)
	        {
	            remainder = trial%10;
	            trial /= 10;
	            decimalNumber += remainder*pow(2,i);
	            i++;
	        }*/
	        return Number;
}
uint32_t Steering_ActualReading()
{
	return (Number*360)/1024;
}












