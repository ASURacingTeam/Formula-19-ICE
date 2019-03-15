#include "stm32f10x.h"
#include <stdio.h>
#define CRC32_POLYNOMIAL 0xEDB88320
#define CANID 0x0F

USART_InitTypeDef USART_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
uint8_t tuner[67],Response[13];
uint8_t i,R;
uint16_t dis;
uint16_t size;
uint8_t counter;
uint32_t CRC32;
bool check,TComp;

uint32_t CRC32_Value(uint16_t i)
{
	uint32_t ulCRC =(uint32_t)i;
	for(uint8_t f=0;f<8;f++)
	{
		if(ulCRC & 1) ulCRC = (ulCRC >> 1)^CRC32_POLYNOMIAL;
		else ulCRC >>= 1;
	}
	return ulCRC;
}

uint32_t CalculateBlock_CRC32(uint8_t j,uint8_t *Response)
{
	uint32_t ulTemp1,ulTemp2,ulCRC = 0;
	while(j++ != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFF;
		ulTemp2 = CRC32_Value(((uint16_t)ulCRC ^ *Response++)&0xFF);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}

void USART1_Init(void){
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Init(USART1,&USART_InitStructure);

	USART_Cmd(USART1,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	DMA_Cmd(DMA1_Channel5,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	DMA_Cmd(DMA1_Channel4, DISABLE);
}

void DMA1_Init(void){
	DMA_DeInit(DMA1_Channel5);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(USART1->DR));
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)tuner;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 67;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Response;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_BufferSize = 13;
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
	DMA_Init(DMA1_Channel4,&DMA_InitStructure);
}

void NVIC1_Init(void)
{
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

void RCC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_AFIO, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

void GPIOA_Init(void)
{
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


extern "C" void USART1_IRQHandler(void){
	if(!TComp){
	counter++;
	if(counter == 2) size = tuner[1]|(tuner[0]<<8);
	if(counter == size - 1){
		counter = 0;
		DMA_Cmd(DMA1_Channel5,DISABLE);
		TComp = true;
	}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	}

extern "C" void DMA1_Channel4_IRQHandler(){
	__NOP();
}

extern "C" void DMA1_Channel5_IRQHandler(){
	if(DMA_GetITStatus(DMA1_IT_TC4) != RESET) {
		check = false;
		DMA_ClearFlag(DMA1_FLAG_TC4);
	}
}

int main(void){
	RCC_Init();
	GPIOA_Init();
	DMA1_Init();
	USART1_Init();
	NVIC1_Init();
	TComp = false;
	dis = counter-size+1;

	while(1){
		R=tuner[2];
		dis = counter-size+1;
		if(!(dis)) TComp = false;
		else TComp = true;
		if (!TComp && !(dis)){
		switch (R)
			{
				i = 0;
				case 'A':
					Response[2]=1;
					Response[3]=0;
					CRC32 = CalculateBlock_CRC32(4,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+4]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'b':
					Response[2]=1;
					CRC32 = CalculateBlock_CRC32(3,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+3]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'c':
					Response[2]=1;
					Response[3]= 0x0f;
					Response[4]=Response[3];
					CRC32 = CalculateBlock_CRC32(5,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+5]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'f':
					Response[2]=1;
					CRC32 = CalculateBlock_CRC32(8,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+8]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case'F':
					Response[2]=1;
					Response[3]='0';
					Response[4]='0';
					Response[5]='2';
					CRC32 = CalculateBlock_CRC32(6,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+6]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case'g':
					Response[2]=1;
					CRC32 = CalculateBlock_CRC32(5,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+5]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case'h':
					Response[2]=1;
					CRC32 = CalculateBlock_CRC32(3,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+3]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'I':
					Response[2]=1;
					Response[3]=CANID;
					CRC32 = CalculateBlock_CRC32(4,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+4]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'k':
					Response[2]=1;
					CRC32 = CalculateBlock_CRC32(3,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+3]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'M':
					Response[2]=1;
					Response[3]=0x48;
					Response[4]=0x40;
					CRC32 = CalculateBlock_CRC32(5,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+5]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'Q':
					Response[2]=1;
					for(int k=0;k<21;k++) Response[k+3]=0x2+4;
					CRC32 = CalculateBlock_CRC32(23,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+23]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'r':
					Response[2]=1;
					Response[3]=0xEC;
					CRC32 = CalculateBlock_CRC32(5,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+5]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'S':
					Response[2]=1;
					CRC32 = CalculateBlock_CRC32(63,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+63]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;

				case 'w':
					Response[2]=1;
					CRC32 = CalculateBlock_CRC32(3,Response);
					for(int q=24;q<49;q+=8)
					{
						Response[i+3]=(uint8_t)(CRC32>>q);
						i++;
					}
					break;
				}


			}
		DMA_Cmd(DMA1_Channel4,ENABLE);

	}

  }
