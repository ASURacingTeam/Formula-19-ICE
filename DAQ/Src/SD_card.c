
#include "SD_card.h"

SPI_HandleTypeDef* SD_hspi; //spi handler that SD card is connected with
GPIO_TypeDef* SD_CS_port;   //chip select port
uint16_t SD_CS_pin;         //chip select pin
static uint32_t count=0;
static uint8_t state;
static uint8_t SD_card_t1=0;
/*------------------------------------ private functions prototypes ------------------------------*/
HAL_StatusTypeDef SPI_Recieve(SPI_HandleTypeDef* hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef lSPI_WaitFlagStateUntilTimeout(SPI_HandleTypeDef *hspi,
		uint32_t Flag, uint32_t State, uint32_t Timeout,
		uint32_t Tickstart); //helping function for SPI receive function
		
void WaitUntilReady(); //waits until the card is ready
uint8_t SendCommand(uint8_t command, uint32_t argument); //send a command to SD card
uint8_t SendACMD(uint8_t command, uint32_t argument);	   //send an application command (ACMD) to SD card 
void SelectCard(void);
void DeselectCard(void);
/*------------------------------------------------------------------------------------------------*/
	
HAL_StatusTypeDef lSPI_WaitFlagStateUntilTimeout(
		SPI_HandleTypeDef *hspi, uint32_t Flag, uint32_t State,
		uint32_t Timeout, uint32_t Tickstart) 
{
	while ((hspi->Instance->SR & Flag) != State) 
	{
		if (Timeout != HAL_MAX_DELAY)
		{
			if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) >= Timeout)) 
			{
				/* Disable the SPI and reset the CRC: the CRC value should be cleared
				 on both master and slave sides in order to resynchronize the master
				 and slave for their respective CRC calculation */

				/* Disable TXE, RXNE and ERR interrupts for the interrupt process */
					__HAL_SPI_DISABLE_IT(hspi,
					(SPI_IT_TXE | SPI_IT_RXNE | SPI_IT_ERR));

				if ((hspi->Init.Mode == SPI_MODE_MASTER)
						&& ((hspi->Init.Direction == SPI_DIRECTION_1LINE)
								|| (hspi->Init.Direction == SPI_DIRECTION_2LINES_RXONLY))) 
				{
					/* Disable SPI peripheral */
					__HAL_SPI_DISABLE(hspi);
				}

				/* Reset CRC Calculation */
				if (hspi->Init.CRCCalculation == SPI_CRCCALCULATION_ENABLE) 
				{
					SPI_RESET_CRC(hspi);
				}

				hspi->State = HAL_SPI_STATE_READY;

				/* Process Unlocked */
				__HAL_UNLOCK(hspi);

				return HAL_TIMEOUT;
			}
		}
	}
	return HAL_OK;
}
		
/*custom SPI Recieve function that does not clock out whatever was in the array your reading 
  into before hand as original HAL function does, and instead just passes 0xFF out the port */
HAL_StatusTypeDef SPI_Recieve(SPI_HandleTypeDef* hspi, uint8_t* pData, uint16_t Size) 
{
	HAL_StatusTypeDef errorcode = HAL_OK;

	/* Process Locked */
	__HAL_LOCK(hspi);

	/* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
	if (hspi->State == HAL_SPI_STATE_READY) {
		hspi->State = HAL_SPI_STATE_BUSY_TX_RX;
	}

	/* Set the transaction information */
	hspi->ErrorCode = HAL_SPI_ERROR_NONE;
	hspi->pRxBuffPtr = (uint8_t *) pData;
	hspi->RxXferCount = Size;
	hspi->RxXferSize = Size;
	hspi->pTxBuffPtr = (uint8_t *) pData;
	hspi->TxXferCount = Size;
	hspi->TxXferSize = Size;

	/*Init field not used in handle to zero */
	hspi->RxISR = NULL;
	hspi->TxISR = NULL;
	/* Check if the SPI is already enabled */
	if ((hspi->Instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) {
		/* Enable SPI peripheral */
		__HAL_SPI_ENABLE(hspi);
	}
	/* Transmit and Receive data in 8 Bit mode */
	while ((hspi->RxXferCount > 0U)) {
		hspi->Instance->DR = 0xFF; //send data
		while (!(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE)))
			;
		while (!(__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE)))
			;
		(*(uint8_t *) pData++) = hspi->Instance->DR;
		hspi->RxXferCount--;
	}

	if (lSPI_WaitFlagStateUntilTimeout(hspi, SPI_FLAG_BSY, RESET, 100,
			HAL_GetTick()) != HAL_OK) {
		hspi->ErrorCode |= HAL_SPI_ERROR_FLAG;

		errorcode = HAL_TIMEOUT;
	}

	hspi->State = HAL_SPI_STATE_READY;
	__HAL_UNLOCK(hspi);
	return errorcode;
}

void WaitUntilReady()
{
	uint8_t trans_byte=0xFF;
	uint8_t rec_byte=0xFF;
	while(rec_byte!=0xFF)
	{
		HAL_SPI_TransmitReceive(SD_hspi,&trans_byte,&rec_byte,1,1);
	}
	
}
void SelectCard(void)
{
	HAL_GPIO_WritePin(SD_CS_port,SD_CS_pin,GPIO_PIN_RESET);
}

void DeselectCard(void)
{
	HAL_GPIO_WritePin(SD_CS_port,SD_CS_pin,GPIO_PIN_SET);
}

uint8_t SendCommand(uint8_t command, uint32_t argument)
{
	SelectCard();
	WaitUntilReady();
	uint8_t command_seq[6]; //6 bytes command to be sent to SD card
	command_seq[0]=(uint8_t)(command | 0x40);
	command_seq[1]=(uint8_t)(argument>>24);
	command_seq[2]=(uint8_t)(argument>>16);
	command_seq[3]=(uint8_t)(argument>>8);
	command_seq[4]=(uint8_t)(argument&0xFF);
	if(command==CMD0) //CRC byte
	{
		command_seq[5]=0x95;
	}
	else if(command==CMD8)
	{
		command_seq[5]=0x87;

	}
	/*else if(command==CMD55)
	{
		command_seq[5]=0x65;
	}
	else if(command==ACMD41)
	{
		command_seq[5]=0x77;
	}*/
	else if(command==CMD1)
	{
		command_seq[5]=0xF9;
	}
	else
	{
		command_seq[5]=0xFF;
	}
	HAL_SPI_Transmit(SD_hspi,command_seq,6,100);
	uint8_t rec_byte=0xFF;
	uint8_t trials_no=20;
	while((rec_byte&0x80) && trials_no) //wait until SD card respond to the  sent command
	{
		SPI_Recieve(SD_hspi, &rec_byte,1);
		trials_no--;
	}

	return rec_byte; //return response frame 
}

uint8_t SendACMD(uint8_t command, uint32_t argument)
{
	count++;
	SendCommand(CMD55,0);
	return SendCommand(command,argument);
}

uint8_t SD_Initalize(SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_port, uint16_t CS_pin)
{
	SD_hspi=hspi;
	SD_CS_port=CS_port;
	SD_CS_pin=CS_pin;
	SD_hspi->Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;
	HAL_SPI_Init(SD_hspi);
	//we should supply 74~80 clk cycles while CS is idle (high)
	DeselectCard();
	uint8_t buffer[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
	HAL_SPI_Transmit(SD_hspi, buffer, 4, 100);
	HAL_SPI_Transmit(SD_hspi, buffer, 4, 100);
	HAL_SPI_Transmit(SD_hspi, buffer, 4, 100);
	HAL_SPI_Transmit(SD_hspi, buffer, 4, 100);
	HAL_SPI_Transmit(SD_hspi, buffer, 4, 100);
	HAL_Delay(5);
	SelectCard();
	//select SPI mode for SD card
	while(SendCommand(CMD0,0)!=R1_IDLE_STATE);

	if(SendCommand(CMD8,0x1AA)&R1_ILLEGAL_COMMAND)
	{
		//DeselectCard();
		//return 0;
		SD_card_t1=1;
	}
	else 
	{
		HAL_SPI_Receive(SD_hspi, buffer, 4, 100);
		if(buffer[3]!=0xAA)
		{
			return 0;
		}
	}

	//while((state=SendCommand(CMD1,0))!=R1_READY_STATE);
	if(SD_card_t1)
	{
		while((state =SendACMD(ACMD41,0x00000000))!=R1_READY_STATE);
	}
	else
	{
		while ((state = SendACMD(ACMD41, 0X40000000)) != R1_READY_STATE);
	}

	if(!SD_card_t1)
	{
		if(SendCommand(CMD58,0))
		{
			DeselectCard();
			return 0;
		}
	}

	SPI_Recieve(SD_hspi, buffer, 4);
	DeselectCard();
	
	SD_hspi->Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_2;
	HAL_SPI_Init(SD_hspi);
	HAL_Delay(1000);
	return 1;
}

uint8_t SD_ReadBlock(uint32_t block_addr, uint8_t* buffer)
{
	if(SD_card_t1)
	{
		block_addr<<=9;
	}
	if(SendCommand(CMD17,block_addr))
	{
		DeselectCard();
		return 0;
		
	}
	
	uint8_t temp = 0xFF;
	while (temp == 0xFF) {
		HAL_SPI_Receive(SD_hspi, &temp, 1, 100);
	}
	
	SPI_Recieve(SD_hspi, buffer, 512);
	
	temp=0xFF;
	SPI_Recieve(SD_hspi, &temp, 1);
	temp=0xFF;
	SPI_Recieve(SD_hspi, &temp, 1);
	
	DeselectCard();
	return 1;
}

uint8_t SD_WriteBlock(uint32_t block_addr, uint8_t* buffer)
{
//	if(SD_card_t1)
//	{
//		block_addr<<=9;
//	}
	if(SendCommand(CMD24,block_addr))
	{
		DeselectCard();
		return 0;
	}
	
	uint8_t temp=DATA_START_BLOCK;
	HAL_SPI_Transmit(SD_hspi,&temp,1,100);
	HAL_SPI_Transmit(SD_hspi,buffer,512,100);
	temp = 0xFF;
	HAL_SPI_Transmit(SD_hspi, &temp, 1, 100);
	HAL_SPI_Transmit(SD_hspi, &temp, 1, 100);
	
	SPI_Recieve(SD_hspi,&temp, 1);
	if ((temp & DATA_RES_MASK) != DATA_RES_ACCEPTED) 
	{
		DeselectCard();
		return 0;
	}
	// wait for flash programming to complete
	WaitUntilReady();
	
	if (SendCommand(CMD13, 0)) 
	{
		DeselectCard();
		return 0;
	}
	SPI_Recieve(SD_hspi, &temp, 1);
	if (temp) 
	{
		DeselectCard();
		return 0;
	}
	
	DeselectCard();
	return 1;
}
