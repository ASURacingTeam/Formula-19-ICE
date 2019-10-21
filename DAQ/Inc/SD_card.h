
/*check these links:
 * https://openlabpro.com/guide/interfacing-microcontrollers-with-sd-card/
 * https://www.engineersgarage.com/avr-microcontroller/interfacing-sd-card-with-avr-microcontroller-part-38-46/
 * https://ralimtek.com/stm32/firmware/stm32_spi_sd/
 * https://electronics.stackexchange.com/questions/77417/what-is-the-correct-command-sequence-for-microsd-card-initialization-in-spi/238217
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include "stm32f1xx_hal.h"
#include "spi.h"

/*-------------------------------- SD card commands ----------------------------------------------*/

/*A command is 6 bytes consists of 1 byte for command no (2 MSBs are 01 following by command no),
 *4 bytes for argument and 1 byte for CRC
 */
	
#define CMD0 0x00   //software reset - init card in spi mode if CS is low instead of SDIO mode 
#define CMD1 0x01
#define CMD8 0x08	  //verify SD Card interface operating condition
#define CMD13 0x0D  //read the card status register
#define CMD16 0x10  //set block size
#define CMD17 0x11  //read a single data block from the card
#define CMD24 0x18  //write a single data block from the card
#define CMD55 0x37  //leading application commands (ACMD)
#define CMD58 0x3A  //read OCR (operating condition register) 
#define ACMD41 0x29 //activate the card's initialization process
/*------------------------------------------------------------------------------------------------*/

/*-------------------------------- SD card responses ---------------------------------------------*/
#define R1_READY_STATE 0x00       //no bit in response frame is set
#define R1_IDLE_STATE 0x01        //idle bit is set
#define R1_ILLEGAL_COMMAND 0x04   //illegal bit is set
#define DATA_START_BLOCK 0xFE     //indication for the start of a continuous transmission of a block of data
#define DATA_RES_MASK 0x1F        //mask for data response tokens after a write block operation
#define DATA_RES_ACCEPTED 0x05    //write data accepted token
/*-----------------------------------------------------------------------------------------------*/

/*------------------------------- public functions prototypes ------------------------------------------*/
	uint8_t SD_Initalize(SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_port, uint16_t CS_pin); //startup the card
	uint8_t SD_ReadBlock(uint32_t block_addr, uint8_t* buffer); //reads a single 512 byte block
	uint8_t SD_WriteBlock(uint32_t block_addr, uint8_t* buffer); //writes a single 512 byte block
/*----------------------------------------------------------------------------------------------*/


#endif /*SD_CARD_H_*/
