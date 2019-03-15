#include "stm32f10x.h"


#define ADR_WIDTH 5
#define TX_PLOAD_WIDTH 32
#define RX_PLOAD_WIDTH 32

#define READ_nRF_REG    0x00  // Command for read register
#define WRITE_nRF_REG   0x20 	// Command for write register
#define RD_RX_PLOAD     0x61  // Command for read Rx payload
#define WR_TX_PLOAD     0xA0  // Command for write Tx payload
#define FLUSH_TX        0xE1 	// Command for flush Tx FIFO
#define FLUSH_RX        0xE2  // Command for flush Rx FIFO
#define REUSE_TX_PL     0xE3  // Command for reuse Tx payload
#define NOP             0xFF  // Reserve

//Define the register address for nRF24L01P
#define CONFIG          0x00  //  Configurate the status of transceiver, mode of CRC and the replay of transceiver status
#define EN_AA           0x01  //  Enable the atuo-ack in all channels
#define EN_RXADDR       0x02  //  Enable Rx Address
#define SETUP_AW        0x03  // Configurate the address width
#define SETUP_RETR      0x04  //  setup the retransmit
#define RF_CH           0x05  // Configurate the RF frequency
#define RF_SETUP        0x06  // Setup the rate of data, and transmit power
#define NRFRegSTATUS    0x07  //
#define OBSERVE_TX      0x08  //
#define CD              0x09  // Carrier detect
#define RX_ADDR_P0      0x0A  // Receive address of channel 0
#define TX_ADDR         0x10  //       Transmit address
#define RX_PW_P0        0x11  //  Size of receive data in channel 0
#define FIFO_STATUS     0x17 // FIFO Status

unsigned char  TX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
unsigned char  RX_ADDRESS[ADR_WIDTH]= {0xE7,0xE7,0xE7,0xE7,0xE7};
unsigned char x[32];
unsigned char arr[32] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,
												 0x17,0x18,0x19,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x30,0x31,0x32}; // RPM,Coolant,MAP,MAT,TPS,Weel_Speed,Battary,    .
unsigned char len = 32;
static volatile uint32_t gDelaycounter;
unsigned char pTX[32];

void NRF24l01_Delay_us(unsigned long us)
{
	unsigned char i;
	while(us--)
	{
		i = 100;
		while(i--)
		{
			__NOP();
		}
	}
}

void Sys_Init(void)
{
	//SysTick_Config(SystemCoreClock/1000);
	return;
}

void Delay_ms(volatile uint32_t ms)
{
	gDelaycounter = ms;
	while(gDelaycounter != 0);
}

void Delaycounter_De(void)
{
	if (gDelaycounter != 0)
		gDelaycounter--;
}

extern "C" void SysTick_Handler(void)
{
	Delaycounter_De();
}


void Spi_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );//PORTA,C,SPI1????
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PC4 <---> CS_pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//PC4 as CS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //PA5.6.7?AF??
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_CalculateCRC(SPI1, DISABLE);
	SPI_Cmd(SPI1,ENABLE);

}

void NRF24l01_HW_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );//PORTA????
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_2;//PA4 <---> CE_pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);//PA4 as CE

	Spi_Init();

}

void SPI1_NSS_L(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
}

void SPI1_NSS_H(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

void NRF24l01_NSS_L(void)
{
	SPI1_NSS_L();
}

void NRF24l01_NSS_H(void)
{
	SPI1_NSS_H();
}

void NRF24l01_CE_L(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
}

void NRF24l01_CE_H(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_3);;
}

unsigned char SPI1_Send_Byte(unsigned char dat)
{
	/* Loop while DR register in not emplty */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

	/* Send byte through the SPIx peripheral */
	SPI_I2S_SendData(SPI1, dat);

	/* Wait to receive a byte */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI1);
}

unsigned char NRF24l01_Send_Byte(unsigned char dat)
{
	return SPI1_Send_Byte(dat);
 }

unsigned char NRF24l01_WR_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;

	NRF24l01_NSS_L();
	NRF24l01_Delay_us(20);	// CSN low, init SPI transaction
	status = NRF24l01_Send_Byte(reg);// select register
	NRF24l01_Send_Byte(value); 	// ..and write value to it..
	//NRF24l01_Delay_us(20);
	NRF24l01_NSS_H();                   // CSN high again

	return(status);            // return NRF24l01 status unsigned char
}

unsigned char NRF24l01_RD_Reg(unsigned char reg)
{
	unsigned char reg_val;

	NRF24l01_NSS_L();                // CSN low, initialize SPI communication...
	NRF24l01_Delay_us(20);
	NRF24l01_Send_Byte(reg);            // Select register to read from..
	reg_val = NRF24l01_Send_Byte(0);    // ..then read register value
	NRF24l01_NSS_H();                // CSN high, terminate SPI communication

	return(reg_val);        // return register value
}


unsigned char NRF24l01_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
	unsigned int status,i;

	NRF24l01_NSS_L();                  // Set CSN low, init SPI tranaction
	NRF24l01_Delay_us(20);
	status = NRF24l01_Send_Byte(reg);  // Select register to write to and read status unsigned char

  for(i=0;i<Len;i++)
  {
     pBuf[i] = NRF24l01_Send_Byte(0);
   }

	NRF24l01_NSS_H();

	return(status);                    // return NRF24l01 status unsigned char
}


unsigned char NRF24l01_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char Len)
{
	unsigned int status;

	NRF24l01_NSS_L();
	NRF24l01_Delay_us(20);
	status = NRF24l01_Send_Byte(reg);
	for(int i=0; i<Len; i++)
	{
		NRF24l01_Send_Byte(*pBuf);
		x[i] = *pBuf;
		pBuf ++;
	}
	NRF24l01_NSS_H();
	return(status);
}

void NRF24l01_TX_Mode(void)
{
	NRF24l01_CE_L();

	NRF24l01_WR_Reg(WRITE_nRF_REG + SETUP_AW, 0x03); // setup add width 5 bytes
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + CONFIG, 0x70); // enable power up and ptx (need to be modified)
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01); //Enable data P0
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + SETUP_RETR, 0x00);//Auto Retransm	it Delay: 500 us, Auto Retransmit Count: Up to 2 Re-Transmit
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + RF_CH,0x02);// setup frequency
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + RF_SETUP ,0x07); //setup power 0dbm, rate 1Mbps
	NRF24l01_Delay_us(20);
	NRF24l01_Write_Buf(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH); // write address into tx_add
	NRF24l01_Delay_us(20);
	NRF24l01_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH); // write address into rx_add_p0
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + CONFIG, 0x70);
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + EN_AA, 0x00);     // Disable Auto.Ack:Pipe0
	NRF24l01_Delay_us(200);

	NRF24l01_CE_H();

}

void NRF24l01_RX_Mode(void)
{
	NRF24l01_CE_L();
	NRF24l01_WR_Reg(WRITE_nRF_REG + CONFIG, 0x73);
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + SETUP_AW, 0x03); // setup add width 5 bytes
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + RF_CH,0x02);// setup frequency
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + RF_SETUP,  0x07);// setup power and rate
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + RX_PW_P0,32); //Number of bytes in data P0
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + EN_RXADDR, 0x01); //Enable data P0
	NRF24l01_Delay_us(20);
	NRF24l01_Write_Buf(WRITE_nRF_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH); // write address into tx_add
	NRF24l01_Delay_us(20);
	NRF24l01_Write_Buf(WRITE_nRF_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH); // write address into rx_add_p0
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + EN_AA, 0x00);     //disable auto-ack for all channels
	NRF24l01_Delay_us(20);



	NRF24l01_WR_Reg(WRITE_nRF_REG + CONFIG, 0x33); // enable power up and prx (need to be modified)
	NRF24l01_Delay_us(20);
	NRF24l01_CE_H();
	NRF24l01_Delay_us(2000);
	//NRF24l01_CE_L();
}



void NRF24l01_TX_Packet(unsigned char* tx_buf)
{
	NRF24l01_CE_L();
	NRF24l01_WR_Reg(WRITE_nRF_REG+NRFRegSTATUS, 0x7E); // Write 1 to clear bit
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + CONFIG, 0x70); // enable power up and ptx
	NRF24l01_Delay_us(20);
	NRF24l01_NSS_L();
	NRF24l01_Send_Byte(FLUSH_TX);
	NRF24l01_Send_Byte(0x00);
	NRF24l01_NSS_H();
	NRF24l01_Delay_us(20);
	NRF24l01_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);
	NRF24l01_WR_Reg(WRITE_nRF_REG + CONFIG, 0x72);
	NRF24l01_CE_H();
	NRF24l01_Delay_us(500);
}

unsigned char NRF24l01_RX_Packet(unsigned char* rx_buf)
{
	unsigned char flag=0;
	unsigned char status;
	status=NRF24l01_RD_Reg(NRFRegSTATUS);
	NRF24l01_CE_L();
	NRF24l01_Delay_us(20);
	if(status & 0x40) //Data Ready RX FIFO interrupt
	{
		NRF24l01_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);
		flag =1;
	}
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG+NRFRegSTATUS, 0x40); // Write 1 to clear bit
	NRF24l01_Delay_us(20);
	NRF24l01_NSS_L();
	NRF24l01_Send_Byte(FLUSH_RX);//Flush RX FIFO
	NRF24l01_NSS_H();
	NRF24l01_Delay_us(20);
	NRF24l01_WR_Reg(WRITE_nRF_REG + CONFIG, 0x33); // enable power up and prx
	NRF24l01_CE_H();
	NRF24l01_Delay_us(20);
	return flag;
}

int main()
{
	//Sys_Init();
	NRF24l01_HW_Init();
	NRF24l01_TX_Mode();
	//pTX = x;
	while(1)
	{
		NRF24l01_TX_Packet(arr);
	}
}
