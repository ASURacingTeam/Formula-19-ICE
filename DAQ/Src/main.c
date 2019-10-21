
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "SD_card.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// SD card variables
/*SD card 512 bytes writing block representation*/
typedef union {
	struct {
		double acc[3];
		double gyro[3];
		double mag[3];
		double sus[4];
		double steering;
		double speed[5];
		uint8_t reserved[360];
	} Data;
	uint8_t buff[512];
} SD_data_t;

SD_data_t d1;
uint32_t block_no=1;
uint8_t status=1;

//wheel speed variables
uint32_t tim1_overflow_counter = 0;
uint32_t tim2_overflow_counter = 0;
uint32_t no_counts[5] = {0};
double Trev[5] = {0};

//suspension variables
volatile uint16_t adc_reading[4]={0};
volatile uint8_t measured_distance[4]={0};

// variables for mpu6050
MPU6050_Readings mpu1;                          // structure of mpu readings

//can variables
CanTxMsgTypeDef TxMessage1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		tim1_overflow_counter++;
	}
	if (htim == &htim2) {
		tim2_overflow_counter++;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		uint8_t i;
		for (i = 0; i < 4; i++) {
			//converting ADC readings into length in mm

			/*    adc_reading     ------------> 4096 (full scale of 12 bit ADC)
			 *  measured_distance   ----------> 30 (3cm)
			 */
			measured_distance[i] =(uint8_t)(adc_reading[i] * (30) / (4096));
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	static uint32_t tim1_ch1_last_capture = 0;
	static uint32_t tim1_ch2_last_capture = 0;
	static uint32_t tim1_ch3_last_capture = 0;
	static uint32_t tim1_ch4_last_capture = 0;
	static uint32_t tim2_ch1_last_capture = 0;
	static uint32_t tim1_ch1_capture = 0;
	static uint32_t tim1_ch2_capture = 0;
	static uint32_t tim1_ch3_capture = 0;
	static uint32_t tim1_ch4_capture = 0;
	static uint32_t tim2_ch1_capture = 0;
	if (htim == &htim2)
	{
		if (htim->Channel == 1)
		{
			tim2_ch1_last_capture = tim2_ch1_capture;
			tim2_ch1_capture = HAL_TIM_ReadCapturedValue(&htim2, 1);
			no_counts[4]=((tim2_ch1_capture + (tim2_overflow_counter * 6400)) - tim2_ch1_last_capture);
			Trev[4] = no_counts[4] * (1/(double)72000.0) * 12;  //revolution time (12 for no of pulses)
			d1.Data.speed[4] = (1/Trev[4]) * 60;   //rpm
			tim2_overflow_counter = 0;
		}
	}
	else if (htim == &htim1)
	{
		if (htim->Channel == 1)
		{
			tim1_ch1_last_capture = tim1_ch1_capture;
			tim1_ch1_capture = HAL_TIM_ReadCapturedValue(&htim1, 1);
			no_counts[0]=((tim1_ch1_capture + (tim1_overflow_counter * 6400)) - tim1_ch1_last_capture);
			Trev[0] = no_counts[0] * (1/(double)72000.0) * 4;   //revolution time (4 for no of pulses)
			d1.Data.speed[0] = (1/Trev[0]) * (3.14 * 0.065) * ((double)5.0/18);   // km/h (0.065 for wheel radius)
			tim1_overflow_counter = 0;
		}
		else if (htim->Channel == 2)
		{
			tim1_ch2_last_capture = tim1_ch2_capture;
			tim1_ch2_capture = HAL_TIM_ReadCapturedValue(&htim1, 2);
			no_counts[1]=((tim1_ch2_capture + (tim1_overflow_counter * 6400)) - tim1_ch2_last_capture);
			Trev[1] = no_counts[1] * (1/(double)72000.0) * 4;   //revolution time (4 for no of pulses)
			d1.Data.speed[1] = (1/Trev[1]) * (3.14 * 0.065) * ((double)5.0/18);   // km/h (0.065 for wheel radius)
			tim1_overflow_counter = 0;
		}
		else if (htim->Channel == 4)
		{
			tim1_ch3_last_capture = tim1_ch3_capture;
			tim1_ch3_capture = HAL_TIM_ReadCapturedValue(&htim1, 4);
			no_counts[2] =((tim1_ch3_capture + (tim1_overflow_counter * 6400)) - tim1_ch3_last_capture);
			Trev[2] = no_counts[2] * (1/(double)72000.0) * 4;   //revolution time (4 for no of pulses)
			d1.Data.speed[2] = (1/Trev[2]) * (3.14 * 0.065) * ((double)5.0/18);   // km/h (0.065 for wheel radius)
			tim1_overflow_counter = 0;
		}
		else if (htim->Channel == 8)
		{
			tim1_ch4_last_capture = tim1_ch4_capture;
			tim1_ch4_capture = HAL_TIM_ReadCapturedValue(&htim1, 8);
			no_counts[3] =  ((tim1_ch4_capture + (tim1_overflow_counter * 6400)) - tim1_ch4_last_capture);
			Trev[3] = no_counts[3] * (1/(double)72000.0) * 4;   //revolution time (4 for no of pulses)
			d1.Data.speed[3] = (1/Trev[3]) * (3.14 * 0.065) * ((double)5.0/18);   // km/h (0.065 for wheel radius)
			tim1_overflow_counter = 0;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Init(&hcan);
  HAL_SPI_Init(&hspi1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_reading,4);

  //Inialization of mpu6050
  MPU6050_Init(&hi2c2, MPU6050_ADDRESS_AD0LOW, FULLSCALE_250, FULLSCALE_2g, SAMPLERATE_1KHz,&mpu1);

  //SD card testing
  /*
  status=SD_Initalize(&hspi1,SD_CS_GPIO_Port,SD_CS_Pin);
  d1.Data.acc[0]=1;
  d1.Data.acc[1]=2;
  d1.Data.acc[2]=3;
  d1.Data.gyro[0]=1;
  d1.Data.gyro[1]=2;
  d1.Data.gyro[2]=3;
  d1.Data.mag[0]=1;
  d1.Data.mag[1]=2;
  d1.Data.mag[2]=3;
  status=SD_ReadBlock(block_no,d1.buff);
  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  MPU6050_ReadData(&hi2c2,MPU6050_ADDRESS_AD0LOW,&mpu1);

	  TxMessage1.StdId = 2000;
	  TxMessage1.RTR = CAN_RTR_DATA;
	  TxMessage1.IDE = CAN_ID_STD;
	  hcan.pTxMsg = &TxMessage1;
	  TxMessage1.DLC = 7;
	  TxMessage1.Data[0] =d1.Data.speed[0];//spped
	  TxMessage1.Data[1] =d1.Data.speed[1];//speed
	  TxMessage1.Data[2] =d1.Data.speed[2];//speed
	  TxMessage1.Data[3] =d1.Data.speed[3];//speed
	  TxMessage1.Data[4] =(uint8_t)(d1.Data.speed[0] + d1.Data.speed[1])/2;//avg
	  TxMessage1.Data[5] =((uint16_t)(d1.Data.speed[4]))>>8;//most byte rpm
	  TxMessage1.Data[6] =((uint16_t)(d1.Data.speed[4]));//least byte rpm
	  HAL_CAN_Transmit(&hcan, 10);
	  HAL_Delay(1000);
	  TxMessage1.StdId = 1998;
	  TxMessage1.DLC = 4;
	  TxMessage1.Data[0] =measured_distance[0];//suspension
	  TxMessage1.Data[1] =measured_distance[1];//suspension
	  TxMessage1.Data[2] =measured_distance[2];//suspension
	  TxMessage1.Data[3] =measured_distance[3];//suspension
	  HAL_CAN_Transmit(&hcan, 10);
	  HAL_Delay(1000);
	  TxMessage1.StdId = 2002;
	  TxMessage1.DLC = 6;
	  TxMessage1.Data[0] =((uint16_t)((mpu1.accelerometer_x+2)*100))>>8;//mpu accx - max is 2g
	  TxMessage1.Data[1] =((uint16_t)((mpu1.accelerometer_x+2)*100));
	  TxMessage1.Data[2] =((uint16_t)((mpu1.accelerometer_y+2)*100))>>8;//mpu accy
	  TxMessage1.Data[3] =((uint16_t)((mpu1.accelerometer_y+2)*100));
	  TxMessage1.Data[4] =((uint16_t)((mpu1.accelerometer_z+2)*100))>>8;//mpu accz
	  TxMessage1.Data[5] =((uint16_t)((mpu1.accelerometer_z+2)*100));
	  HAL_CAN_Transmit(&hcan, 10);
	  HAL_Delay(1000);
	  TxMessage1.StdId = 2003;
	  TxMessage1.DLC = 6;
	  TxMessage1.Data[0] =((uint16_t)((mpu1.gyroscope_x)+250))>>8;//mpu gyrox
	  TxMessage1.Data[1] =((uint16_t)((mpu1.gyroscope_x)+250));
	  TxMessage1.Data[2] =((uint16_t)((mpu1.gyroscope_y)+250))>>8;//mpu gyroy
	  TxMessage1.Data[3] =((uint16_t)((mpu1.gyroscope_y)+250));
	  TxMessage1.Data[4] =((uint16_t)((mpu1.gyroscope_z)+250))>>8;//mpu gyroz
	  TxMessage1.Data[5] =((uint16_t)((mpu1.gyroscope_z)+250));
	  HAL_CAN_Transmit(&hcan, 10);
	  HAL_Delay(1000);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
