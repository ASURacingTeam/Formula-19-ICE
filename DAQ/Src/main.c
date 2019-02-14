
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
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "wheel_speed.h"
#include "steering.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// variables for mpu6050
MPU6050_RawValues mpu1;                          // structure of raw values readings

//variables for steering sensor
uint32_t steering_graycode_reading,steering_decimal_reading,steering_final_reading;

//variables for 4 suspension sensors
uint16_t adc_reading[4];
float measured_distance[4];

// variables of 5 wheel speed sensors
uint32_t g_old_capture_val[5] = {0,0,0,0,0};      /*TIME OF FIRST EDGE*/
uint32_t g_capture_val[5] = {0,0,0,0,0};          /*TIME OF SECOND EDGE*/
uint8_t updateCounter[5] = {0,0,0,0,0};           /*counter to know how many overflows between 2 edges*/
uint8_t Number_of_over_flows[5] = {0,0,0,0,0};    // each timer over flow the counter increases by 1
uint8_t flag[5] = {0,0,0,0,0};                    // to determine this is the first rising edge or the second one

//variables for CAN
CanTxMsgTypeDef TxMessage1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//for speed sensors
	HAL_TIM_Base_Start_IT(&htim2);                        /*TIMER 2 START COUNTING */
	HAL_TIM_IC_Start_IT(&htim2 , TIM_CHANNEL_1);

	HAL_TIM_Base_Start_IT(&htim1);                        /*TIMER 1 START COUNTING*/
    HAL_TIM_IC_Start_IT(&htim1 , TIM_CHANNEL_1);      /*START IC interrupt mode FOR CHANNEL 1*/
    HAL_TIM_IC_Start_IT(&htim1 , TIM_CHANNEL_2);      /*START IC interrupt mode FOR CHANNEL 2*/
    HAL_TIM_IC_Start_IT(&htim1 , TIM_CHANNEL_3);      /*START IC interrupt mode FOR CHANNEL 3*/
    HAL_TIM_IC_Start_IT(&htim1 , TIM_CHANNEL_4);      /*START IC interrupt mode FOR CHANNEL 4*/

    //for suspension sensors
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adc_reading,4);

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  //Inialization of mpu6050
  MPU6050_Init(&hi2c2, MPU6050_ADDRESS_AD0LOW, FULLSCALE_250, FULLSCALE_2g, SAMPLERATE_8KHz,&mpu1);
		TxMessage1.StdId = 0x7D0;
	  TxMessage1.RTR = CAN_RTR_DATA;
	  TxMessage1.IDE = CAN_ID_STD;
	  TxMessage1.DLC = 1;
	  TxMessage1.Data[0] =10;
	  hcan.pTxMsg = &TxMessage1;
	  HAL_CAN_Transmit(&hcan, 10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //read mpu readings
	  MPU6050_ReadRawData(&hi2c2,MPU6050_ADDRESS_AD0LOW,&mpu1);
	  //read steering reaings in gray code and convert it to decimal
	  steering_graycode_reading = Steering_InputReading();
	  steering_decimal_reading = Steering_GrayToDecimalConversion(steering_graycode_reading,10);
	  steering_final_reading = Steering_ActualReading(steering_decimal_reading);
	  //can transmit
	  TxMessage1.StdId = 0x7D0;
	  TxMessage1.RTR = CAN_RTR_DATA;
	  TxMessage1.IDE = CAN_ID_STD;
	  TxMessage1.DLC = 1;
	  TxMessage1.Data[0] =10;
	  hcan.pTxMsg = &TxMessage1;
	  HAL_CAN_Transmit(&hcan, 10);
	  if(hcan.State==HAL_CAN_STATE_TIMEOUT) {
		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		  HAL_Delay(1000);
	  }
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
		Number_of_over_flows[0]++;
		Number_of_over_flows[1]++;
		Number_of_over_flows[2]++;
		Number_of_over_flows[3]++;
	}

	if (htim->Instance == TIM2)
	{
		Number_of_over_flows[4]++;
	}

	if(htim->Instance==TIM3)
	{
		uint8_t i;
		for(i=0; i<4; i++)
		{
			//converting ADC readings into length in mm
			/*
			*    adc_reading     ------------> 4095 (full scale of 12 bit ADC)
			*  measured_distance   ----------> 30   (3cm)
			*/
			measured_distance[i]=adc_reading[i]*(30)/(4095.0);
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM1)
	{
		if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U)  // check if channel 1 is interrupted
		{
			if (flag[0] == 0)
			{
                Number_of_over_flows[0] = 0;
                flag[0] = 1;
			}
			else if (flag[0] == 1)
			{
                updateCounter[0] = Number_of_over_flows[0];
                flag[0] = 0;
			}
			g_old_capture_val[0] = g_capture_val[0];
			g_capture_val[0] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
		}

		if ((htim->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U)  // check if channel 2 is interrupted
		{
			if (flag[1] == 0)
			{
				Number_of_over_flows[1] = 0;
				flag[1] = 1;
			}
			else if (flag[1] == 1)
			{
				updateCounter[1] = Number_of_over_flows[1];
				flag[1] = 0;
			}
			g_old_capture_val[1] = g_capture_val[1];
			g_capture_val[1] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
		}

		if ((htim->Instance->CCMR2 & TIM_CCMR2_CC3S) != 0x00U)  // check if channel 3 is interrupted
		{
			if (flag[2] == 0)
			{
				Number_of_over_flows[2] = 0;
				flag[2] = 1;
			}
			else if (flag[2] == 1)
			{
				updateCounter[2] = Number_of_over_flows[2];
				flag[2] = 0;
			}
			g_old_capture_val[2] = g_capture_val[2];
			g_capture_val[2] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
		}

		if ((htim->Instance->CCMR2 & TIM_CCMR2_CC4S) != 0x00U)  // check if channel 4 is interrupted
		{

			if (flag[3] == 0)
			{
				Number_of_over_flows[3] = 0;
				flag[3] = 1;
			}
			else if (flag[3] == 1)
			{
				updateCounter[3] = Number_of_over_flows[3];
				flag[3] = 0;
			}
			g_old_capture_val[3] = g_capture_val[3];
			g_capture_val[3] = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
		}
	}

	if (htim->Instance == TIM2)
	{
		if ((htim->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U)  // check if channel 1 is interrupted
		{
			if (flag[4] == 0) {
				Number_of_over_flows[4] = 0;
				flag[4] = 1;
			}
			else if (flag[4] == 1)
			{
				updateCounter[4] = Number_of_over_flows[4];
				flag[4] = 0;
			}
			g_old_capture_val[4] = g_capture_val[4];
			g_capture_val[4] = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
		}
	}
}





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
