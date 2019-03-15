#include <stm32f10x_conf.h>
#include <algorithm>

double test = 1;

bool syncing_done = false;
bool first_ignition = false;
bool second_ignition = false;
uint16_t syncing_RPM = 500; //Dummy Value till actual value known.

void RCC_Initializer(void)
{
	//GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	//Timers
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);

	//ADC
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //ADC PreScalor, ADC_CLK = PCCLK2/Prescalor, Must NOT Exceed 14MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	//DMA
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

}

void NVIC_Initializer(void)
{
	  NVIC_InitTypeDef NVIC_InitStructure;

	// Timer2_Interrupt_Enable (Injection)
 	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Timer3_Interrupt_Enable (RPM)
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Timer4_Interrupt_Enable (Advance Angle and Ignition)
 	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// EX_Interrupt_Enable
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; //Depends on Which Pin we Will use Finally For Input Signals
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

	//Enable DMA1_Interrupt for ADC values
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Timers_Initializer(void)
{
	//TIM3_Configartion;
	TIM_TimeBaseInitTypeDef Timer_3;
	Timer_3.TIM_Prescaler = 21; //This will overflow every 20.2mSec. Assuming Worst Case RPM (Syncing) is 300RPM which is 16.6mSec
	Timer_3.TIM_Period = 65535;
	Timer_3.TIM_CounterMode = TIM_CounterMode_Up;
	Timer_3.TIM_ClockDivision = TIM_CKD_DIV1;
	Timer_3.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &Timer_3);

	//TIM3_Input_Capture_Mode
	TIM_ICInitTypeDef Input_Capture;
	Input_Capture.TIM_Channel = TIM_Channel_2;
	Input_Capture.TIM_ICPolarity = TIM_ICPolarity_Rising;
	Input_Capture.TIM_ICSelection = TIM_ICSelection_DirectTI;
	Input_Capture.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	Input_Capture.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM3, &Input_Capture);

	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	//TIM2_Configartion;
	TIM_TimeBaseInitTypeDef Timer_2_syncing;
	Timer_2_syncing.TIM_Prescaler = 127; //First For Syncing CAM, Overflow every 116.5 mSec. Assuming longest pulse during syncing takes 0.96 mSec
	Timer_2_syncing.TIM_Period = 65535;
	Timer_2_syncing.TIM_CounterMode = TIM_CounterMode_Up;
	Timer_2_syncing.TIM_ClockDivision = TIM_CKD_DIV1;
	Timer_2_syncing.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &Timer_2_syncing);

	//TIM2_Input_Capture_Mode
	TIM_ICInitTypeDef Input_Capture_syncing;
	Input_Capture_syncing.TIM_Channel = TIM_Channel_4;
	Input_Capture_syncing.TIM_ICPolarity = TIM_ICPolarity_Rising;
	Input_Capture_syncing.TIM_ICSelection = TIM_ICSelection_DirectTI;
	Input_Capture_syncing.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	Input_Capture_syncing.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM2, &Input_Capture_syncing);

	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void GPIO_Initializer(char port_x, int PIN, char mode, int speed)
{
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = (uint16_t) (1 << PIN);

	switch(mode)
	{
		case 'O': gpio.GPIO_Mode = GPIO_Mode_Out_PP; break;
		case 'I': gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING; break;
		case 'A': gpio.GPIO_Mode = GPIO_Mode_AIN; break;
		case 'F': gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	}

	switch(speed)
	{
		case 2: gpio.GPIO_Speed = GPIO_Speed_2MHz; break;
		case 10: gpio.GPIO_Speed = GPIO_Speed_10MHz; break;
		case 50: gpio.GPIO_Speed = GPIO_Speed_50MHz;
	}

	GPIO_Init((GPIO_TypeDef *) (GPIOA_BASE + (port_x - 'A')*1024), &gpio);
}

void EXIT_Initializer(char port_x, int PIN, int speed)
{
	GPIO_Initializer(port_x, PIN, 'I', speed);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA  + (port_x - 'A'), GPIO_PinSource0 + PIN);

	EXTI_InitTypeDef EXTI_InitStruct;

	EXTI_InitStruct.EXTI_Line = (uint16_t) (1 << PIN);
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStruct);
}

#define ADC1_DR_Address ((uint32_t)0x4001244C)
uint16_t map_mat_tps[3];

void ADC_Initializer()
{
	DMA_InitTypeDef DMA_InitStructure;

  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) map_mat_tps;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  DMA_Cmd(DMA1_Channel1, ENABLE);

	ADC_InitTypeDef adc;

	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = ENABLE;
	adc.ADC_ContinuousConvMode = ENABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = 3;

	ADC_Init(ADC1, &adc);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_28Cycles5);

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	ADC_DMACmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);

	//Calibration Commands
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1)); //Waiting For Calibration RESET

  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1)); //Waiting For Calibration

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

double MAP;
double MAT;
double TPS;

extern "C" void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC1))
		{
			DMA_ClearITPendingBit(DMA1_IT_GL1);

			//This values are only Voltages, Need to know Min and Max Reading of MAP MAT and TPS to be able to make Static IO Relation

			MAP = ((double)(map_mat_tps[0])/0xFFF)*3.3;
			MAT = ((double)(map_mat_tps[1])/0xFFF)*3.3;
			TPS = ((double)(map_mat_tps[2])/0xFFF)*3.3;
		}
}


bool compression = false;
bool TDC = false;
bool BDC = false;

uint16_t IC3ReadValue1 = 0, IC3ReadValue2 = 0;
uint16_t IC3ReadValue1_Syncing = 0, IC3ReadValue2_Syncing = 0;
uint16_t CaptureNumber = 0;
uint16_t CaptureNumber_Syncing = 0;
uint32_t Capture = 0;
uint32_t Capture_Syncing = 0;
double Period1_Syncing = 0;
double Period2_Syncing = 0;
double period_per_teeth = 0;
//uint32_t TIM3Freq = 0;
uint16_t RPM = 0;
double fuel_mass;
double fuel_mass_syncing = 12.3; //Dummy Value till actual Syncing Fuel Mass is known
int current_tooth = 0;
int injection_tooth = 21; //Dummy Value till actual injection Tooth is known
int ignition_tooth = 255; //To Ensure It takes its first value during Syncing
double time_required_advance = 0;
double time_required_inj = 0;
double time_required_ign = 0.030; //Random Value till actual Coil Charge Value known

void ignite()
{
	if (current_tooth <= 6)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_6); //piston 3
	}
	else if(current_tooth <= 12)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_7); //piston 4
	}
	else if(current_tooth <= 18)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_5); //piston 2
	}
	else if(current_tooth <= 24)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_4); //piston 1
	}
}

void deignite()
{
	if (current_tooth <= 6)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_6); //piston 3
	}
	else if(current_tooth <= 12)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_7); //piston 4
	}
	else if(current_tooth <= 18)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_5); //piston 2
	}
	else if(current_tooth <= 24)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_4); //piston 1
	}
}

bool ignite_check = false;

extern "C" void TIM4_IRQHandler(void) //This Logic will only work if only 1 ignition coil is on at a time.
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		if(ignite_check)
		{
			TIM_Cmd(TIM4, DISABLE);
			TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);

			deignite();
			ignite_check = false;
		}
		else
		{
			TIM_DeInit(TIM4);

			TIM_TimeBaseInitTypeDef timer_4;
			timer_4.TIM_Prescaler = 63;
			timer_4.TIM_Period = (uint16_t) (time_required_ign*(72000000/64));
			timer_4.TIM_CounterMode = TIM_CounterMode_Up;
			timer_4.TIM_ClockDivision = TIM_CKD_DIV1;
			timer_4.TIM_RepetitionCounter = 0;
			TIM_TimeBaseInit(TIM4, &timer_4);

			TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
			TIM_Cmd(TIM4, ENABLE);

			ignite();
			ignite_check = true;
		}
	}
}

void inject_fuel(double fuel_m)
{
	//time_required_inj = fuel_m * 62/5; //RANDOM EQUATION TILL ACTUAL EQUATION KNOWN

	time_required_inj = 0.030*test; //For testing on OSCILOSCOPE

	TIM_DeInit(TIM2); //This logic will only work assuming injectors are never open together.

	TIM_TimeBaseInitTypeDef timer_2;
	timer_2.TIM_Prescaler = 63; //Will overflow every 20.2mSec
	timer_2.TIM_Period = (uint16_t) (time_required_inj*(72000000/64));
	timer_2.TIM_CounterMode = TIM_CounterMode_Up;
	timer_2.TIM_ClockDivision = TIM_CKD_DIV1;
	timer_2.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timer_2);

	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	if (current_tooth <= 6)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_2); //piston 3
	}
	else if(current_tooth <= 12)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_3); //piston 4
	}
	else if(current_tooth <= 18)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_1); //piston 2
	}
	else if(current_tooth <= 24)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_0); //piston 1
	}
}

int check2 = 0;
int check3 = 0;

extern "C" void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		TIM_Cmd(TIM2, DISABLE);
		TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

		if (current_tooth <= 6)
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_2); //piston 3
		}
		else if(current_tooth <= 12)
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_3); //piston 4
		}
		else if(current_tooth <= 18)
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_1); //piston 2
		}
		else if(current_tooth <= 24)
		{
			GPIO_ResetBits(GPIOB, GPIO_Pin_0); //piston 1
		}
	}
  else if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) //Syncing!!!
  {
     // Clear TIM2 Capture compare interrupt pending bit //
     TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);

		 if(CaptureNumber_Syncing == 0)
		 {
		  	IC3ReadValue1_Syncing = TIM_GetCapture4(TIM2);
				CaptureNumber_Syncing = 1;
		 }
		 else if(CaptureNumber_Syncing == 1)
		 {
		  	IC3ReadValue2_Syncing = TIM_GetCapture4(TIM2);

				if(Period1_Syncing == 0)
				{
					if (IC3ReadValue2_Syncing > IC3ReadValue1_Syncing)
						Capture_Syncing = (IC3ReadValue2_Syncing - IC3ReadValue1_Syncing);
					else
						Capture_Syncing = ((0xFFFF - IC3ReadValue1_Syncing) + IC3ReadValue2_Syncing);

					Period1_Syncing = Capture_Syncing*((double)128/72000000);
				}
				else
				{
					if (IC3ReadValue2_Syncing > IC3ReadValue1_Syncing)
						Capture_Syncing = (IC3ReadValue2_Syncing - IC3ReadValue1_Syncing);
					else
						Capture_Syncing = ((0xFFFF - IC3ReadValue1_Syncing) + IC3ReadValue2_Syncing);

					Period2_Syncing = Capture_Syncing*((double)128/72000000);
				}

				IC3ReadValue1_Syncing = IC3ReadValue2_Syncing;

				if (Period2_Syncing > 0)
				{
					if (Period1_Syncing > Period2_Syncing) //In Refrence Pulse. Thus, Either Tooth 7 or 19. We will Assume Compression
					{
						TIM_Cmd(TIM2, DISABLE);
						TIM_ITConfig(TIM2, TIM_IT_CC4, DISABLE);

						current_tooth = 19;
						inject_fuel(fuel_mass_syncing);

						ignition_tooth = 23;
						time_required_advance = (period_per_teeth/30) * (30 - 15); //Depends on Syncing Advance Angle, Here it is Assumed 15 degrees (Dummy).

						first_ignition = true;
					}
					else
					{
						Period1_Syncing = Period2_Syncing; //Wait Until you are In Refrence Pulse.
					}
				}
		 }
	}
}

extern "C" void TIM3_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM3, TIM_IT_CC2) == SET)
   {
     // Clear TIM3 Capture compare interrupt pending bit //
     TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		 current_tooth++;

		 if(current_tooth > 24)
			 current_tooth = 1;

		 if(syncing_done == false) //Syncing!!!
		 {
			 if(first_ignition && current_tooth == 3) ///To wait until 3 pins have passed for RPM to increase.
			 {
				 if(RPM > syncing_RPM)
				 {
				 	 syncing_done = true;
					 first_ignition = false;

					 ////////////
					 test = 0.0005*2/0.030;
					 time_required_ign = 0.0005*2;
					 //////////

					 EXIT_Initializer('A', 3, 50);

					 BDC = true;
					 compression = false;
				 }
				 else if (second_ignition == false)
				 {
				 	 current_tooth = 15;
					 second_ignition = true;
				 }
			 }
			 if(current_tooth == ignition_tooth)
			 {
			 		 	 TIM_DeInit(TIM4);

						 TIM_TimeBaseInitTypeDef Timer_4;
						 Timer_4.TIM_Prescaler = 63; //This will overflow every 20.2 mSec. Assuming Worst Case RPM (IDLE) is 3000RPM which is 1.67mSec
						 Timer_4.TIM_Period = (uint16_t) (time_required_advance*(72000000/64));
						 Timer_4.TIM_CounterMode = TIM_CounterMode_Up;
						 Timer_4.TIM_ClockDivision = TIM_CKD_DIV1;
						 Timer_4.TIM_RepetitionCounter = 0;
						 TIM_TimeBaseInit(TIM4, &Timer_4);

						 TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
						 TIM_Cmd(TIM4, ENABLE);
			 }
		 }
		 else if(syncing_done)
		 {
			if(current_tooth == ignition_tooth || current_tooth == ignition_tooth-6 || current_tooth == ignition_tooth-12 || current_tooth == ignition_tooth-18)
			{
				if(time_required_advance > 0)
				{
						TIM_DeInit(TIM4);

						TIM_TimeBaseInitTypeDef Timer_4;
						Timer_4.TIM_Prescaler = 63; //This will overflow every 20.2mSec. Assuming Worst Case RPM (IDLE) is 3000RPM which is 1.67mSec
						Timer_4.TIM_Period = (uint16_t) (time_required_advance*(72000000/64));
						Timer_4.TIM_CounterMode = TIM_CounterMode_Up;
						Timer_4.TIM_ClockDivision = TIM_CKD_DIV1;
						Timer_4.TIM_RepetitionCounter = 0;
						TIM_TimeBaseInit(TIM4, &Timer_4);

						TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
						TIM_Cmd(TIM4, ENABLE);
				}
				else
					ignite();
			}

			if(current_tooth == injection_tooth || current_tooth == injection_tooth-6 || current_tooth == injection_tooth-12 || current_tooth == injection_tooth-18)
			{
					inject_fuel(fuel_mass);
			}
		 }

		 if(CaptureNumber == 0)
		 {
		  	IC3ReadValue1 = TIM_GetCapture2(TIM3);
				CaptureNumber = 1;
		 }
		 else if(CaptureNumber == 1)
		 {
		  	IC3ReadValue2 = TIM_GetCapture2(TIM3);

				if (IC3ReadValue2 > IC3ReadValue1)
					Capture = (IC3ReadValue2 - IC3ReadValue1);
				else
					Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2);

				//Frequency computation in case needed
				//TIM3Freq = (uint32_t) (72000000 / (Capture*6));

				period_per_teeth = Capture*((double)22/72000000);
				RPM = 60/(period_per_teeth*12);

				IC3ReadValue1 = IC3ReadValue2;
		 }
	}
}

int arr_x[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; //Horrizental Axis is MAP
int arr_y[16] = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000, 15000}; // Vertical Axis is RPM


double arr_AFR[16][16] = {
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
	};

double arr_VO[16][16] = {
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
	};

double arr_AVA[16][16] = {
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
		{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
	};

double bilinear_interpolate(double arr[16][16], double x, double y)
{
	int col_bound;
	int row_bound;

	double Q11;
	double Q12;
	double Q21;
	double Q22;

	double x2;
	double x1;
	double y2;
	double y1;
	double result = 0;

	col_bound = std::lower_bound(&arr_x[0], &arr_x[2], x) - &arr_x[0];
	row_bound = std::lower_bound(&arr_y[0], &arr_y[2], y) - &arr_y[0];

	if (row_bound == 0)
		row_bound++;
	if (col_bound == 0)
		col_bound++;

	Q11 = arr[row_bound - 1][col_bound - 1];
	Q21 = arr[row_bound - 1][col_bound];
	Q12 = arr[row_bound][col_bound - 1];
	Q22 = arr[row_bound][col_bound];

	x2 = arr_x[col_bound];
	x1 = arr_x[col_bound - 1];
	y2 = arr_y[row_bound];
  y1 = arr_y[row_bound - 1];

	result += (((x2 - x)*(y2 - y)) / ((x2 - x1)*(y2 - y1)))*Q11;
	result += (((x - x1)*(y2 - y)) / ((x2 - x1)*(y2 - y1)))*Q21;
	result += (((x2 - x)*(y - y1)) / ((x2 - x1)*(y2 - y1)))*Q12;
	result += (((x - x1)*(y - y1)) / ((x2 - x1)*(y2 - y1)))*Q22;

	return result;
}

#define R ((uint16_t) 287)
#define V 30 //Dummy Value Till Actual Volume

extern "C" void EXTI3_IRQHandler(void)
{
	double air_mass;
	double AFR = 0;
	double VO = 0;
	double AVA = 0;

  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
			if(TDC) //This is the TDC pulse
			{
				BDC = true;
				TDC = false;

				if(compression)
				{
					compression = false;

					current_tooth = 0;

					air_mass = (MAP*V)/(R*MAT); //R units is J/(KG.KELVIN)

					AFR = bilinear_interpolate(arr_AFR, RPM, MAP);
					VO = bilinear_interpolate(arr_VO, RPM, MAP);
					AVA = bilinear_interpolate(arr_AVA, RPM, MAP);

					fuel_mass = air_mass/(AFR*VO);

					int offset = 1;

					while(AVA > 30)
					{
						AVA -= 30;
						offset++;
					}

					ignition_tooth = 24 - offset;

					time_required_advance = (period_per_teeth/30) * (30 - AVA);
				}
				else
				{
					compression = true;
				}
			}
			else if(BDC) //This is the BDC pulse
			{
				BDC = false;
			}
			else //This is the refrence pulse
			{
				TDC = true;
			}
	}

	EXTI_ClearFlag(EXTI_Line3);
}

int main(void)
{
	RCC_Initializer();

	GPIO_Initializer('A' , 7, 'I', 50); //Crank Signal, Used for Input Capture Mode
	GPIO_Initializer('A' , 0, 'A', 50); //Used for ADC
	GPIO_Initializer('A' , 1, 'A', 50); //Used for ADC
	GPIO_Initializer('A' , 2, 'A', 50); //Used for ADC
	GPIO_Initializer('A' , 3, 'I', 50); //Cam Signal, Used for Input Capture Mode in Syncing
	GPIO_Initializer('B' , 0, 'O', 50); //Injector of Piston 1
	GPIO_Initializer('B' , 1, 'O', 50); //Injector of Piston 2
	GPIO_Initializer('B' , 2, 'O', 50); //Injector of Piston 3
	GPIO_Initializer('B' , 3, 'O', 50); //Injector of Piston 4
	GPIO_Initializer('B' , 4, 'O', 50); //Ignition of Piston 1
	GPIO_Initializer('B' , 5, 'O', 50); //Ignition of Piston 2
	GPIO_Initializer('B' , 6, 'O', 50); //Ignition of Piston 3
	GPIO_Initializer('B' , 7, 'O', 50); //Ignition of Piston 4

	NVIC_Initializer();

	Timers_Initializer();

	ADC_Initializer();

	//EXIT_Initializer('B', 5, 2);

	while(1)
	{
	};

}
