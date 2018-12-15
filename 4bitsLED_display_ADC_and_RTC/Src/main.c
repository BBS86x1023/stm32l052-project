
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#define SEVEN_SEGMENT_SCAN0		0
#define SEVEN_SEGMENT_SCAN1		1
#define SEVEN_SEGMENT_SCAN2		2
#define SEVEN_SEGMENT_SCAN3		3
#define SEVEN_SEGMENT_BLACK		4

#define INITIAL 0
#define TASK1	1
#define TASK2	2
#define TASK3	3
#define TASK4	4

#define WAIT_PRESS_THE_BUTTON  0
#define WAIT_NO_PRESS_THE_BUTTON  1
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_RTC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t mstep=0;
uint8_t Seven_seg_scan = 0;
uint16_t Seven_segment_scan_timer = 0;
uint16_t LED_flicker_timer = 0;
uint16_t PWM_Counter = 0;
uint8_t Key_sw(uint8_t index);
uint8_t Key_sw_state = 0;
uint8_t ADC_index = 0;
uint16_t ADC_Value[10] = {0};
float Variable_resistance_ADC_Voltage_Value = 0.0;
uint8_t RTC_Second,RTC_hour,RTC_minute;
uint8_t RTC_WeekDay,RTC_Date,RTC_Month,RTC_Year;


uint8_t Seven_segment_display_decoding[16]={//common vcc decode table a~g pin no p pin
	0x01,0x4f,0x12,0x06,0x4c,0x24,
	0x60,0x0f,0x00,0x0c,0x72,0x66,
	0x5c,0x34,0x70,0xff
};

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Seven_segment_Display(uint8_t scan,uint8_t data);
void Seven_segment_scan(uint8_t scan);
void Seven_segment_data(uint8_t data);
static void RTC_TimeShow(void);	

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(__HAL_ADC_GET_FLAG(hadc,ADC_IT_EOC)){
		ADC_Value[ADC_index] = HAL_ADC_GetValue(hadc);
		ADC_index++;
	}
	if(__HAL_ADC_GET_FLAG(hadc,ADC_IT_EOS)){
		ADC_index = 0;
		Variable_resistance_ADC_Voltage_Value = (((float)ADC_Value[0]*3.3)/4096);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
		LED_flicker_timer = (LED_flicker_timer>0)?LED_flicker_timer - 1:0;
		Seven_segment_scan_timer = (Seven_segment_scan_timer>0)?Seven_segment_scan_timer - 1:0;
		
		Key_sw_state = Key_sw(Key_sw_state);
		
		if(Key_sw_state == 2){
			mstep = (mstep >= TASK4)? TASK1 : mstep + 1;
			Key_sw_state = 0;
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
	uint8_t PWM_Flag = 1;
	RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
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
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_ADC_Start_IT(&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(LED_flicker_timer == 0){//25ms
			LED_flicker_timer = 24;
			if(PWM_Flag){
				PWM_Flag = (PWM_Counter > 999)?0:1;
				PWM_Counter = (PWM_Counter > 1000)?0:PWM_Counter + 5;
			}
			else{
				PWM_Flag = (PWM_Counter > 0)?0:1;
				PWM_Counter = (PWM_Counter > 0)?PWM_Counter - 5:0;
			}
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,PWM_Counter);
		}
		switch(mstep){
			case INITIAL:	// Set RTC time initial
				sTime.Hours = 0x15;
				sTime.Minutes = 0x20;
				sTime.Seconds = 0x0;
				sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
				sTime.StoreOperation = RTC_STOREOPERATION_RESET;
				if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
				{
					_Error_Handler(__FILE__, __LINE__);
				}

				sDate.WeekDay = RTC_WEEKDAY_MONDAY;
				sDate.Month = RTC_MONTH_JULY;
				sDate.Date = 0x23;
				sDate.Year = 0x0;

				if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
				{
					_Error_Handler(__FILE__, __LINE__);
				}
				mstep = TASK1;
				break;
			case TASK1:	//Display PWM Value is zero to one thousand
				if(Seven_segment_scan_timer == 0){
					Seven_segment_scan_timer = 1;
					switch(Seven_seg_scan){
						case 0:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[PWM_Counter%10]);
							break;
						case 1:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(PWM_Counter/10)%10]);
							break;
						case 2:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(PWM_Counter/100)%10]);
							break;
						case 3:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(PWM_Counter/1000)%10]);
							break;
						default:
							Seven_seg_scan = 0;
							break;
					}
					Seven_seg_scan = (Seven_seg_scan >= 3)?0:Seven_seg_scan + 1;
				}
				break;
			case TASK2:	//scan Seven_segment from one to three
				if(Seven_segment_scan_timer == 0){
					Seven_segment_scan_timer = 1000;
					switch(Seven_seg_scan){
						case 0:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[0]);
							break;
						case 1:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[1]);
							break;
						case 2:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[2]);
							break;
						case 3:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[3]);
							break;
						default:
							Seven_seg_scan = 0;
							break;
					}
					Seven_seg_scan = (Seven_seg_scan >= 3)?0:Seven_seg_scan + 1;
				}
				break;
			case TASK3:		//Variable resistance Voltage Value
				HAL_ADC_Start_IT(&hadc);
				if(Seven_segment_scan_timer == 0){
					Seven_segment_scan_timer = 1;
					switch(Seven_seg_scan){
						case 0:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(uint16_t)(Variable_resistance_ADC_Voltage_Value*1000)%10]);
							break;
						case 1:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(uint16_t)((Variable_resistance_ADC_Voltage_Value*1000)/10)%10]);
							break;
						case 2:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(uint16_t)((Variable_resistance_ADC_Voltage_Value*1000)/100)%10]);
							break;
						case 3:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(uint16_t)((Variable_resistance_ADC_Voltage_Value*1000)/1000)%10]);
							break;
						default:
							Seven_seg_scan = 0;
							break;
					}
					Seven_seg_scan = (Seven_seg_scan >= 3)?0:Seven_seg_scan + 1;
				}
				break;
			case TASK4:		//RTC
				RTC_TimeShow();
				if(Seven_segment_scan_timer == 0){
					Seven_segment_scan_timer = 1;
					switch(Seven_seg_scan){
						case 0:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[RTC_minute%10]);
							break;
						case 1:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(RTC_minute/10)%10]);
							break;
						case 2:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[RTC_hour%10]);
							break;
						case 3:
							Seven_segment_Display(Seven_seg_scan,~Seven_segment_display_decoding[(RTC_hour/10)%10]);
							break;
						default:
							Seven_seg_scan = 0;
							break;
					}
					Seven_seg_scan = (Seven_seg_scan >= 3)?0:Seven_seg_scan + 1;
				}
				break;
			default:
				break;
		}

	
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Configure LSE Drive Capability 
    */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Enable the TimeStamp 
    */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 32;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_scan0_Pin|LED_scan1_Pin|LED_scan2_Pin|LED_scan3_Pin 
                          |LED_port_A_Pin|LED_port_B_Pin|LED_port_C_Pin|LED_port_D_Pin 
                          |LED_port_E_Pin|LED_port_F_Pin|LED_port_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_scan0_Pin LED_scan1_Pin LED_scan2_Pin LED_scan3_Pin 
                           LED_port_A_Pin LED_port_B_Pin LED_port_C_Pin LED_port_D_Pin 
                           LED_port_E_Pin LED_port_F_Pin LED_port_G_Pin */
  GPIO_InitStruct.Pin = LED_scan0_Pin|LED_scan1_Pin|LED_scan2_Pin|LED_scan3_Pin 
                          |LED_port_A_Pin|LED_port_B_Pin|LED_port_C_Pin|LED_port_D_Pin 
                          |LED_port_E_Pin|LED_port_F_Pin|LED_port_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : key_sw_Pin key_clk_Pin key_dt_Pin */
  GPIO_InitStruct.Pin = key_sw_Pin|key_clk_Pin|key_dt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Seven_segment_Display(uint8_t scan,uint8_t data){
		Seven_segment_scan(SEVEN_SEGMENT_BLACK);
		Seven_segment_data(data);
		Seven_segment_scan(scan);
}
void Seven_segment_scan(uint8_t scan){
	
	switch(scan){
		case SEVEN_SEGMENT_SCAN0:
			/*HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_RESET);*/
		
			HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_SET);
			break;
		case SEVEN_SEGMENT_SCAN1:
			/*HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_RESET);*/
		
			HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_SET);
			break;
		case SEVEN_SEGMENT_SCAN2:
		/*	HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_RESET);*/
		
			HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_SET);
			break;
		case SEVEN_SEGMENT_SCAN3:
		/*	HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_SET);*/
		
			HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_RESET);
			break;
		default://SEVEN_SEGMENT_BLACK
			HAL_GPIO_WritePin(LED_scan0_GPIO_Port,LED_scan0_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan1_GPIO_Port,LED_scan1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan2_GPIO_Port,LED_scan2_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_scan3_GPIO_Port,LED_scan3_Pin,GPIO_PIN_SET);
			break;
	}
}
void Seven_segment_data(uint8_t data){
	if(data & 0x40)
		HAL_GPIO_WritePin(LED_port_A_GPIO_Port,LED_port_A_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LED_port_A_GPIO_Port,LED_port_A_Pin,GPIO_PIN_RESET);
	
	if(data & 0x20)
		HAL_GPIO_WritePin(LED_port_B_GPIO_Port,LED_port_B_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LED_port_B_GPIO_Port,LED_port_B_Pin,GPIO_PIN_RESET);
	
	if(data & 0x10)
		HAL_GPIO_WritePin(LED_port_C_GPIO_Port,LED_port_C_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LED_port_C_GPIO_Port,LED_port_C_Pin,GPIO_PIN_RESET);
	
	if(data & 0x08)
		HAL_GPIO_WritePin(LED_port_D_GPIO_Port,LED_port_D_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LED_port_D_GPIO_Port,LED_port_D_Pin,GPIO_PIN_RESET);
	
	if(data & 0x04)
		HAL_GPIO_WritePin(LED_port_E_GPIO_Port,LED_port_E_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LED_port_E_GPIO_Port,LED_port_E_Pin,GPIO_PIN_RESET);
	
	if(data & 0x02)
		HAL_GPIO_WritePin(LED_port_F_GPIO_Port,LED_port_F_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LED_port_F_GPIO_Port,LED_port_F_Pin,GPIO_PIN_RESET);
	
	if(data & 0x01)
		HAL_GPIO_WritePin(LED_port_G_GPIO_Port,LED_port_G_Pin,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LED_port_G_GPIO_Port,LED_port_G_Pin,GPIO_PIN_RESET);
}
static void RTC_TimeShow(void)
{
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructureget;
  
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
  /* Display time Format : hh:mm:ss */
  //sprintf((char*)showtime,"%02d:%02d:%02d",stimestructureget.Hours, stimestructureget.Minutes, stimestructureget.Seconds);
	RTC_WeekDay = sdatestructureget.WeekDay;
	RTC_Date = sdatestructureget.Date;
	RTC_Month = sdatestructureget.Month;
	RTC_Year = sdatestructureget.Year;
	
	RTC_Second = stimestructureget.Seconds;
	RTC_minute = stimestructureget.Minutes;
	RTC_hour = stimestructureget.Hours;
}
uint8_t Key_sw(uint8_t index){
	switch(index){
		case WAIT_PRESS_THE_BUTTON:
			if(!(HAL_GPIO_ReadPin(key_sw_GPIO_Port,key_sw_Pin))){
				index = WAIT_NO_PRESS_THE_BUTTON;
			}
			break;
		case WAIT_NO_PRESS_THE_BUTTON:
			if(HAL_GPIO_ReadPin(key_sw_GPIO_Port,key_sw_Pin)){
				index = 2;
			}
			break;
		default:
				break;
	}
	return index;
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
