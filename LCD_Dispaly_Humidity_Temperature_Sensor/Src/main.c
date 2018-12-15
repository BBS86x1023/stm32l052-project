
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
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct{
	uint8_t temparature;
	uint8_t humidity;
}DH11_Dev;


uint8_t Message[][32]={
				"Hello World My name is XXX",
				"Humidity=",
				"Temperature=",
};

uint16_t timer6 = 0;
int8_t Is_it_wrong;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM21_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void delay_us(uint16_t _1us);
void LCD_initial(void);
void LCD_Write_instruction(uint8_t data);
void LCD_Write_Data(uint8_t data);
void LCD_Write_instruction_8Bits(uint8_t data);
void LCD_Write_Data_8Bits(uint8_t data);
void LCD_Clear_Display(void);
void LCD_Return_Home(void);
void LCD_Entry_Mode_Set(uint8_t I_D,uint8_t S);
void LCD_Display_ON_OFF_Control(uint8_t D,uint8_t C,uint8_t B);
void LCD_Cursor_or_Display_Shift(uint8_t S_C,uint8_t R_L);
void LCD_Function_Configure(uint8_t DL,uint8_t N,uint8_t F);
void LCD_Set_CGRAM_Address(uint8_t AC_5bits);
void LCD_Set_DDRAM_Address(uint8_t AC_6bits);
void LCD_String_Display(uint8_t a,uint8_t b);

int8_t DH11_Read(DH11_Dev* dev);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM6){
		timer6 = (timer6 > 0)?timer6 - 1:0;
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
	uint8_t i = 0;
	uint16_t PWM_Value = 800;
	DH11_Dev DH11;
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
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
	LCD_initial();
	
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start(&htim21);
	HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,PWM_Value);
	LCD_String_Display(0x00,0);
	LCD_Write_instruction_8Bits(0x80|20);
	for(i=0;i<strlen("Humidity=");i++)
		LCD_Write_Data_8Bits(Message[1][i]);
	LCD_Write_instruction_8Bits(0xC0|20);
	for(i=0;i<strlen("Temperature=");i++)
		LCD_Write_Data_8Bits(Message[2][i]);
	//HAL_Delay(500);;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(timer6 == 0){
			timer6 = 499;
			Is_it_wrong = DH11_Read(&DH11);
			LCD_Write_instruction_8Bits(0x80|29);
			LCD_Write_Data_8Bits(((DH11.humidity/10)%10)+ 0x30);
			LCD_Write_Data_8Bits((DH11.humidity%10)+ 0x30);
			LCD_Write_instruction_8Bits(0xC0|32);
			LCD_Write_Data_8Bits(((DH11.temparature/10)%10)+ 0x30);
			LCD_Write_Data_8Bits((DH11.temparature%10)+ 0x30);
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32;
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

/* TIM21 init function */
static void MX_TIM21_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 32;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DH11_signal_GPIO_Port, DH11_signal_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin 
                           DH11_signal_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin 
                          |DH11_signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_EN_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void LCD_initial(void){
	LCD_Function_Configure(0,1,0);
	LCD_Display_ON_OFF_Control(1,1,1);	// Display on, cursor on
	LCD_Clear_Display();
	LCD_Entry_Mode_Set(1,0);
	HAL_Delay(100);//delay 100ms
}

void LCD_Clear_Display(void){
	LCD_Write_instruction_8Bits(0x01);
}
void LCD_Return_Home(void){
	LCD_Write_instruction_8Bits(0x02);
}
void LCD_Entry_Mode_Set(uint8_t I_D,uint8_t S){
	LCD_Write_instruction_8Bits(0x04|((I_D&0x01)<<1)|(S&0x01));
}
void LCD_Display_ON_OFF_Control(uint8_t D,uint8_t C,uint8_t B){
	LCD_Write_instruction_8Bits(0x08|((D&0x01)<<2)|((C&0x01)<<1)|(B&0x01));
}

void LCD_Cursor_or_Display_Shift(uint8_t S_C,uint8_t R_L){
	LCD_Write_instruction_8Bits(0x10|((S_C&0x01)<<3)|((R_L&0x01)<<2));
}
void LCD_Function_Configure(uint8_t DL,uint8_t N,uint8_t F){
	LCD_Write_instruction_8Bits(0x20|((DL&0x01)<<4)|((N&0x01)<<3)|((F&0x01)<<2));
}
void LCD_Set_CGRAM_Address(uint8_t AC_5bits){
	LCD_Write_instruction_8Bits(0x40|(AC_5bits&0x3F));
}
void LCD_Set_DDRAM_Address(uint8_t AC_6bits){
	LCD_Write_instruction_8Bits(0x80|(AC_6bits&0x7F));
}


void LCD_Write_instruction_8Bits(uint8_t data){
	LCD_Write_instruction((data&0xF0)>>4);
	LCD_Write_instruction(data&0x0F);
}
void LCD_Write_Data_8Bits(uint8_t data){
	LCD_Write_Data((data&0xF0)>>4);
	LCD_Write_Data(data&0x0F);
}

void LCD_Write_instruction(uint8_t data){
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,(data&0x08));
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,(data&0x04));
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,(data&0x02));
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,(data&0x01));
	delay_us(60);
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_RESET);
	delay_us(60);
	
}
void LCD_Write_Data(uint8_t data){
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port,LCD_RS_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_D7_GPIO_Port,LCD_D7_Pin,(data&0x08));
	HAL_GPIO_WritePin(LCD_D6_GPIO_Port,LCD_D6_Pin,(data&0x04));
	HAL_GPIO_WritePin(LCD_D5_GPIO_Port,LCD_D5_Pin,(data&0x02));
	HAL_GPIO_WritePin(LCD_D4_GPIO_Port,LCD_D4_Pin,(data&0x01));
	delay_us(60);
	HAL_GPIO_WritePin(LCD_EN_GPIO_Port,LCD_EN_Pin,GPIO_PIN_RESET);
	delay_us(60);
}
void LCD_String_Display(uint8_t a,uint8_t b){
	unsigned char i=0;

	LCD_Write_instruction_8Bits(0x80|a);

	while(Message[b][i] != 0){
		if(i == 20)
			LCD_Write_instruction_8Bits(0xc0);
		LCD_Write_Data_8Bits(Message[b][i]);
		i++;
	}
}

int8_t DH11_Read(DH11_Dev* dev){
	uint8_t i, j, temp;
	uint8_t data[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin = DH11_signal_Pin;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	HAL_GPIO_WritePin(DH11_signal_GPIO_Port,DH11_signal_Pin,GPIO_PIN_RESET);
	
	__HAL_TIM_SET_COUNTER(&htim21,0);
	while((__HAL_TIM_GET_COUNTER(&htim21)) <= 18000);
	
	HAL_GPIO_WritePin(DH11_signal_GPIO_Port,DH11_signal_Pin,GPIO_PIN_SET);
	
	__HAL_TIM_SET_COUNTER(&htim21,0);
	while((__HAL_TIM_GET_COUNTER(&htim21)) <= 40);
	
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	//DHT11 ACK
	//should be LOW for at least 80us
	//while(!GPIO_ReadInputDataBit(dev->port, dev->pin));
	__HAL_TIM_SET_COUNTER(&htim21,0);
	while(!(HAL_GPIO_ReadPin(DH11_signal_GPIO_Port,DH11_signal_Pin))){
		if((__HAL_TIM_GET_COUNTER(&htim21)) > 100)
			return -1;
	}
	__HAL_TIM_SET_COUNTER(&htim21,0);
	while(HAL_GPIO_ReadPin(DH11_signal_GPIO_Port,DH11_signal_Pin)){
		if((__HAL_TIM_GET_COUNTER(&htim21)) > 100)
			return -1;
	}
	//Read 40 bits (8*5)
	
	for(j=0;j<5;++j){
		for(i=0;i<8;++i){
			//LOW for 50us
			while(!(HAL_GPIO_ReadPin(DH11_signal_GPIO_Port,DH11_signal_Pin)));
			//Start counter
			__HAL_TIM_SET_COUNTER(&htim21,0);
			//HIGH for 26-28us = 0 / 70us = 1
			while(HAL_GPIO_ReadPin(DH11_signal_GPIO_Port,DH11_signal_Pin));
			
			temp = __HAL_TIM_GET_COUNTER(&htim21);
			data[j] = data[j] << 1;
			if(temp > 40)
				data[j] = data[j] + 1;
			
		}
	}
	
	if(data[4] != (data[0] + data[2]))
		return -2;
	
	dev->temparature = data[2];
	dev->humidity = data[0];
	
	return 1;
}

void delay_us(uint16_t _1us){
	uint16_t i,j;
	for(i=0;i<_1us;i++)
		for(j=0;j<8;j++);
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
