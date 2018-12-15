
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
#include "math.h"
#define div 4		//除 4
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM21_Init(void);
static void MX_ADC_Init(void);
static void MX_DAC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint8_t Task = 0;  //項目(任務)
uint8_t Time_Flag = 0;		//時間旗標
uint16_t Past_Time = 0;   //過去時間
uint16_t Present_Time = 0;//現在時間
uint16_t Timer_clock = 0;	

/*--------------------數位轉類比(D/A)函式及變數------------------------------------*/
uint8_t Mode = 1;

const uint16_t Wave_forms_Table[][120] = 		//數位轉類比查表
{  
		{   // Sin wave
				0x7ff /div, 0x86a /div, 0x8d5 /div, 0x93f /div, 0x9a9 /div, 0xa11 /div, 0xa78 /div, 0xadd /div, 0xb40 /div, 0xba1 /div,   
				0xbff /div, 0xc5a /div, 0xcb2 /div, 0xd08 /div, 0xd59 /div, 0xda7 /div, 0xdf1 /div, 0xe36 /div, 0xe77 /div, 0xeb4 /div,
				0xeec /div, 0xf1f /div, 0xf4d /div, 0xf77 /div, 0xf9a /div, 0xfb9 /div, 0xfd2 /div, 0xfe5 /div, 0xff3 /div, 0xffc /div,
				0xfff /div, 0xffc /div, 0xff3 /div, 0xfe5 /div, 0xfd2 /div, 0xfb9 /div, 0xf9a /div, 0xf77 /div, 0xf4d /div, 0xf1f /div,
				0xeec /div, 0xeb4 /div, 0xe77 /div, 0xe36 /div, 0xdf1 /div, 0xda7 /div, 0xd59 /div, 0xd08 /div, 0xcb2 /div, 0xc5a /div,
				0xbff /div, 0xba1 /div, 0xb40 /div, 0xadd /div, 0xa78 /div, 0xa11 /div, 0x9a9 /div, 0x93f /div, 0x8d5 /div, 0x86a /div,
				0x7ff /div, 0x794 /div, 0x729 /div, 0x6bf /div, 0x655 /div, 0x5ed /div, 0x586 /div, 0x521 /div, 0x4be /div, 0x45d /div,
				0x3ff /div, 0x3a4 /div, 0x34c /div, 0x2f6 /div, 0x2a5 /div, 0x257 /div, 0x20d /div, 0x1c8 /div, 0x187 /div, 0x14a /div,
				0x112 /div, 0x0df /div, 0x0b1 /div, 0x087 /div, 0x064 /div, 0x045 /div, 0x02c /div, 0x019 /div, 0x00b /div, 0x002 /div,
				0x000 /div, 0x002 /div, 0x00b /div, 0x019 /div, 0x02c /div, 0x045 /div, 0x064 /div, 0x087 /div, 0x0b1 /div, 0x0df /div,
				0x112 /div, 0x14a /div, 0x187 /div, 0x1c8 /div, 0x20d /div, 0x257 /div, 0x2a5 /div, 0x2f6 /div, 0x34c /div, 0x3a4 /div,
				0x3ff /div, 0x45d /div, 0x4be /div, 0x521 /div, 0x586 /div, 0x5ed /div, 0x655 /div, 0x6bf /div, 0x729 /div, 0x794 /div
		},
		{   // Triangular wave
				0x044 /div, 0x088 /div, 0x0cc /div, 0x110 /div, 0x154 /div, 0x198 /div, 0x1dc /div, 0x220 /div, 0x264 /div, 0x2a8 /div,   
				0x2ec /div, 0x330 /div, 0x374 /div, 0x3b8 /div, 0x3fc /div, 0x440 /div, 0x484 /div, 0x4c8 /div, 0x50c /div, 0x550 /div,
				0x594 /div, 0x5d8 /div, 0x61c /div, 0x660 /div, 0x6a4 /div, 0x6e8 /div, 0x72c /div, 0x770 /div, 0x7b4 /div, 0x7f8 /div,
				0x83c /div, 0x880 /div, 0x8c4 /div, 0x908 /div, 0x94c /div, 0x990 /div, 0x9d4 /div, 0xa18 /div, 0xa5c /div, 0xaa0 /div,
				0xae4 /div, 0xb28 /div, 0xb6c /div, 0xbb0 /div, 0xbf4 /div, 0xc38 /div, 0xc7c /div, 0xcc0 /div, 0xd04 /div, 0xd48 /div,
				0xd8c /div, 0xdd0 /div, 0xe14 /div, 0xe58 /div, 0xe9c /div, 0xee0 /div, 0xf24 /div, 0xf68 /div, 0xfac /div, 0xff0 /div,
				0xfac /div, 0xf68 /div, 0xf24 /div, 0xee0 /div, 0xe9c /div, 0xe58 /div, 0xe14 /div, 0xdd0 /div, 0xd8c /div, 0xd48 /div,
				0xd04 /div, 0xcc0 /div, 0xc7c /div, 0xc38 /div, 0xbf4 /div, 0xbb0 /div, 0xb6c /div, 0xb28 /div, 0xae4 /div, 0xaa0 /div,
				0xa5c /div, 0xa18 /div, 0x9d4 /div, 0x990 /div, 0x94c /div, 0x908 /div, 0x8c4 /div, 0x880 /div, 0x83c /div, 0x7f8 /div,
				0x7b4 /div, 0x770 /div, 0x72c /div, 0x6e8 /div, 0x6a4 /div, 0x660 /div, 0x61c /div, 0x5d8 /div, 0x594 /div, 0x550 /div,
				0x50c /div, 0x4c8 /div, 0x484 /div, 0x440 /div, 0x3fc /div, 0x3b8 /div, 0x374 /div, 0x330 /div, 0x2ec /div, 0x2a8 /div,
				0x264 /div, 0x220 /div, 0x1dc /div, 0x198 /div, 0x154 /div, 0x110 /div, 0x0cc /div, 0x088 /div, 0x044 /div, 0x000 /div
		},  
		{   // Square wave
				0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div,   
				0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div,
				0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div,
				0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div,
				0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div,
				0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div, 0xfff /div,
				0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div,
				0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div,
				0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div,
				0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div,
				0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div,
				0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div, 0x000 /div
	  }
}; 

const uint8_t DAC_Mode[3][3]=	 //DAC的模式顯示
{
	{0x71,0xfd,0x9f},		//F-1			//DAC_Mode[2][1]
	{0x71,0xfd,0x25},		//F-2
	{0x71,0xfd,0x0d}		//F-3
};

/*---------------------------------------------------------------------------------*/

/*----------------------溫度&類比轉數位(A/D)函式及變數-----------------------------*/
uint8_t ADC_inndex = 0; 						//類比轉數位 旗標
uint16_t ADC_Buffer[3] = {0,0,0};		//類比轉數位 暫存器

double R18;			//R18電阻值
double Temperature(void);		//溫度

/*----------------------------------------------------------------------------*/
/*----------------------開關函式----------------------------------------------*/
uint8_t S1_state = 0;				//S1按鈕的狀態
uint8_t S2_state = 0;				//S2按鈕的狀態
uint8_t S1_push_function(uint8_t state);		//S1 按鈕函式
uint8_t S2_push_function(uint8_t state);		//S2 按鈕函式


/*----------------------------------------------------------------------------*/

/*-------------------- 七段函式-----------------------------------------------*/
uint16_t Seven_segment_clock = 0; 				//七段上數計數器變數
uint8_t Seven_segment_Brightness = 2;     //七段顯示的亮度 (0~4)
uint8_t Seven_segment_Bit = 0;						//七段位元
uint8_t Seven_segment_Bits[4] = {0,0,0,0};	//七段掃描顯示位元
void Seven_segment_display(uint8_t scan);	//七段顯示掃描
void Seven_segment_display_Shun_Down(void);//七段顯示黑屏
void Seven_segment_Get_Data(uint8_t data); //丟資料給七段

const uint8_t Seven_segment_decode[10] = {//共陽 七段解碼
		0x03,0x9f,0x25,0x0d,0x99,0x49,
	  0xc1,0x1f,0x01,0x19
};
/*-----------------------------------------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(__HAL_ADC_GET_FLAG(hadc,ADC_FLAG_EOC)){
		ADC_Buffer[ADC_inndex] = HAL_ADC_GetValue(hadc);		//讀類比電壓值
		ADC_inndex++;  //ADC旗標加一
	}
	if(__HAL_ADC_GET_FLAG(hadc,ADC_FLAG_EOS)){
		ADC_inndex = 0;		//ADC 旗標清除
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//時間溢位時產生中斷發生，而來到TIM_PeriodElapsedCallback函式執行
	if(htim->Instance == TIM21){		//判斷TIM21是否中斷 , per 0.5ms interrupt equal per 2000Hz interrupt
		
		Timer_clock = (Timer_clock > 0)?Timer_clock - 1:0;
		
		/*--------------------- 七段掃描調整亮度  PWM = 0% 25% 50% 75% 100%    頻率100Hz------------------------------------------------------------------------------------*/
		if(Task != 0){
  		//掃描位元
			Seven_segment_Bit = (Seven_segment_Bit > 3)?0:(Seven_segment_clock > 4)?Seven_segment_Bit + 1:Seven_segment_Bit;
			
			if(Seven_segment_clock < Seven_segment_Brightness){
				Seven_segment_display(Seven_segment_Bit);	//顯示現在的位元
			}
			else{
				Seven_segment_display_Shun_Down();	//顯示黑屏
			}
			
			Seven_segment_clock = (Seven_segment_clock > 4)?0:Seven_segment_clock + 1;  //上數從 0 數到 4 
		}
		/*----------------	---------------------------------------------------------------------------------------------------------------------------------------------------*/
		
		
		/*-----------------------按鈕開關掃描偵測---------------------------------------------------------------*/
		S1_state = S1_push_function(S1_state);		//按下去S1_state = 1, 放開S1_state = 2
		S2_state = S2_push_function(S2_state);		//按下去S2_state = 1, 放開S2_state = 2
		if(S1_state == 2){	//當S1按鈕放開時執行下面的程式
			Seven_segment_Bits[3] = 0xff;		//七段第四格位元丟資料為0xff
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PC0 pin 為高電壓
			Timer_clock = 2000;				//給時間時脈為2000,給主程式case 3 和 case 4用,讓七段顯示能維持一秒
			Task = 4;  	//任務 四 ,主程式裡狀態表示
			Mode = (Mode <= 1)?3:Mode - 1;		//DAC 模式顯示在七段的值
			S1_state = 0;	//S1狀態清除為零
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(Wave_forms_Table + (Mode - 1)), 120, DAC_ALIGN_12B_R);	//DAC輸出波型查表
		}
		if(S2_state == 2){  //當S2按鈕放開時執行下面的程式
			Seven_segment_Bits[3] = 0xff;		//七段第四格位元丟資料為0xff
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET); 	//PC0 pin 為高電壓
			Timer_clock = 2000;		//給時間時脈為2000,給主程式case 3 和 case 4用,讓七段顯示能維持一秒
			Task = 3;			//任務 三 ,主程式裡狀態表示
			Mode = (Mode >= 3)?1:Mode + 1; //DAC 模式顯示在七段的值
			S2_state = 0;			//S2狀態清除為零
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(Wave_forms_Table + (Mode - 1)), 120, DAC_ALIGN_12B_R);  //DAC輸出波型查表
		}
		
		/*--------------------------------------------------------------------------------------------------*/
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
	uint8_t Time_Clock=0;     //時間計數器
	uint8_t Number = 9; //數字
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
  MX_TIM21_Init();
  MX_ADC_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim21);			//TIM21 中斷開始
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);		//PC7 為低電位
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);			////PC6 為高電位
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(Wave_forms_Table + (Mode - 1)), 120, DAC_ALIGN_12B_R);		//DAC輸出波型
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			switch(Task){  	//判斷執行的任務
				case 0: 			//LED 相對閃爍 3 次
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);	//PC11 為低電位
					if(Time_Flag){  //初始值 Time_Flag = 0;
						Present_Time = HAL_GetTick();		//儲存現在的時間
						if(Present_Time - Past_Time >250){  // delay 250ms
							HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
							HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
							Time_Flag = 0;        //時間旗標
							if(Time_Clock >= 6){
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
								Task = 1;						//任務為1
								Number = 9;					//數字9
								Time_Clock = 0;			//時間計數器
								Time_Flag = 0;			//清除Time旗標
								Present_Time = 0;		//清除現在時間
								Past_Time = 0;			//清除過去時間
							}else{
								Time_Clock = Time_Clock + 1;		
							}
						}
					}
					else{
						Past_Time = HAL_GetTick();		//儲存過去的時間
						Time_Flag = 1;	//Time 旗標
					}
					break;
				case 1:   		//七段倒數 9 ~ 0
					if(Time_Flag){  //初始值 Time_Flag = 0;
						Present_Time = HAL_GetTick();  //儲存現在的時間
						if(Present_Time - Past_Time >500){  // delay 500ms
							Seven_segment_Bits[2] = Seven_segment_decode[Number];		//七段顯示第二位 Number為丟出來的數字
							Seven_segment_Bits[1] = Seven_segment_decode[Number];		//七段顯示第一位 Number為丟出來的數字
							Seven_segment_Bits[0] = Seven_segment_decode[Number];		//七段顯示第零位 Number為丟出來的數字
							Time_Flag = 0;        //時間旗標
							if(Number > 0){
								Number = Number - 1;	//七段數值減一
							}else{
								Number = 0;						//Number 歸零
								Task = 2;							//任務 2
								Time_Flag = 0;        //時間旗標清除
								Present_Time = 0;			//清除現在時間
								Past_Time = 0;				//清除過去時間
							}
						}
					}
					else{
						Past_Time = HAL_GetTick();   //儲存過去的時間
						Time_Flag = 1;		//Time 旗標
					}
					break;
				case 2: // 顯示現在的室溫
					if(Temperature() >= 10 && Temperature() <= 35){		//判斷溫度是否在 10~35之間
							if(Seven_segment_Bit == 3){			//LED顯示判斷溫度是否超標
								Seven_segment_Bits[3] = 0x7f;	//溫度範圍在10~35之間亮綠燈
							}
					}
					else{
						if(Seven_segment_Bit == 3){
							Seven_segment_Bits[3] = 0xbf; //溫度範圍不在10~35之間亮紅燈
						}
					}
					
					if(Time_Flag){  //初始值 Time_Flag = 0;
						Present_Time = HAL_GetTick();			//儲存現在的時間
						if(Present_Time - Past_Time >1000){		//delay 1 s
							Seven_segment_Bits[2] = Seven_segment_decode[((uint8_t)Temperature())/10];		//溫度的十位數
							Seven_segment_Bits[1] = Seven_segment_decode[((uint8_t)Temperature())%10];		//溫度的個位數
							Seven_segment_Bits[0] = Seven_segment_decode[((uint8_t)(Temperature()*10))%10]; ////溫度的小數第一位數
							Time_Flag = 0;        //時間旗標清除
						}
					}
					else{
						Past_Time = HAL_GetTick();   //儲存過去的時間
						Time_Flag = 1;			//Time 旗標
					}
					break;
				case 3:			//變換DAC的波型 從 F-1 ~ F-3  維持1秒
					Seven_segment_Bits[2] = DAC_Mode[Mode-1][0];		// F
					Seven_segment_Bits[1] = DAC_Mode[Mode-1][1];		// -
					Seven_segment_Bits[0] = DAC_Mode[Mode-1][2];		// x - 數字
				
					if(Timer_clock == 0){		//維持1秒
						Task = 2;				//任務回到2
						Seven_segment_Bits[2] = Seven_segment_decode[((uint8_t)Temperature())/10];		//溫度的十位數
						Seven_segment_Bits[1] = Seven_segment_decode[((uint8_t)Temperature())%10];		//溫度的個位數
						Seven_segment_Bits[0] = Seven_segment_decode[((uint8_t)(Temperature()*10))%10]; ////溫度的小數第一位數
					}
					break;
				case 4:     //變換DAC的波型 從 F-3 ~ F-1  維持1秒
					Seven_segment_Bits[2] = DAC_Mode[Mode-1][0];		// F
					Seven_segment_Bits[1] = DAC_Mode[Mode-1][1];		// -
					Seven_segment_Bits[0] = DAC_Mode[Mode-1][2];		// x - 數字
					if(Timer_clock == 0){		//維持1秒
						Task = 2;				//任務回到2
						Seven_segment_Bits[2] = Seven_segment_decode[((uint8_t)Temperature())/10];		//溫度的十位數
						Seven_segment_Bits[1] = Seven_segment_decode[((uint8_t)Temperature())%10];		//溫度的個位數
						Seven_segment_Bits[0] = Seven_segment_decode[((uint8_t)(Temperature()*10))%10]; ////溫度的小數第一位數
					}		
					break;
				default:
					Task = 0;
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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
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
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;
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

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_EXT_IT9;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
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
  htim21.Init.Prescaler = 16;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 1000;
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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, dp_Pin|g_Pin|f_Pin|e_Pin 
                          |d_Pin|c_Pin|b_Pin|a_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, q4_Pin|q3_Pin|q2_Pin|q1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : dp_Pin g_Pin f_Pin e_Pin 
                           d_Pin c_Pin b_Pin a_Pin 
                           q4_Pin q3_Pin q2_Pin q1_Pin */
  GPIO_InitStruct.Pin = dp_Pin|g_Pin|f_Pin|e_Pin 
                          |d_Pin|c_Pin|b_Pin|a_Pin 
                          |q4_Pin|q3_Pin|q2_Pin|q1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S2_Pin S1_Pin */
  GPIO_InitStruct.Pin = S2_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void Seven_segment_display_Shun_Down(void){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
}
void Seven_segment_display(uint8_t scan){
	switch(scan){
		case 0: //個位數
			Seven_segment_Get_Data(Seven_segment_Bits[0]);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
			break;
		case 1: //十位數
			if(Task == 2)
				Seven_segment_Get_Data(Seven_segment_Bits[1] & 0xFE);			//顯示小數點
			else
				Seven_segment_Get_Data(Seven_segment_Bits[1]);						//不顯示小數點
			
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
			break;
		case 2: //百位數
			Seven_segment_Get_Data(Seven_segment_Bits[2]);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_11,GPIO_PIN_SET);
			break;
		case 3:
			if(Task != 0){
				Seven_segment_Get_Data(Seven_segment_Bits[3]);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10,GPIO_PIN_SET);
			}
			break;
		default:
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
			break;
	}
}
void Seven_segment_Get_Data(uint8_t data){		//丟資料給七段
	uint8_t i;
	for(i=0;i<=7;i++){
		if((data>>i) & 0x01)
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0 << i,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0 << i,GPIO_PIN_RESET);
	}
}

double Temperature(void){
	double Temperature;
	const float Vcc = 3.3;							//電壓Vcc
	const int     B = 3425;			//3380				//beta 放大係數
	const double T_25 = 273.15 + 25;		//常溫25度
	const double R1 = 10000.0;					//在25度時熱敏電阻為10K歐姆
	const double V1 = 0.1;							//OPA 正端輸入為0.1伏特
	HAL_ADC_Start_IT(&hadc);
	double AIN0 = (((double)ADC_Buffer[0]) * Vcc )/4096.0;
	R18 = (AIN0 - V1) / 0.00001;
	
	Temperature = ((T_25 * B) / (B + T_25 * log (R18 / R1))) - 273.15;
	
	return Temperature;
}

uint8_t S1_push_function(uint8_t state){
	switch(state){
		case 0:			//判斷按鈕是否按下
			if(HAL_GPIO_ReadPin(S1_GPIO_Port,S1_Pin)){
				state = 1;
			}
			break;
		case 1:			//判斷按鈕是否放開
			if(!(HAL_GPIO_ReadPin(S1_GPIO_Port,S1_Pin))){
				state = 2;
			}
			break;
		case 2: 		//確定按鈕被按下  備註:請手動把S1_state 清除為零;
			break;
	}
	return state;
}
uint8_t S2_push_function(uint8_t state){
	switch(state){
		case 0:			//判斷按鈕是否按下
			if(HAL_GPIO_ReadPin(S2_GPIO_Port,S2_Pin)){
				state = 1;
			}
			break;
		case 1:			//判斷按鈕是否放開
			if(!(HAL_GPIO_ReadPin(S2_GPIO_Port,S2_Pin))){
				state = 2;
			}
			break;
		case 2: 		//確定按鈕被按下  備註:請手動把S2_state 清除為零;
			break;
	}
	return state;
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
