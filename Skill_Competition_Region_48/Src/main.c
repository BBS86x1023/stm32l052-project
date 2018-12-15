
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
#define div 4		//�� 4
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

uint8_t Task = 0;  //����(����)
uint8_t Time_Flag = 0;		//�ɶ��X��
uint16_t Past_Time = 0;   //�L�h�ɶ�
uint16_t Present_Time = 0;//�{�b�ɶ�
uint16_t Timer_clock = 0;	

/*--------------------�Ʀ�������(D/A)�禡���ܼ�------------------------------------*/
uint8_t Mode = 1;

const uint16_t Wave_forms_Table[][120] = 		//�Ʀ�������d��
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

const uint8_t DAC_Mode[3][3]=	 //DAC���Ҧ����
{
	{0x71,0xfd,0x9f},		//F-1			//DAC_Mode[2][1]
	{0x71,0xfd,0x25},		//F-2
	{0x71,0xfd,0x0d}		//F-3
};

/*---------------------------------------------------------------------------------*/

/*----------------------�ū�&������Ʀ�(A/D)�禡���ܼ�-----------------------------*/
uint8_t ADC_inndex = 0; 						//������Ʀ� �X��
uint16_t ADC_Buffer[3] = {0,0,0};		//������Ʀ� �Ȧs��

double R18;			//R18�q����
double Temperature(void);		//�ū�

/*----------------------------------------------------------------------------*/
/*----------------------�}���禡----------------------------------------------*/
uint8_t S1_state = 0;				//S1���s�����A
uint8_t S2_state = 0;				//S2���s�����A
uint8_t S1_push_function(uint8_t state);		//S1 ���s�禡
uint8_t S2_push_function(uint8_t state);		//S2 ���s�禡


/*----------------------------------------------------------------------------*/

/*-------------------- �C�q�禡-----------------------------------------------*/
uint16_t Seven_segment_clock = 0; 				//�C�q�W�ƭp�ƾ��ܼ�
uint8_t Seven_segment_Brightness = 2;     //�C�q��ܪ��G�� (0~4)
uint8_t Seven_segment_Bit = 0;						//�C�q�줸
uint8_t Seven_segment_Bits[4] = {0,0,0,0};	//�C�q���y��ܦ줸
void Seven_segment_display(uint8_t scan);	//�C�q��ܱ��y
void Seven_segment_display_Shun_Down(void);//�C�q��ܶ«�
void Seven_segment_Get_Data(uint8_t data); //���Ƶ��C�q

const uint8_t Seven_segment_decode[10] = {//�@�� �C�q�ѽX
		0x03,0x9f,0x25,0x0d,0x99,0x49,
	  0xc1,0x1f,0x01,0x19
};
/*-----------------------------------------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(__HAL_ADC_GET_FLAG(hadc,ADC_FLAG_EOC)){
		ADC_Buffer[ADC_inndex] = HAL_ADC_GetValue(hadc);		//Ū����q����
		ADC_inndex++;  //ADC�X�Х[�@
	}
	if(__HAL_ADC_GET_FLAG(hadc,ADC_FLAG_EOS)){
		ADC_inndex = 0;		//ADC �X�вM��
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//�ɶ�����ɲ��ͤ��_�o�͡A�ӨӨ�TIM_PeriodElapsedCallback�禡����
	if(htim->Instance == TIM21){		//�P�_TIM21�O�_���_ , per 0.5ms interrupt equal per 2000Hz interrupt
		
		Timer_clock = (Timer_clock > 0)?Timer_clock - 1:0;
		
		/*--------------------- �C�q���y�վ�G��  PWM = 0% 25% 50% 75% 100%    �W�v100Hz------------------------------------------------------------------------------------*/
		if(Task != 0){
  		//���y�줸
			Seven_segment_Bit = (Seven_segment_Bit > 3)?0:(Seven_segment_clock > 4)?Seven_segment_Bit + 1:Seven_segment_Bit;
			
			if(Seven_segment_clock < Seven_segment_Brightness){
				Seven_segment_display(Seven_segment_Bit);	//��ܲ{�b���줸
			}
			else{
				Seven_segment_display_Shun_Down();	//��ܶ«�
			}
			
			Seven_segment_clock = (Seven_segment_clock > 4)?0:Seven_segment_clock + 1;  //�W�Ʊq 0 �ƨ� 4 
		}
		/*----------------	---------------------------------------------------------------------------------------------------------------------------------------------------*/
		
		
		/*-----------------------���s�}�����y����---------------------------------------------------------------*/
		S1_state = S1_push_function(S1_state);		//���U�hS1_state = 1, ��}S1_state = 2
		S2_state = S2_push_function(S2_state);		//���U�hS2_state = 1, ��}S2_state = 2
		if(S1_state == 2){	//��S1���s��}�ɰ���U�����{��
			Seven_segment_Bits[3] = 0xff;		//�C�q�ĥ|��줸���Ƭ�0xff
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);	//PC0 pin �����q��
			Timer_clock = 2000;				//���ɶ��ɯ߬�2000,���D�{��case 3 �M case 4��,���C�q��ܯ�����@��
			Task = 4;  	//���� �| ,�D�{���̪��A���
			Mode = (Mode <= 1)?3:Mode - 1;		//DAC �Ҧ���ܦb�C�q����
			S1_state = 0;	//S1���A�M�����s
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(Wave_forms_Table + (Mode - 1)), 120, DAC_ALIGN_12B_R);	//DAC��X�i���d��
		}
		if(S2_state == 2){  //��S2���s��}�ɰ���U�����{��
			Seven_segment_Bits[3] = 0xff;		//�C�q�ĥ|��줸���Ƭ�0xff
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET); 	//PC0 pin �����q��
			Timer_clock = 2000;		//���ɶ��ɯ߬�2000,���D�{��case 3 �M case 4��,���C�q��ܯ�����@��
			Task = 3;			//���� �T ,�D�{���̪��A���
			Mode = (Mode >= 3)?1:Mode + 1; //DAC �Ҧ���ܦb�C�q����
			S2_state = 0;			//S2���A�M�����s
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(Wave_forms_Table + (Mode - 1)), 120, DAC_ALIGN_12B_R);  //DAC��X�i���d��
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
	uint8_t Time_Clock=0;     //�ɶ��p�ƾ�
	uint8_t Number = 9; //�Ʀr
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
	HAL_TIM_Base_Start_IT(&htim21);			//TIM21 ���_�}�l
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);		//PC7 ���C�q��
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);			////PC6 �����q��
	HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)(Wave_forms_Table + (Mode - 1)), 120, DAC_ALIGN_12B_R);		//DAC��X�i��
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			switch(Task){  	//�P�_���檺����
				case 0: 			//LED �۹�{�{ 3 ��
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);	//PC11 ���C�q��
					if(Time_Flag){  //��l�� Time_Flag = 0;
						Present_Time = HAL_GetTick();		//�x�s�{�b���ɶ�
						if(Present_Time - Past_Time >250){  // delay 250ms
							HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_7);
							HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
							Time_Flag = 0;        //�ɶ��X��
							if(Time_Clock >= 6){
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
								Task = 1;						//���Ȭ�1
								Number = 9;					//�Ʀr9
								Time_Clock = 0;			//�ɶ��p�ƾ�
								Time_Flag = 0;			//�M��Time�X��
								Present_Time = 0;		//�M���{�b�ɶ�
								Past_Time = 0;			//�M���L�h�ɶ�
							}else{
								Time_Clock = Time_Clock + 1;		
							}
						}
					}
					else{
						Past_Time = HAL_GetTick();		//�x�s�L�h���ɶ�
						Time_Flag = 1;	//Time �X��
					}
					break;
				case 1:   		//�C�q�˼� 9 ~ 0
					if(Time_Flag){  //��l�� Time_Flag = 0;
						Present_Time = HAL_GetTick();  //�x�s�{�b���ɶ�
						if(Present_Time - Past_Time >500){  // delay 500ms
							Seven_segment_Bits[2] = Seven_segment_decode[Number];		//�C�q��ܲĤG�� Number����X�Ӫ��Ʀr
							Seven_segment_Bits[1] = Seven_segment_decode[Number];		//�C�q��ܲĤ@�� Number����X�Ӫ��Ʀr
							Seven_segment_Bits[0] = Seven_segment_decode[Number];		//�C�q��ܲĹs�� Number����X�Ӫ��Ʀr
							Time_Flag = 0;        //�ɶ��X��
							if(Number > 0){
								Number = Number - 1;	//�C�q�ƭȴ�@
							}else{
								Number = 0;						//Number �k�s
								Task = 2;							//���� 2
								Time_Flag = 0;        //�ɶ��X�вM��
								Present_Time = 0;			//�M���{�b�ɶ�
								Past_Time = 0;				//�M���L�h�ɶ�
							}
						}
					}
					else{
						Past_Time = HAL_GetTick();   //�x�s�L�h���ɶ�
						Time_Flag = 1;		//Time �X��
					}
					break;
				case 2: // ��ܲ{�b���Ƿ�
					if(Temperature() >= 10 && Temperature() <= 35){		//�P�_�ū׬O�_�b 10~35����
							if(Seven_segment_Bit == 3){			//LED��ܧP�_�ū׬O�_�W��
								Seven_segment_Bits[3] = 0x7f;	//�ū׽d��b10~35�����G��O
							}
					}
					else{
						if(Seven_segment_Bit == 3){
							Seven_segment_Bits[3] = 0xbf; //�ū׽d�򤣦b10~35�����G���O
						}
					}
					
					if(Time_Flag){  //��l�� Time_Flag = 0;
						Present_Time = HAL_GetTick();			//�x�s�{�b���ɶ�
						if(Present_Time - Past_Time >1000){		//delay 1 s
							Seven_segment_Bits[2] = Seven_segment_decode[((uint8_t)Temperature())/10];		//�ūת��Q���
							Seven_segment_Bits[1] = Seven_segment_decode[((uint8_t)Temperature())%10];		//�ūת��Ӧ��
							Seven_segment_Bits[0] = Seven_segment_decode[((uint8_t)(Temperature()*10))%10]; ////�ūת��p�ƲĤ@���
							Time_Flag = 0;        //�ɶ��X�вM��
						}
					}
					else{
						Past_Time = HAL_GetTick();   //�x�s�L�h���ɶ�
						Time_Flag = 1;			//Time �X��
					}
					break;
				case 3:			//�ܴ�DAC���i�� �q F-1 ~ F-3  ����1��
					Seven_segment_Bits[2] = DAC_Mode[Mode-1][0];		// F
					Seven_segment_Bits[1] = DAC_Mode[Mode-1][1];		// -
					Seven_segment_Bits[0] = DAC_Mode[Mode-1][2];		// x - �Ʀr
				
					if(Timer_clock == 0){		//����1��
						Task = 2;				//���Ȧ^��2
						Seven_segment_Bits[2] = Seven_segment_decode[((uint8_t)Temperature())/10];		//�ūת��Q���
						Seven_segment_Bits[1] = Seven_segment_decode[((uint8_t)Temperature())%10];		//�ūת��Ӧ��
						Seven_segment_Bits[0] = Seven_segment_decode[((uint8_t)(Temperature()*10))%10]; ////�ūת��p�ƲĤ@���
					}
					break;
				case 4:     //�ܴ�DAC���i�� �q F-3 ~ F-1  ����1��
					Seven_segment_Bits[2] = DAC_Mode[Mode-1][0];		// F
					Seven_segment_Bits[1] = DAC_Mode[Mode-1][1];		// -
					Seven_segment_Bits[0] = DAC_Mode[Mode-1][2];		// x - �Ʀr
					if(Timer_clock == 0){		//����1��
						Task = 2;				//���Ȧ^��2
						Seven_segment_Bits[2] = Seven_segment_decode[((uint8_t)Temperature())/10];		//�ūת��Q���
						Seven_segment_Bits[1] = Seven_segment_decode[((uint8_t)Temperature())%10];		//�ūת��Ӧ��
						Seven_segment_Bits[0] = Seven_segment_decode[((uint8_t)(Temperature()*10))%10]; ////�ūת��p�ƲĤ@���
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
		case 0: //�Ӧ��
			Seven_segment_Get_Data(Seven_segment_Bits[0]);
		  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
			break;
		case 1: //�Q���
			if(Task == 2)
				Seven_segment_Get_Data(Seven_segment_Bits[1] & 0xFE);			//��ܤp���I
			else
				Seven_segment_Get_Data(Seven_segment_Bits[1]);						//����ܤp���I
			
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_11,GPIO_PIN_SET);
			break;
		case 2: //�ʦ��
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
void Seven_segment_Get_Data(uint8_t data){		//���Ƶ��C�q
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
	const float Vcc = 3.3;							//�q��Vcc
	const int     B = 3425;			//3380				//beta ��j�Y��
	const double T_25 = 273.15 + 25;		//�`��25��
	const double R1 = 10000.0;					//�b25�׮ɼ��ӹq����10K�کi
	const double V1 = 0.1;							//OPA ���ݿ�J��0.1��S
	HAL_ADC_Start_IT(&hadc);
	double AIN0 = (((double)ADC_Buffer[0]) * Vcc )/4096.0;
	R18 = (AIN0 - V1) / 0.00001;
	
	Temperature = ((T_25 * B) / (B + T_25 * log (R18 / R1))) - 273.15;
	
	return Temperature;
}

uint8_t S1_push_function(uint8_t state){
	switch(state){
		case 0:			//�P�_���s�O�_���U
			if(HAL_GPIO_ReadPin(S1_GPIO_Port,S1_Pin)){
				state = 1;
			}
			break;
		case 1:			//�P�_���s�O�_��}
			if(!(HAL_GPIO_ReadPin(S1_GPIO_Port,S1_Pin))){
				state = 2;
			}
			break;
		case 2: 		//�T�w���s�Q���U  �Ƶ�:�Ф�ʧ�S1_state �M�����s;
			break;
	}
	return state;
}
uint8_t S2_push_function(uint8_t state){
	switch(state){
		case 0:			//�P�_���s�O�_���U
			if(HAL_GPIO_ReadPin(S2_GPIO_Port,S2_Pin)){
				state = 1;
			}
			break;
		case 1:			//�P�_���s�O�_��}
			if(!(HAL_GPIO_ReadPin(S2_GPIO_Port,S2_Pin))){
				state = 2;
			}
			break;
		case 2: 		//�T�w���s�Q���U  �Ƶ�:�Ф�ʧ�S2_state �M�����s;
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
