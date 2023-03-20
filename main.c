/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OS.h"
#include "lcd_st7565_pinconf.h"
#include "lcd_st7565.h"
#include "font.h"

#include "afisari.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* Definitions for stergator */
osThreadId_t stergatorHandle;
const osThreadAttr_t stergator_attributes = {
  .name = "stergator",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for joystick */
osThreadId_t joystickHandle;
const osThreadAttr_t joystick_attributes = {
  .name = "joystick",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for afisare */
osThreadId_t afisareHandle;
const osThreadAttr_t afisare_attributes = {
  .name = "afisare",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for semnalizare */
osThreadId_t semnalizareHandle;
const osThreadAttr_t semnalizare_attributes = {
  .name = "semnalizare",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
void Stergator(void *argument);
void Joystick(void *argument);
void Afisare(void *argument);
void Semnalizare(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t* varSemnal=(unsigned char *)("default");
uint8_t* varAfisare=(unsigned char *)("default");
uint8_t* preAfis=(unsigned char *)("default");
uint32_t previousMillis=0;
uint32_t previousMillis2=0;
uint32_t currentMillis;
uint32_t servoDelay=0;
uint8_t treapta=0;
uint32_t VR[2];
uint8_t stareAvarii=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM16_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	//Start timer used for displaying a char on the bottom left corner each second
	HAL_TIM_Base_Start(&htim16);

	// initialize LCD
	st7565_init();
	st7565_backlight_enable();
	HAL_Delay(500);

	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);

	uint32_t currentMillis = HAL_GetTick();
	previousMillis=currentMillis;


	//varAfisare="default";

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of stergator */
  stergatorHandle = osThreadNew(Stergator, NULL, &stergator_attributes);

  /* creation of joystick */
  joystickHandle = osThreadNew(Joystick, NULL, &joystick_attributes);

  /* creation of afisare */
  afisareHandle = osThreadNew(Afisare, NULL, &afisare_attributes);

  /* creation of semnalizare */
  semnalizareHandle = osThreadNew(Semnalizare, NULL, &semnalizare_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{



//			  	button=HAL_GPIO_ReadPin(button_GPIO_Port, button_Pin);
//			  	if(button == 1){
//			  		  		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, SET);
//
//			  		  	}
//			  		  	else
//			  		  		if(button == 0){
//			  		  			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
//
//			  		  		}

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 4800-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Ch1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPICD_GPIO_Port, SPICD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BL_Pin|SPIRST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPICS_GPIO_Port, SPICS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPICD_Pin PC10 PC12 */
  GPIO_InitStruct.Pin = SPICD_Pin|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BL_Pin SPIRST_Pin */
  GPIO_InitStruct.Pin = BL_Pin|SPIRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPICS_Pin */
  GPIO_InitStruct.Pin = SPICS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPICS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void reset(void){
	varAfisare=(unsigned char *)("default");
	treapta=0;
	servoDelay=0;
	TIM3 ->CCR4=450;
	preAfis=varAfisare;
	stareAvarii=0;
	varSemnal=varAfisare;
}

void joystickSus(void){
	afisareStergereSimpla();
	servoDelay=800;
	treapta=0;
	varAfisare=(unsigned char *)("sus");
	previousMillis2= HAL_GetTick(); //marchez inceputul afisarii
	previousMillis= previousMillis2;
}

void joystickJos(void){

	varAfisare=(unsigned char *)("jos");

	if(treapta==1){
		servoDelay=750;
	}
	else if(treapta==2){
		servoDelay=550;
	}
	else if(treapta==3){
		servoDelay=350;
	}
	else if(treapta>3){
		treapta=0;
		servoDelay=0;
		TIM3 ->CCR4=450;
	}

	previousMillis2= HAL_GetTick();
}

void afisareJos(void){
	currentMillis = HAL_GetTick();
	switch(treapta) {
	    case 1:
	    	afisareStergTreapta1();
	        break;
	    case 2:
	    	afisareStergTreapta2();
	        break;
	    case 3:
	    	afisareStergTreapta3();
	    	break;
	}

	if(stareAvarii==1){
		if (currentMillis - previousMillis2 >= 1000){
			varAfisare=(unsigned char *)("stanga-jos");
			previousMillis2=currentMillis;
		}
	}
}

void joystickStanga(void){
	if(varAfisare==(unsigned char *)("jos"))
			preAfis=varAfisare;
	varAfisare=(unsigned char *)("stanga");
	varSemnal=varAfisare;
	previousMillis2= HAL_GetTick();
	stareAvarii=0;
}

void afisareStanga(void){
	currentMillis = HAL_GetTick();
	uint32_t perioada=currentMillis - previousMillis2;
	if (( perioada % 1000 <=500)  ){  //sa apara la fiecare 500ms
		afisareSemnalStanga();
	}
	else{
		golireEcran();
	}

	if (currentMillis - previousMillis2 >= 5000){  //dupa 5 secunde sa se opreasca afisarea
		varAfisare=preAfis;
	}
}

void semnalStanga(void){
	currentMillis = HAL_GetTick();
	uint32_t perioada=currentMillis - previousMillis2;
	if (( perioada % 1000 <=500)  ){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
	}

	if (currentMillis - previousMillis2 >= 5000){  //dupa 5 secunde sa se opreasca afisarea
		varSemnal=(unsigned char *)("default");
	}
}

void joystickDreapta(void){
	if(varAfisare==(unsigned char *)("jos"))
			preAfis=varAfisare;
	varAfisare=(unsigned char *)("dreapta");
	varSemnal=varAfisare;
	previousMillis2= HAL_GetTick();
	stareAvarii=0;
}

void afisareDreapta(void){
	currentMillis = HAL_GetTick();
	uint32_t perioada=currentMillis - previousMillis2;
	if (( perioada % 1000 <=500)  ){  //sa apara la fiecare 500ms
		afisareSemnalDreapta();
	}
	else golireEcran();

	if (currentMillis - previousMillis2 >= 5000){  //dupa 5 secunde sa se opreasca afisarea
		varAfisare=preAfis;
	}
}

void semnalDreapta(void){
	currentMillis = HAL_GetTick();
	uint32_t perioada=currentMillis - previousMillis2;
	if (( perioada % 1000 <=500)  ){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
	}

	if (currentMillis - previousMillis2 >= 5000){  //dupa 5 secunde sa se opreasca afisarea
		varSemnal=(unsigned char *)("default");
	}
}

void joystickDreaptaSus(void){
	if(varAfisare==(unsigned char *)("jos"))
		preAfis=varAfisare;
	varAfisare=(unsigned char *)("dreapta-sus");
	previousMillis2= HAL_GetTick();
}

void joystickStangaSus(void){
	if(varAfisare==(unsigned char *)("jos"))
			preAfis=varAfisare;
	varAfisare=(unsigned char *)("stanga-sus");
	previousMillis2= HAL_GetTick();
}


void joystickStangaJos(void){
	if(stareAvarii==0){
		varAfisare=(unsigned char *)("stanga-jos");
		varSemnal=(unsigned char *)("avarii");
		previousMillis2= HAL_GetTick();
		stareAvarii=1;
	}
	else{
		if(treapta>0){
			varAfisare==(unsigned char *)("jos");
		}
		else{
			varAfisare=(unsigned char *)("default");
		}
		varSemnal=(unsigned char *)("default");
		stareAvarii=0;
	}
}

void afisareStangaJos(void){
	currentMillis = HAL_GetTick();
	uint32_t perioada=currentMillis - previousMillis2;
	if (( perioada % 1000 <=500)  ){  //sa apara la fiecare 500ms
		afisareAvarii();
	}
	else golireEcran();

	if(treapta>0){
		if (currentMillis - previousMillis2 >= 2000){
			varAfisare=(unsigned char *)("jos");
			previousMillis2=currentMillis;
		}
	}
}

void avarii(void){
		currentMillis = HAL_GetTick();
		uint32_t perioada=currentMillis - previousMillis2;
		if (( perioada % 1000 <=500)  ){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
		}

}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_Stergator */
/**
  * @brief  Function implementing the stergator thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Stergator */
void Stergator(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  currentMillis = HAL_GetTick();

	  if(servoDelay){
		  if (currentMillis - previousMillis <= servoDelay){
		  	TIM3 ->CCR4=450;
		  }
		  if (currentMillis - previousMillis >= servoDelay){
		  	TIM3 ->CCR4=2500;
		  }
		  if (currentMillis - previousMillis >= 2*servoDelay){
		  	 previousMillis=currentMillis;
		  	 if(servoDelay==800){ //daca e in sus sa miste o singura data
		  		TIM3 ->CCR4=450;
		  		servoDelay=0;
		  	 }
		  }

	  }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Joystick */
/**
* @brief Function implementing the joystick thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Joystick */
void Joystick(void *argument)
{
  /* USER CODE BEGIN Joystick */
  /* Infinite loop */
  for(;;)
  {
	  //gasire pozitie joystick
	  			  HAL_ADC_Start_DMA(&hadc,VR,2);
	  			  HAL_Delay(300);

	  			  	if ((VR[0]>=1000) && (VR[0]<=3500) && (VR[1]>=1000) && (VR[1]<=3500)){  //centru
	  			  	}

	  			  	else if ((VR[0]<1000)  && (VR[1]>=1000) && (VR[1]<=3500)){ //dreapta
	  			  			joystickDreapta();

	  			  		}

	  			  	else if ((VR[0]>3500)  && (VR[1]>=1000) && (VR[1]<=3500)){ //stanga
	  			  		joystickStanga();
	  			  	}

	  			  	else if ((VR[0]>=1000) && (VR[0]<=3500) && (VR[1]<1000)){   //jos
	  			  		treapta=treapta+1;
	  			  		joystickJos();
	  			  	}

	  			  	else if ((VR[0]>=1000) && (VR[0]<=3500) && (VR[1]>3500)){   //sus
	  			  		joystickSus();
	  			  	}

	  			  	else if ((VR[0]<1000) && (VR[1]>3500)){ //dreapta-sus
	  			  		joystickDreaptaSus();
	  			  	}

	  			  	else if ((VR[0]>3500) && (VR[1]>3500)){  //stanga-sus
	  			  		joystickStangaSus();
	  			  	}

	  			  	else if ((VR[0]>3500) && (VR[1]<1000)){//stanga-jos
	  			  		joystickStangaJos();
	  			  	}

	  			  	else if ((VR[0]<1000) && (VR[1]<1000)){ //dreapta-jos
	  			  		reset();
	  			  	}
    osDelay(1);
  }
  /* USER CODE END Joystick */
}

/* USER CODE BEGIN Header_Afisare */
/**
* @brief Function implementing the afisare thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Afisare */
void Afisare(void *argument)
{
  /* USER CODE BEGIN Afisare */
  /* Infinite loop */
  for(;;)
  {
	  currentMillis = HAL_GetTick();
	  if(varAfisare==(unsigned char *)("default"))
		  afisareDefault();

	  else if(varAfisare==(unsigned char *)("sus")){
		  afisareStergereSimpla();
		  if (currentMillis - previousMillis2 >= 2500){
			  varAfisare=preAfis;
		  }
	  }

	  else if(varAfisare==(unsigned char *)("jos"))
		  afisareJos();

	  else if(varAfisare==(unsigned char *)("stanga-sus")){
		  afisareStropParb();
		  if (currentMillis - previousMillis2 >= 1000){
			  varAfisare=preAfis;
		  }
	  }

	  else if(varAfisare==(unsigned char *)("dreapta-sus")){
		  afisareStropLun();
		  if (currentMillis - previousMillis2 >= 1000){
			  varAfisare=preAfis;
		  }
	  }

	  else if(varAfisare==(unsigned char *)("dreapta")){
		  afisareDreapta();
	  }

	  else if(varAfisare==(unsigned char *)("stanga")){
		  afisareStanga();
	  }

	  else if(varAfisare==(unsigned char *)("stanga-jos")){
		  afisareStangaJos();
	  }

    osDelay(1);
  }
  /* USER CODE END Afisare */
}

/* USER CODE BEGIN Header_Semnalizare */
/**
* @brief Function implementing the semnalizare thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Semnalizare */
void Semnalizare(void *argument)
{
  /* USER CODE BEGIN Semnalizare */
  /* Infinite loop */
  for(;;)
  {

	  if(varSemnal==(unsigned char *)("default")){
		   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, RESET);
		   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, RESET);
	  }
	  else if(varSemnal==(unsigned char *)("dreapta")){
		  semnalDreapta();
	  	  }

	  else if(varSemnal==(unsigned char *)("stanga")){
		  semnalStanga();
	  	  }
	  else if(varSemnal==(unsigned char *)("avarii")){
		  avarii();
	  	  }
    osDelay(1);
  }
  /* USER CODE END Semnalizare */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
