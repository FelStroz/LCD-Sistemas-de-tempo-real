/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "Lcd_Alpha_16x2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	uint8_t channel;
	uint16_t value[5];
}adcData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_THRESHOLD (20)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId AdcTaskHandle;
osThreadId LedTaskHandle;
osThreadId LogTaskHandle;
osThreadId Digital_IO_TaskHandle;
osThreadId MainTaskHandle;
osMessageQId AdcQueueHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);
void StartAdcTask(void const * argument);
void StartLedTask(void const * argument);
void StartLogTask(void const * argument);
void StartDigital_IO_Task(void const * argument);
void StartMainTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_SDIO_SD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of AdcQueue */
  osMessageQDef(AdcQueue, 10, adcData_t);
  AdcQueueHandle = osMessageCreate(osMessageQ(AdcQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of AdcTask */
  osThreadDef(AdcTask, StartAdcTask, osPriorityBelowNormal, 0, 128);
  AdcTaskHandle = osThreadCreate(osThread(AdcTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, StartLedTask, osPriorityLow, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of LogTask */
  osThreadDef(LogTask, StartLogTask, osPriorityLow, 0, 128);
  LogTaskHandle = osThreadCreate(osThread(LogTask), NULL);

  /* definition and creation of Digital_IO_Task */
  osThreadDef(Digital_IO_Task, StartDigital_IO_Task, osPriorityHigh, 0, 128);
  Digital_IO_TaskHandle = osThreadCreate(osThread(Digital_IO_Task), NULL);

  /* definition and creation of MainTask */
  osThreadDef(MainTask, StartMainTask, osPriorityNormal, 0, 128);
  MainTaskHandle = osThreadCreate(osThread(MainTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 42;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Luz_Erro_Pin|Buzzer_PWM_Pin|Motor_D_Pin|Motor_C_Pin 
                          |Motor_B_Pin|D4_Pin|D5_Pin|D6_Pin 
                          |D7_Pin|EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_A_GPIO_Port, Motor_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Bot_o_Silenciar_Alarme_Pin Bot_o_E_Stop_Pin Bot_o_Modo_Opera__o_Pin */
  GPIO_InitStruct.Pin = Bot_o_Silenciar_Alarme_Pin|Bot_o_E_Stop_Pin|Bot_o_Modo_Opera__o_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Chave_Fim_Curso_Pin */
  GPIO_InitStruct.Pin = Chave_Fim_Curso_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Chave_Fim_Curso_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Luz_Erro_Pin Buzzer_PWM_Pin Motor_D_Pin Motor_C_Pin 
                           Motor_B_Pin D4_Pin D5_Pin D6_Pin 
                           D7_Pin EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = Luz_Erro_Pin|Buzzer_PWM_Pin|Motor_D_Pin|Motor_C_Pin 
                          |Motor_B_Pin|D4_Pin|D5_Pin|D6_Pin 
                          |D7_Pin|EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_A_Pin */
  GPIO_InitStruct.Pin = Motor_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_A_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Delay_us(uint16_t delay){
	__HAL_TIM_SET_AUTORELOAD(&htim4, delay);
	HAL_TIM_Base_Start_IT(&htim4);
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	HAL_TIM_Base_Stop_IT(&htim4);
}

void MotorDriveHalfStep (uint8_t step){
	switch(step){
		case 0:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_SET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_SET);
			break;
		default:
			HAL_GPIO_WritePin(GPIOB, Motor_A_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_C_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, Motor_D_Pin, GPIO_PIN_RESET);
			break;
	}

}

void MotorStep (bool directionForward){
	static int8_t step = 0;
	
	MotorDriveHalfStep (step);
	
	if(directionForward){
		step++;
		if(step > 7){
			step = 0;
		}
	}else{
		step--;
		if(step < 0){
			step = 7;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
	BaseType_t TaskWoken = pdFALSE;
	
	UNUSED(hadc);
	
	vTaskNotifyGiveFromISR(AdcTaskHandle, &TaskWoken);
	
	portYIELD_FROM_ISR(TaskWoken);
}

void makeBarGraph(uint16_t value, char str[]){
	int8_t i;
	uint8_t aux;
	
	aux = (uint8_t) (value/256);
	for(i=0; i <= aux ; i++){
		str[i] = 0xFF;
	}
	str[i] = '\0';
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		 HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(250);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartAdcTask */
/**
* @brief Function implementing the AdcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAdcTask */
void StartAdcTask(void const * argument)
{
  /* USER CODE BEGIN StartAdcTask */
	static int16_t adcValuesDma[5], adcOldValuesDma[5] = {0xFFFF};
	int8_t i;
	adcData_t adcData;
	
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcValuesDma, 5);
	HAL_TIM_Base_Start(&htim3);
	
  /* Infinite loop */
  for(;;)
  {
		
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		adcData.channel = 0xFF;
		for(i = 0; i< 5; i++){
			if(abs(adcValuesDma[i] - adcOldValuesDma[i]) > ADC_THRESHOLD){
				
				adcData.channel = i;
			}
			adcData.value[i] = (uint16_t) adcValuesDma[i];
			adcOldValuesDma[i] = adcValuesDma[i];
		}
		xQueueSendToBack(AdcQueueHandle, &adcData, 10);
		
  }
  /* USER CODE END StartAdcTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
	adcData_t adcData;
	uint16_t lcdTimeout = 0;
	int8_t lcdState = 0;
	uint8_t controlMode = 0;
	char linha1[17] = {0};
	char linha2[17] = {0};
	
	
	LCD1_Init();
	
	LCD1_WriteLineStr(1, "Respirator Felipe");
	LCD1_WriteLineStr(2, "Inicializando...");
	
	osDelay(2000);
  /* Infinite loop */
  for(;;)
  {
		xQueueReceive(AdcQueueHandle, &adcData, portMAX_DELAY);
		
		if(lcdState == (adcData.channel +1)){
			lcdTimeout = 20;
		}
		
		if(lcdTimeout > 0){
			lcdTimeout--;
			if(lcdTimeout == 0){
				lcdState = 0;
			}
		}
		
		switch(lcdState){
			case 0: // Tela principal
			if(adcData.channel == 0xFF){		
					sprintf(linha1, "Vt= %02i%%  BPM = %02i", (uint8_t) (100.0*adcData.value[0]/4000),(uint8_t) (adcData.value[1]));
				if(controlMode == 0){
					sprintf(linha2, "I/E= 1:%1i  Assist.", 1+ (uint8_t) (100.0*adcData.value[0]/1024));
				}else{
					sprintf(linha2, "I/E= 1:%1i     Vol.", 1+ (uint8_t) (100.0*adcData.value[0]/1024));
				}
			}else{
				lcdState = adcData.channel + 1;
				lcdTimout = 20;
			}
				break; 
			case 1: // tela BPM
				sprintf(linha1, "BPM = %03i b/min",(uint16_t) (adcData.value[0]));
				makeBarGraph(adcData.value[0], linha2);
				break;
			case 2: // tela Tidal
				sprintf(linha1, "Volume = %03i mL",(uint16_t) (adcData.value[1]));
				makeBarGraph(adcData.value[1], linha2);
			
				break;
			case 3: // tela IE
				sprintf(linha1, "    IE = 1:%i mL",(uint8_t) (adcData.value[2]));
				makeBarGraph(adcData.value[2], linha2);
				break;
			case 4: // tela Limiar
				sprintf(linha1, "Sensib. = %02i cm",(uint8_t) (adcData.value[3]));
				makeBarGraph(adcData.value[3], linha2);
				break;
			case 5:
				sprintf(linha1, "Limiar. = %0X",(uint16_t) (adcData.value[4]));
				makeBarGraph(adcData.value[4], linha2);
				break;
		
		}
    
		LCD1_WriteLineStr(1, linha1);
		LCD1_WriteLineStr(2, linha2);
		
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
* @brief Function implementing the LogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogTask */
void StartLogTask(void const * argument)
{
  /* USER CODE BEGIN StartLogTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartLogTask */
}

/* USER CODE BEGIN Header_StartDigital_IO_Task */
/**
* @brief Function implementing the Digital_IO_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDigital_IO_Task */
void StartDigital_IO_Task(void const * argument)
{
  /* USER CODE BEGIN StartDigital_IO_Task */
  /* Infinite loop */
  for(;;)
  {
		MotorStep(true); //Frente
		Delay_us(10); // 10us
  }
  /* USER CODE END StartDigital_IO_Task */
}

/* USER CODE BEGIN Header_StartMainTask */
/**
* @brief Function implementing the MainTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMainTask */
void StartMainTask(void const * argument)
{
  /* USER CODE BEGIN StartMainTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartMainTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	BaseType_t TaskWoken;
	
	if (htim->Instance == TIM4) {
		
	TaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(Digital_IO_TaskHandle, &TaskWoken);
	portYIELD_FROM_ISR(TaskWoken);
		
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
