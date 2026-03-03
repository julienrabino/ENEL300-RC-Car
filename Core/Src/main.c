/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CONFIGURE_HC05 0   // 1 = AT mode, 0 = Data mode
#define BUF_LEN 12 //  length of command buffer, set at 12 so that it can hold 3 sets of commands at once
#define CMD_LEN 4 /* length of a command from the remote.
					--- FORMAT in BYTES ---
					Byte 0: Left Motor Direction (0 for reverse, 1 for forward)
					Byte 1: Left Motor Speed
					Byte 2: Right Motor Direction (" ")
					Byte 3: Right Motor Speed


				  */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ===== METAL DETECTOR DEFINES =====
#define NPULSE 12
#define NMEAS 256
#define METAL_THRESHOLD 50  // IMPORTANT tweak this accordingly for metal detector ***********************

#define METAL_PULSE_GPIO_Port GPIOA
#define METAL_PULSE_Pin GPIO_PIN_7

#define METAL_LED_GPIO_Port GPIOA
#define METAL_LED_Pin LD2_Pin

#define METAL_OUT_GPIO_Port GPIOB
#define METAL_OUT_Pin GPIO_PIN_2

long int sumsum = 0;
long int skip = 0;
long int diff = 0;
long int flash_period = 0;
uint32_t prev_flash = 0;

uint32_t echo_start = 0;
uint32_t echo_end = 0;
uint8_t last_edge = 0;
uint32_t time_diff = 0;
uint8_t echo_measured = 0; // flag for determining if distance sensing done
float distance_cm = 0;
uint32_t last_dist_time = 0;
volatile uint8_t rx_byte = 0;
volatile uint8_t cmd_buffer [BUF_LEN];
volatile uint8_t cmd_idx = 0; // incremented in ISR
volatile uint8_t process_idx = 0;  // main loop processes from this idx

void triggerDistanceSensing(){
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
    for(volatile int i = 0; i < 280; i++); // delay 10 us
    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

}
void driveLeft(int speed){
	// assume speed has already been scaled from 0-1999

    if (speed > 0)
    {
        HAL_GPIO_WritePin(LEFT_DIR_GPIO_Port, LEFT_DIR_Pin, GPIO_PIN_RESET);  // DIR = 0 (forward)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);  // PWM duty
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(LEFT_DIR_GPIO_Port, LEFT_DIR_Pin, GPIO_PIN_SET);   // DIR = 1 (reverse)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, -speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);      // brake
    }
}

void driveRight(int speed){
	// assume speed has already been scaled from 0-1999

    if (speed > 0)
    {
        HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, GPIO_PIN_SET);  // DIR = 1 (forward)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);  // PWM duty
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, GPIO_PIN_RESET);   // DIR = 0 (reverse)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -speed);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);      // brake
    }
}


void processCommand(uint8_t start_idx){
	// call drive left and drive right
	uint8_t process_idx = start_idx;



	uint8_t raw_data [CMD_LEN];

	for (uint8_t i = 0; i < CMD_LEN;  i++){
		raw_data[i] = cmd_buffer[(process_idx % BUF_LEN)];
		process_idx ++;

	}

	uint8_t left_dir = raw_data[0];
	int left_speed = (int)raw_data[1];
	uint8_t right_dir = raw_data[2];
	int right_speed = (int)raw_data[3];

	int scaled_left = (left_speed * 1999) / 255;
	int scaled_right = (right_speed * 1999) / 255;


	scaled_left = (left_dir == 1 ? scaled_left : - scaled_left);
	scaled_right = (right_dir == 1? scaled_right: - scaled_right);

	driveLeft(scaled_left);
	driveRight(scaled_right);

}



void configureHC05(){
	char resp[50] = {0};


	// set car as "slave"
	char cmd2[] = "AT+ROLE=0\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd2, strlen(cmd2), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart1, (uint8_t*)resp, 4, HAL_MAX_DELAY);
	HAL_Delay(500);


	// set data mode baud rate = 9600
	char cmd4[] = "AT+UART=9600,0,0\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd4, strlen(cmd4), HAL_MAX_DELAY);
	HAL_UART_Receive(&huart1, (uint8_t*)resp, 10, HAL_MAX_DELAY);
	HAL_Delay(500);
}


uint16_t readADC(void) {

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return val;
}

void runMetalDetector(void) {

    int minval = 4095;
    int maxval = 0;
    uint32_t sum = 0;

    for (int imeas = 0; imeas < NMEAS + 2; imeas++) {
        HAL_GPIO_WritePin(METAL_PULSE_GPIO_Port, METAL_PULSE_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);

        for (int ipulse = 0; ipulse < NPULSE; ipulse++) {
            HAL_GPIO_WritePin(METAL_PULSE_GPIO_Port, METAL_PULSE_Pin, GPIO_PIN_SET);
            for(volatile int i=0;i<20;i++);
            HAL_GPIO_WritePin(METAL_PULSE_GPIO_Port, METAL_PULSE_Pin, GPIO_PIN_RESET);
            for(volatile int i=0;i<20;i++);
        }

        int val = readADC();

        if (val < minval) minval = val;
        if (val > maxval) maxval = val;

        sum += val;
    }

    sum -= minval;
    sum -= maxval;

    if (sumsum == 0)
        sumsum = sum << 6;

    long int avgsum = (sumsum + 32) >> 6;
    diff = sum - avgsum;

    if (abs(diff) < avgsum >> 10) {
        sumsum = sumsum + sum - avgsum;
        skip = 0;
    }
    else {
        skip++;
    }

    if (skip > 64) {
        sumsum = sum << 6;
        skip = 0;
    }

    if (diff == 0) flash_period = 1000000;
    else flash_period = avgsum / (2 * abs(diff));

    uint32_t timestamp = HAL_GetTick();

    if (flash_period > 1000) {
        HAL_GPIO_WritePin(METAL_LED_GPIO_Port, METAL_LED_Pin, GPIO_PIN_RESET);
    }
    else {
        if (timestamp - prev_flash >= flash_period) {
            HAL_GPIO_TogglePin(METAL_LED_GPIO_Port, METAL_LED_Pin);
            prev_flash = timestamp;
        }
    }

    if (abs(diff) > METAL_THRESHOLD) {
        HAL_GPIO_WritePin(METAL_OUT_GPIO_Port, METAL_OUT_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(METAL_OUT_GPIO_Port, METAL_OUT_Pin, GPIO_PIN_RESET);
    }

}


# if 0
void driveRight(int speed){
	// assume speed has already been scaled from 0-1000
	if (speed > 0){
		// if speed is positive, drive forward
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed); // PWM on IN1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // LOW on IN2
	} else if (speed < 0) {
		// if speed is negative, drive backward
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); // LOW on IN1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -speed); // PWM on IN2

	} else {
		// speed is 0, turn motor off
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); // LOW on IN1
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); // LOW on IN2
	}
}

#endif
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart1, (uint8_t *) &rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

# if CONFIGURE_HC05
	configureHC05();

	// important: after running this ONCE, change CONFIGURE_HC05 to 0 and also the baud rate to 9600.
	while (1){} // stop execution so normal car code doesn't run
#endif

  while (1)
  {

	  uint8_t idx_diff = 0;

	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  // if there is a command ready to be processed
	  if (cmd_idx < process_idx){
		  idx_diff = (BUF_LEN - process_idx) + cmd_idx;

	  } else if (cmd_idx > process_idx){
		  idx_diff = cmd_idx - process_idx;
	  }


	  if (idx_diff >= CMD_LEN){
		  processCommand(process_idx);
		  process_idx += CMD_LEN;
		  if (process_idx >= BUF_LEN) process_idx = 0; // wrap pointer around


	  }

	  static uint32_t last_metal_time = 0;

	  if (HAL_GetTick() - last_metal_time > 50) // 50 ms delays
	  {
	      runMetalDetector();
	      last_metal_time = HAL_GetTick();
	  }
#if 1

	  // trigger distance sensor every 2 seconds
	  if (HAL_GetTick() - last_dist_time >= 2000){
		  triggerDistanceSensing();
		  last_dist_time = HAL_GetTick();

	  }
	  if (echo_measured){
		  echo_measured = 0; // reset flag
		  distance_cm  = time_diff / 58;
	  }


	  uint32_t distance_mm = (int)(distance_cm * 10);

	  printf("Distance: %d mm\n\r", distance_mm);

#endif

# if 0
	  // test drive
	  driveLeft(1000);

	  HAL_Delay(5000);

	  driveLeft(-1000);
	  HAL_Delay(5000);

	  driveLeft(0);
	  HAL_Delay(5000);

#endif




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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 359;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|LD2_Pin|metal_pulse_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEFT_DIR_Pin|RIGHT_DIR_Pin|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin LD2_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : metal_pulse_out_Pin */
  GPIO_InitStruct.Pin = metal_pulse_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(metal_pulse_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_DIR_Pin RIGHT_DIR_Pin PB2 */
  GPIO_InitStruct.Pin = LEFT_DIR_Pin|RIGHT_DIR_Pin|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(metal_pulse_out_GPIO_Port, metal_pulse_out_Pin, GPIO_PIN_RESET);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		cmd_buffer[cmd_idx++] = rx_byte;
		if (cmd_idx == BUF_LEN) cmd_idx = 0; // wrap around
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {

    	// ECHO rising edge
    	if (last_edge == 0) {
    		echo_start = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    		last_edge = 1;

    	} else {
    		echo_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    		if (echo_end < echo_start){
    			time_diff = htim->Init.Period - echo_start + echo_end;
    		} else {
    			time_diff = echo_end - echo_start;
    		}

    		last_edge = 0;
    		echo_measured = 1;
    	}
    }

}
/* USER CODE END 4 */

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
