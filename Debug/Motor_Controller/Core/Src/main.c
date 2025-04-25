/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>

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

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
#define LEFT_IC htim11
#define LEFT_IC_CHANNEL TIM_CHANNEL_1
#define LEFT_PWM htim8
#define LEFT_PWM_CHANNEL TIM_CHANNEL_4
#define LEFT_FORWARD GPIOE
#define LEFT_FORWARD_PIN GPIO_PIN_14
#define LEFT_REVERSE GPIOE
#define LEFT_REVERSE_PIN GPIO_PIN_13

#define RIGHT_IC htim10
#define RIGHT_IC_CHANNEL TIM_CHANNEL_1
#define RIGHT_PWM htim8
#define RIGHT_PWM_CHANNEL TIM_CHANNEL_3
#define RIGHT_FORWARD GPIOE
#define RIGHT_FORWARD_PIN GPIO_PIN_11
#define RIGHT_REVERSE GPIOE
#define RIGHT_REVERSE_PIN GPIO_PIN_12

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
void TIM_Start_IC(TIM_HandleTypeDef *htim);
void TIM_Stop_IC(TIM_HandleTypeDef *htim);
void Start_Diff_PWM(void);
void w_calc(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void poll_speed(TIM_HandleTypeDef *htim);
void motors_controller(float speed_r, float speed_l);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define error_t 10
#define pwm_period 2399
#define ic_period 59999

uint16_t counter_buffer[4] = {0,0,0,0};
uint8_t w_time[3] = {0,0,'\n'};
uint8_t re_count = 0;
int is_sent = 0;
uint8_t poll_motor = 0;
float time_ms = 0;
//facing direction. 0 - forward, 1 - right, 2 - left, 3 - back
int facing = 0;
int tide_angle = 0;

int tim9_counter = 0;
int ic_overrun = 0;

void TIM_Start_IC(TIM_HandleTypeDef *htim) {
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
}

void TIM_Stop_IC(TIM_HandleTypeDef *htim) {
	HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_1);
}

void TIM10_Stop_IC(void) {
	HAL_TIM_IC_Stop_IT(&htim10, TIM_CHANNEL_1);
}

void reset_tim9(void) {
	TIM9 -> ARR = 0;
	TIM9 -> CR1 &= ~TIM_CR1_UDIS;
	TIM9 -> EGR = TIM_EGR_UG;
	TIM9 -> CR1 |= TIM_CR1_UDIS;
}

void Start_Diff_PWM(void) {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
}

//takes forward displacement and the angle that it needs to be at
void move(float x, float angle) {
	while(angle != tide_angle) {
		int angle_diff = angle - tide_angle;
		//assume within tolerance, not enough time in project to do otherwise
		//if angle difference is positive, turn right
		//if angle difference is negative, turn left
		if(angle_diff < 0) {
			while(angle_diff < error_t) {
				motors_controller(1, -1);
				angle_diff = angle - tide_angle;
			}
		}
		else if(angle_diff > 0) {
			while(angle_diff > error_t) {
				motors_controller(-1, 1);
				angle_diff = angle - tide_angle;
			}
		}
	}
	//calculated equation. assume x is in 6 inch intervals
	if(x == 1) {
		motors_controller(1,1);
		HAL_TIM_Base_Start_IT(&htim9);
		TIM9 -> CR1 |= 0b1;
		//wait for timer to overrun the 4.25 second period
		while(!ic_overrun);
		motors_controller(0,0);
		//HAL_TIM_Base_Stop_IT(&htim9);
		//reset_tim9();
		//TIM9 -> ARR = 0;
		TIM9 -> CNT = 0;
		ic_overrun = 0;
		tim9_counter = 0;
	}
}

void motors_controller(float on_r, float on_l) {
	if(on_r == 1) {
		HAL_GPIO_WritePin(RIGHT_REVERSE, RIGHT_REVERSE_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIGHT_FORWARD, RIGHT_FORWARD_PIN, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&RIGHT_PWM, RIGHT_PWM_CHANNEL, 0.8 * pwm_period);
	}
	else if(on_r == -1) {
		HAL_GPIO_WritePin(RIGHT_FORWARD, RIGHT_FORWARD_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIGHT_REVERSE, RIGHT_REVERSE_PIN, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&RIGHT_PWM, RIGHT_PWM_CHANNEL, 0.8 * pwm_period);
	}
	else {
		__HAL_TIM_SET_COMPARE(&RIGHT_PWM, RIGHT_PWM_CHANNEL, 0);
		HAL_GPIO_WritePin(RIGHT_FORWARD, RIGHT_FORWARD_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RIGHT_REVERSE, RIGHT_REVERSE_PIN, GPIO_PIN_RESET);
	}

	if(on_l == 1) {
		HAL_GPIO_WritePin(LEFT_REVERSE, LEFT_REVERSE_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_FORWARD, LEFT_FORWARD_PIN, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&LEFT_PWM, LEFT_PWM_CHANNEL, 0.778 * pwm_period);
	}
	else if(on_l == -1) {
		HAL_GPIO_WritePin(LEFT_FORWARD, LEFT_FORWARD_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_REVERSE, LEFT_REVERSE_PIN, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&LEFT_PWM, LEFT_PWM_CHANNEL, 0.778 * pwm_period);
	}
	else {
		__HAL_TIM_SET_COMPARE(&LEFT_PWM, LEFT_PWM_CHANNEL, 0);
		HAL_GPIO_WritePin(LEFT_FORWARD, LEFT_FORWARD_PIN, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEFT_REVERSE, LEFT_REVERSE_PIN, GPIO_PIN_RESET);
	}
}

//poll motor speed
void poll_speed(TIM_HandleTypeDef *htim) {
	//start IC for motor. reset count
	/*__HAL_TIM_CLEAR_IT(htim ,TIM_IT_UPDATE);
	__HAL_TIM_SET_COUNTER(htim, 0);
	TIM_Start_IC(htim);
	//wait for poll to complete. Set in motor controller and IC ISR. w will update in IC isr
	//volatile uint32_t current = htim->Instance->CNT;
	//volatile uint32_t last = 0;
	//HAL_TIM_Base_Start_IT(&htim9);
	while(poll_motor == 0) {
		if(ic_overrun == 1){
			HAL_TIM_Base_Stop_IT(&htim9);
			poll_motor = 1;
			w = 0;
		}
	}
	HAL_TIM_Base_Stop_IT(&htim9);
	TIM9 -> ARR = 0;
	TIM9 -> CR1 &= ~TIM_CR1_UDIS;
	TIM9 -> EGR = TIM_EGR_UG;
	TIM9 -> CR1 |= TIM_CR1_UDIS;
	ic_overrun = 0;
	poll_motor = 0;
	TIM_Stop_IC(htim);*/
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	/*if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		//add count value of each interrupt to the counter buffer
		counter_buffer[re_count] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		//htim->Instance->CNT = 0;
		re_count++;
		//when there are 4 high signals, calculate and transmit the speed
		if(re_count == 4) {
			TIM_Stop_IC(htim);
			w_calc();
			poll_motor = 1;
		    re_count = 0;
		    //reset timer
		    __HAL_TIM_SET_COUNTER(htim, 0);
		    TIM9 -> ARR = 0;
		    TIM9 -> CR1 &= ~TIM_CR1_UDIS;
		    TIM9 -> EGR = TIM_EGR_UG;
		    TIM9 -> CR1 |= TIM_CR1_UDIS;
		    //HAL_UART_Transmit_DMA(&huart4, transmit_start, 6);
		    }
		}*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == htim9.Instance){
		__HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
		tim9_counter++;
		if(tim9_counter >= 17){
			ic_overrun = 1;
			TIM9 -> CR1 &= ~0b1;
		}
	}
}

void w_calc(void) {
	/*//timer period is 10ms
	//calculate time difference between each
	uint16_t tim01;
	if (counter_buffer[1] > counter_buffer[0]) {
		tim01 = counter_buffer[1] - counter_buffer[0];
	}

	else if (counter_buffer[0] > counter_buffer[1]) {
		tim01 = (0xEA5F - counter_buffer[0]) + counter_buffer[1];
	}
	uint16_t tim12;
	if (counter_buffer[2] > counter_buffer[1]) {
		tim12 = counter_buffer[2] - counter_buffer[1];
	}

	else if (counter_buffer[1] > counter_buffer[2]) {
		tim12 = (0xEA5F - counter_buffer[1]) + counter_buffer[2];
	}
	uint16_t tim23;
	if (counter_buffer[3] > counter_buffer[2]) {
		tim23 = counter_buffer[3] - counter_buffer[2];
	}

	else if (counter_buffer[2] > counter_buffer[3]) {
		tim23 = (0xEA5F - counter_buffer[2]) + counter_buffer[3];
	}
	uint16_t tim34;
	if (counter_buffer[4] > counter_buffer[3]) {
		tim34 = counter_buffer[4] - counter_buffer[3];
	}

	else if (counter_buffer[3] > counter_buffer[4]) {
		tim34 = (0xEA5F - counter_buffer[3]) + counter_buffer[4];
	}


	time_ms = ((float)(tim34 + tim23 + tim12 + tim01) / 59999) *10;
	w = 1/(0.001*time_ms);
	//convert time_s to uint_8
	if(time_ms < 0) {
		w_time[1] = 0;
	}
	//if over 40ms, timer didn't capture full period
	else if(time_ms > 40) {
		w_time[1] = 0xFF;
	}
	else
	{
		w_time[1] = (uint8_t)((time_ms / 40) * (float)(0xFF));
	}
	//convert w to uint_8
	if(w < 0) {
		w_time[0] = 0;
	}
	else if(w > 1000) {
		w_time[0] = 0xFF;
	}
	else
	{
		w_time[0] = (uint8_t)((w / 1000) * (float)(0xFF));
	}*/
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

	  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();/* Enable I-Cache---------------------------------------------------------*/

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM11_Init();
  MX_TIM10_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  //uint16_t duty_cycle1 = 333;
  //uint16_t duty_cycle2 = 500;
  //TIM10_Start_IC();
  Start_Diff_PWM();
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);

  //poll_speed(&htim10);
  //poll_speed(&htim11);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  move(1, 0);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	  //poll_speed(&htim10);
	  HAL_Delay(500);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2399;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 383;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 62499;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 15;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 59999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim10, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 15;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 59999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim11, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE11 PE12 PE13 PE14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
