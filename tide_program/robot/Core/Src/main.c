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
#include "string.h"
#include "i2c-lcd.h"
#include "accel.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define BUFFER_SIZE 8 // 1 address, 6 for different expected data, 1 device status
#define BASE_STATE_BUSY 0
#define BASE_STATE_READY 1

#define TRIG_PORT_FRONT GPIOC
#define TRIG_PIN_FRONT GPIO_PIN_7

#define TRIG_PORT_BACK GPIOE
#define TRIG_PIN_BACK GPIO_PIN_8

#define TRIG_PORT_LEFT GPIOA
#define TRIG_PIN_LEFT GPIO_PIN_6

#define TRIG_PORT_RIGHT GPIOD
#define TRIG_PIN_RIGHT GPIO_PIN_13

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

#define error_t 10
#define pwm_period 2399
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// For device communication
uint8_t base_rx_buffer [BUFFER_SIZE] = {0};  // RX Buffer (for communication with Base)
uint8_t base_tx_buffer [BUFFER_SIZE] = {0};  // TX Buffer (for communication with Base)
volatile uint8_t base_state = BASE_STATE_BUSY;  // State of the base (initialize to busy)
uint8_t ready_confirm_string [BUFFER_SIZE - 2] = {'0', '0', 'r', 'r', 'd', 'y'};
uint8_t move_success_string [BUFFER_SIZE - 2] = {'0', '0', 's', 'u', 'c', 'c'};
uint8_t move_fail_string [BUFFER_SIZE - 2] = {'0', '0', 'f', 'a', 'i', 'l'};
// "Test" string ONLY
uint8_t test2 [6] = {'d', 'e', 'e', 'z', 'n', 't'};

// For ultrasonic sensor reading
uint32_t idx = 0;
uint32_t val1 = 0;
uint32_t val2 = 0;
uint32_t distance [4] = {0};
uint8_t is_first_captured = 0;  // in cm
uint32_t distance_threshold = 10;  // threshold is 10 cm
uint8_t detection_status [6] = {0};
/*
 * Sensor position breakdown:
 * distance[0] -> front
 * distance[1] -> back
 * distance[2] -> left
 * distance[3] -> right
 */

// For IMU
uint8_t imu_readings[LIN_ACC_NUMBER_OF_BYTES + EULER_NUMBER_OF_BYTES];
int16_t euler_heading_raw = 0;

//for motor controller
int facing = 0;
float tide_angle = 0;
int tim9_counter = 0;
int ic_overrun = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */

// For Device Communication
void Send_to_Base (uint8_t addr, uint8_t *data, uint8_t is_ready);
void Receive_from_Base(void);
void Interpret_Commands(uint8_t *rx_buffer);

// For Ultrasonics
void delay_in_us (uint16_t time, TIM_HandleTypeDef *htim);
void poll_ultrasonic (void);

// For Motors Controller
void Start_Diff_PWM(void);
void motors_controller(float on_r, float on_l);
void move(float x, float angle);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/************************ DEVICE COMMUNICATION (BLUETOOTH) ********************/

// Interpret commands
void Interpret_Commands(uint8_t *rx_buffer) {
	// Respond after establishing connection with Jetson
	if (strstr((const char * ) rx_buffer, "jrdy")) {
		Send_to_Base('0', ready_confirm_string, 1);
	}
	// Poll ultrasonic sensors
	else if (strstr((const char * ) rx_buffer, "poll")) {
		poll_ultrasonic();
	}
	// Start requested movement and rotation
	else if (strstr((const char * ) rx_buffer, "mve")) {
		// Define some variables
		float requested_heading;
		float do_move = 0;

		// Interpret requested movement
		if (rx_buffer[5] == '1') {
			do_move = 1;
		}
		else if (rx_buffer[5] == '0') {
			do_move = 0;
		}

		// Interpret requested rotation
		requested_heading = (float)(rx_buffer[6]) * 360 / 255;

		// MOVE!!
		poll_IMU(&hi2c2, imu_readings);
		euler_heading_raw = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
		tide_angle = ((float)(euler_heading_raw))/16.0f;

		move(do_move, requested_heading);
		Send_to_Base('0', move_success_string,1);

		// Verify current heading is close to requested heading
		/*if ((tide_angle >= requested_heading - 10) && ((tide_angle <= requested_heading + 10) || (requested_heading > 350))) {
			Send_to_Base('0', move_success_string, 1);
		}
		else {
			Send_to_Base('0', move_fail_string, 1);
		}*/
	}
}

// Function to send bytes to Base
void Send_to_Base (uint8_t addr, uint8_t *data, uint8_t is_ready) {
	while (base_state != BASE_STATE_READY) {}
	// Assign address
	base_tx_buffer[0] = addr;
	// Assign data (until before the last byte)
	for (int i = 1; i < BUFFER_SIZE - 1; i++) {
		base_tx_buffer[i] = data[i - 1];
	}
	// Assign status byte
	if (is_ready) {
		base_tx_buffer[BUFFER_SIZE - 1] = 'r';
	}
	else {
		base_tx_buffer[BUFFER_SIZE - 1] = 'n';
	}
	// Transmit data packet
	HAL_UART_Transmit_IT(&huart2, base_tx_buffer, BUFFER_SIZE);
}

// Function to Receive bytes from Base
void Receive_from_Base(void) {
	HAL_UART_Receive_IT(&huart2, base_rx_buffer, BUFFER_SIZE);
}

// Callback function after transmitting data
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart2.Instance) {
		// Transmission fully complete, now listen for any messages from base
		base_state = BASE_STATE_BUSY;
		Receive_from_Base();
	}
}

// Callback function after receiving data
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart2.Instance) {
		// Ready?
		if (base_rx_buffer[BUFFER_SIZE - 1] == 'r') {
			base_state = BASE_STATE_READY;
		}
		// Is this for Robot?
		if (base_rx_buffer[0] == '2') {
			// Interpret commands
			Interpret_Commands(base_rx_buffer);
		}
	}
}

/*************************** ULTRASONIC SENSOR READING ************************/

// Function that delays in microseconds (usec)
void delay_in_us (uint16_t time, TIM_HandleTypeDef *htim) {
	__HAL_TIM_SET_COUNTER(htim, 0);
	while (__HAL_TIM_GET_COUNTER(htim) <  time);
}


// Poll ultrasonic sensors
void poll_ultrasonic (void) {
	// Front Sensor
	idx = 0;
	HAL_GPIO_WritePin(TRIG_PORT_FRONT, TRIG_PIN_FRONT, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10, &htim3);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT_FRONT, TRIG_PIN_FRONT, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
	HAL_Delay(10);
	while(is_first_captured != 0);

	// Back Sensor
	idx = 1;

	HAL_GPIO_WritePin(TRIG_PORT_BACK, TRIG_PIN_BACK, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10, &htim1);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT_BACK, TRIG_PIN_BACK, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	HAL_Delay(10);
	while(is_first_captured != 0);

	// Left Sensor
	idx = 2;
	HAL_GPIO_WritePin(TRIG_PORT_LEFT, TRIG_PIN_LEFT, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10, &htim2);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT_LEFT, TRIG_PIN_LEFT, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
	HAL_Delay(10);
	while(is_first_captured != 0);

	// Right Sensor
	idx = 3;
	HAL_GPIO_WritePin(TRIG_PORT_RIGHT, TRIG_PIN_RIGHT, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_in_us(10, &htim4);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT_RIGHT, TRIG_PIN_RIGHT, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
	HAL_Delay(10);
	while(is_first_captured != 0);

	// Display Detection Status on LCD Screen
	lcd_put_cur(1, 2);
	lcd_send_data(detection_status[0]);
	lcd_put_cur(1, 6);
	lcd_send_data(detection_status[1]);
	lcd_put_cur(1, 10);
	lcd_send_data(detection_status[2]);
	lcd_put_cur(1, 14);
	lcd_send_data(detection_status[3]);

	// Set last bytes to '0'
	detection_status[4] = '0';
	detection_status[5] = '0';

	// Send detection status to Jetson
	Send_to_Base('0', detection_status, 1);
}


// Input Capture Callback Function
void HAL_TIM_IC_CaptureCallback (TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) { // If the interrupt source is channel 1
		if (is_first_captured == 0) { // If the first value is not captured
			val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // Read the first value
			is_first_captured = 1;
			// Change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (is_first_captured == 1) { // If the first is already captured
			uint32_t diff;
			val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // Read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // Reset the counter

			if (val2 > val1) {
				diff = val2 - val1; // diff is in microseconds
			}

			else if (val1 > val2) {
				diff = (0xffff - val1) + val2;  // diff is in microseconds
			}

			distance[idx] = diff * .034/2;  // UNITS BREAKDOWN: (10^(-6) s) * (10^4 m/s) = 10^(-2) m = cm -> distance is in cm
			is_first_captured = 0; // Set back to false

			// Set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);

			// Compare with distance threshold
			if (distance[idx] < distance_threshold) {
				detection_status[idx] = '1';  // obstacle detected!
			}
			else {
				detection_status[idx] = '0';  // no obstacle detected
			}
		}
	}
}

/************************ MOTOR CONTROLLER ********************/
void Start_Diff_PWM(void) {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
}

void move(float x, float angle) {
	/*while(fabs(angle - tide_angle) > error_t) {
		poll_IMU(&hi2c2, imu_readings);
		euler_heading_raw = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
		tide_angle = ((float)(euler_heading_raw))/16.0f;
		float angle_diff = fabs(angle - tide_angle);
		//assume within tolerance, not enough time in project to do otherwise
		//if angle difference is positive, turn right
		//if angle difference is negative, turn left
		if(angle_diff < 0) {
			while(angle_diff < error_t) {
				poll_IMU(&hi2c2, imu_readings);
				euler_heading_raw = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
				tide_angle = ((float)(euler_heading_raw))/16.0f;
				motors_controller(1, -1);
				angle_diff = fabs(angle - tide_angle);
			}
		}
		else if(angle_diff > 0) {
			while(angle_diff > error_t) {
				poll_IMU(&hi2c2, imu_readings);
				euler_heading_raw = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
				tide_angle = ((float)(euler_heading_raw))/16.0f;
				motors_controller(-1, 1);
				angle_diff = fabs(angle - tide_angle);
			}
		}
	}*/
	//calculated equation. assume x is in 6 inch intervals
	if(x == 1) {
		motors_controller(1,1);
		HAL_TIM_Base_Start_IT(&htim9);
		TIM9 -> CR1 |= 0b1;
		//wait for timer to overrun the 4.25 second period
		while(!ic_overrun);
		motors_controller(0,0);
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




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/*
	int16_t euler_data [3];
	float euler_heading, euler_roll, euler_pitch;
	float euler_heading_abs, euler_roll_abs, euler_pitch_abs;
	*/

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  Start_Diff_PWM();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  // Turn ON Debug LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);

  // Comment when not using LCD screen
  lcd_init();

  // lcd_send_string("TID-E - Team 14");

  // Display intial stuff
  lcd_send_string (" Px  Nx  Py  Ny");
  lcd_put_cur(1, 2);
  lcd_send_string("0   0   0   0");

  // Start and configure IMU
  BNO055_Init_I2C(&hi2c2);

  // Start receiving from base
  Receive_from_Base();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  poll_ultrasonic();
	  lcd_put_cur(1, 2);
	  lcd_send_data(detection_status[0]);
	  lcd_put_cur(1, 6);
	  lcd_send_data(detection_status[1]);
	  lcd_put_cur(1, 10);
	  lcd_send_data(detection_status[2]);
	  lcd_put_cur(1, 14);
	  lcd_send_data(detection_status[3]);
	  */

	  /*
	  HAL_Delay(100);
	  // Poll IMU
	  poll_IMU(&hi2c2, imu_readings);
	  euler_data[0] = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
	  euler_data[1] = (((int16_t)((uint8_t *)(imu_readings))[7] << 8) | ((uint8_t *)(imu_readings))[6]);
	  euler_data[2] = (((int16_t)((uint8_t *)(imu_readings))[9] << 8) | ((uint8_t *)(imu_readings))[8]);

	  // Get Euler angles in degrees
	  euler_heading = ((float)(euler_data[0]))/16.0f;
	  euler_roll = ((float)(euler_data[1]))/16.0f;
	  euler_pitch = ((float)(euler_data[2]))/16.0f;

	  // Get absolute magnitude of Euler angles
	  euler_heading_abs = (euler_heading < 0) ? -euler_heading : euler_heading;
	  euler_roll_abs = (euler_roll < 0) ? -euler_roll : euler_roll;
	  euler_pitch_abs = (euler_pitch < 0) ? -euler_pitch : euler_pitch;

	  // Print euler_heading
	  lcd_put_cur(1, 0);
	  lcd_send_string((euler_heading < 0) ? "-" : "+");
	  lcd_send_data((((int)(euler_heading_abs) / 100) % 10) + 48);   // 100th pos
	  lcd_send_data((((int)(euler_heading_abs) / 10) % 10) + 48);  // 10th pos
	  lcd_send_data(((int)(euler_heading_abs) % 10) + 48);  // 1st pos

	  // Print euler_roll
	  lcd_put_cur(1, 6);
	  lcd_send_string((euler_roll < 0) ? "-" : "+");
	  lcd_send_data((((int)(euler_roll_abs) / 100) % 10) + 48);   // 100th pos
	  lcd_send_data((((int)(euler_roll_abs) / 10) % 10) + 48);  // 10th pos
	  lcd_send_data(((int)(euler_roll_abs) % 10) + 48);  // 1st pos

	  // Print euler_pitch
	  lcd_put_cur(1, 12);
	  lcd_send_string((euler_pitch < 0) ? "-" : "+");
	  lcd_send_data((((int)(euler_pitch_abs) / 100) % 10) + 48);   // 100th pos
	  lcd_send_data((((int)(euler_pitch_abs) / 10) % 10) + 48);  // 10th pos
	  lcd_send_data(((int)(euler_pitch_abs) % 10) + 48);  // 1st pos
	  */

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
  RCC_OscInitStruct.PLL.PLLN = 192;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 96-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  /* USER CODE END TIM1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  /* USER CODE END TIM2_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 96-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff-1;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 96-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US2_TRIG_GPIO_Port, US2_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, US1_TRIG_Pin|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Debug_LED_GPIO_Port, Debug_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US4_TRIG_GPIO_Port, US4_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(US3_TRIG_GPIO_Port, US3_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BNO055_INT_Pin */
  GPIO_InitStruct.Pin = BNO055_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNO055_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : US2_TRIG_Pin */
  GPIO_InitStruct.Pin = US2_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US2_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : US1_TRIG_Pin PE11 PE12 PE13
                           PE14 */
  GPIO_InitStruct.Pin = US1_TRIG_Pin|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Debug_LED_Pin */
  GPIO_InitStruct.Pin = Debug_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Debug_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : US4_TRIG_Pin */
  GPIO_InitStruct.Pin = US4_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US4_TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : US3_TRIG_Pin */
  GPIO_InitStruct.Pin = US3_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(US3_TRIG_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
