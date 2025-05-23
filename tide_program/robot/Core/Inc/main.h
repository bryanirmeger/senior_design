/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BNO055_SDA_Pin GPIO_PIN_0
#define BNO055_SDA_GPIO_Port GPIOF
#define BNO055_SCL_Pin GPIO_PIN_1
#define BNO055_SCL_GPIO_Port GPIOF
#define BNO055_INT_Pin GPIO_PIN_3
#define BNO055_INT_GPIO_Port GPIOF
#define BNO055_INT_EXTI_IRQn EXTI3_IRQn
#define US2_ECHO_Pin GPIO_PIN_5
#define US2_ECHO_GPIO_Port GPIOA
#define US2_TRIG_Pin GPIO_PIN_6
#define US2_TRIG_GPIO_Port GPIOA
#define US1_TRIG_Pin GPIO_PIN_8
#define US1_TRIG_GPIO_Port GPIOE
#define US1_ECHO_Pin GPIO_PIN_9
#define US1_ECHO_GPIO_Port GPIOE
#define Debug_LED_Pin GPIO_PIN_12
#define Debug_LED_GPIO_Port GPIOB
#define US4_ECHO_Pin GPIO_PIN_12
#define US4_ECHO_GPIO_Port GPIOD
#define US4_TRIG_Pin GPIO_PIN_13
#define US4_TRIG_GPIO_Port GPIOD
#define US3_ECHO_Pin GPIO_PIN_6
#define US3_ECHO_GPIO_Port GPIOC
#define US3_TRIG_Pin GPIO_PIN_7
#define US3_TRIG_GPIO_Port GPIOC
#define Bluetooth_TX_Pin GPIO_PIN_5
#define Bluetooth_TX_GPIO_Port GPIOD
#define Bluetooth_RX_Pin GPIO_PIN_6
#define Bluetooth_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
