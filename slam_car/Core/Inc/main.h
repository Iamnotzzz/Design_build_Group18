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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MPU6500_INT_Pin GPIO_PIN_0
#define MPU6500_INT_GPIO_Port GPIOC
#define MPU6500_INT_EXTI_IRQn EXTI0_IRQn
#define E1A_Pin GPIO_PIN_0
#define E1A_GPIO_Port GPIOA
#define E1B_Pin GPIO_PIN_1
#define E1B_GPIO_Port GPIOA
#define Motor_ADC_Pin GPIO_PIN_2
#define Motor_ADC_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_6
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_7
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_0
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_1
#define BIN2_GPIO_Port GPIOB
#define RADAR_TX_Pin GPIO_PIN_6
#define RADAR_TX_GPIO_Port GPIOC
#define RADAR_RX_Pin GPIO_PIN_7
#define RADAR_RX_GPIO_Port GPIOC
#define BLUETOOTH_TX_Pin GPIO_PIN_10
#define BLUETOOTH_TX_GPIO_Port GPIOC
#define BLUETOOTH_RX_Pin GPIO_PIN_11
#define BLUETOOTH_RX_GPIO_Port GPIOC
#define E2B_Pin GPIO_PIN_6
#define E2B_GPIO_Port GPIOB
#define E2A_Pin GPIO_PIN_7
#define E2A_GPIO_Port GPIOB
#define MPU6500_I2C_SCL_Pin GPIO_PIN_8
#define MPU6500_I2C_SCL_GPIO_Port GPIOB
#define MPU6500_I2C_SDA_Pin GPIO_PIN_9
#define MPU6500_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//#define MPU6050
#define MPU6500
#define PI              3.1415926535897932f

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
