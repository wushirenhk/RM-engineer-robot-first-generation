/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define Snail7_Pin GPIO_PIN_7
#define Snail7_GPIO_Port GPIOI
#define Snail6_Pin GPIO_PIN_6
#define Snail6_GPIO_Port GPIOI
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOC
#define IST8310_RSTN_Pin GPIO_PIN_6
#define IST8310_RSTN_GPIO_Port GPIOG
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define IST8310_DRDY_Pin GPIO_PIN_3
#define IST8310_DRDY_GPIO_Port GPIOG
#define IST8310_DRDY_EXTI_IRQn EXTI3_IRQn
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define KEY_EXTI_IRQn EXTI0_IRQn
#define BMI088_ACCEL_CS1_Pin GPIO_PIN_4
#define BMI088_ACCEL_CS1_GPIO_Port GPIOA
#define BMI088_ACCEL_INT1_Pin GPIO_PIN_4
#define BMI088_ACCEL_INT1_GPIO_Port GPIOC
#define BMI088_ACCEL_INT1_EXTI_IRQn EXTI4_IRQn
#define AIR_PUMP1_Pin GPIO_PIN_13
#define AIR_PUMP1_GPIO_Port GPIOE
#define BMI088_GYRO_INT1_Pin GPIO_PIN_5
#define BMI088_GYRO_INT1_GPIO_Port GPIOC
#define BMI088_GYRO_INT1_EXTI_IRQn EXTI9_5_IRQn
#define PITCH_Servo_Pin GPIO_PIN_9
#define PITCH_Servo_GPIO_Port GPIOE
#define AIR_PUMP3_Pin GPIO_PIN_11
#define AIR_PUMP3_GPIO_Port GPIOE
#define AIR_PUMP2_Pin GPIO_PIN_14
#define AIR_PUMP2_GPIO_Port GPIOE
#define SPI2_NCS_Pin GPIO_PIN_12
#define SPI2_NCS_GPIO_Port GPIOB
#define BMI088_GYRO_CS1_Pin GPIO_PIN_0
#define BMI088_GYRO_CS1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
