/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define SYNC0_Pin GPIO_PIN_0
#define SYNC0_GPIO_Port GPIOA
#define SYNC0_EXTI_IRQn EXTI0_IRQn
#define SYNC1_Pin GPIO_PIN_1
#define SYNC1_GPIO_Port GPIOA
#define SYNC1_EXTI_IRQn EXTI1_IRQn
#define LED7_Pin GPIO_PIN_4
#define LED7_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_5
#define LED6_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_0
#define LED5_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED0_Pin GPIO_PIN_14
#define LED0_GPIO_Port GPIOB
#define INPUT7_Pin GPIO_PIN_15
#define INPUT7_GPIO_Port GPIOB
#define INPUT6_Pin GPIO_PIN_6
#define INPUT6_GPIO_Port GPIOC
#define INPUT5_Pin GPIO_PIN_7
#define INPUT5_GPIO_Port GPIOC
#define INPUT4_Pin GPIO_PIN_8
#define INPUT4_GPIO_Port GPIOC
#define INPUT3_Pin GPIO_PIN_9
#define INPUT3_GPIO_Port GPIOC
#define SPI_IRQ_Pin GPIO_PIN_8
#define SPI_IRQ_GPIO_Port GPIOA
#define SPI_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define INPUT2_Pin GPIO_PIN_10
#define INPUT2_GPIO_Port GPIOC
#define INPUT1_Pin GPIO_PIN_11
#define INPUT1_GPIO_Port GPIOC
#define INPUT0_Pin GPIO_PIN_12
#define INPUT0_GPIO_Port GPIOC
#define SPI_CS_Pin GPIO_PIN_8
#define SPI_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
