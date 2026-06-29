/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define IR7_Pin GPIO_PIN_0
#define IR7_GPIO_Port GPIOA
#define IR8_Pin GPIO_PIN_1
#define IR8_GPIO_Port GPIOA
#define IR9_Pin GPIO_PIN_3
#define IR9_GPIO_Port GPIOA
#define IR10_Pin GPIO_PIN_4
#define IR10_GPIO_Port GPIOA
#define IR11_Pin GPIO_PIN_5
#define IR11_GPIO_Port GPIOA
#define IR12_Pin GPIO_PIN_6
#define IR12_GPIO_Port GPIOA
#define IR13_Pin GPIO_PIN_7
#define IR13_GPIO_Port GPIOA
#define IR14_Pin GPIO_PIN_4
#define IR14_GPIO_Port GPIOC
#define IR15_Pin GPIO_PIN_5
#define IR15_GPIO_Port GPIOC
#define IR16_Pin GPIO_PIN_0
#define IR16_GPIO_Port GPIOB
#define IR17_Pin GPIO_PIN_1
#define IR17_GPIO_Port GPIOB
#define IR18_Pin GPIO_PIN_2
#define IR18_GPIO_Port GPIOB
#define IR19_Pin GPIO_PIN_11
#define IR19_GPIO_Port GPIOB
#define IR20_Pin GPIO_PIN_12
#define IR20_GPIO_Port GPIOB
#define IR21_Pin GPIO_PIN_13
#define IR21_GPIO_Port GPIOB
#define IR22_Pin GPIO_PIN_14
#define IR22_GPIO_Port GPIOB
#define IR23_Pin GPIO_PIN_15
#define IR23_GPIO_Port GPIOB
#define IR24_Pin GPIO_PIN_6
#define IR24_GPIO_Port GPIOC
#define IR1_Pin GPIO_PIN_12
#define IR1_GPIO_Port GPIOC
#define IR2_Pin GPIO_PIN_2
#define IR2_GPIO_Port GPIOD
#define IR3_Pin GPIO_PIN_4
#define IR3_GPIO_Port GPIOB
#define IR4_Pin GPIO_PIN_5
#define IR4_GPIO_Port GPIOB
#define IR5_Pin GPIO_PIN_6
#define IR5_GPIO_Port GPIOB
#define IR6_Pin GPIO_PIN_7
#define IR6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
