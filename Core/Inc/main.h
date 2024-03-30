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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//TIM_HandleTypeDef htim7;

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
#define lora_DIO1_PIN_Pin GPIO_PIN_0
#define lora_DIO1_PIN_GPIO_Port GPIOC
#define lora_DIO1_PIN_EXTI_IRQn EXTI0_IRQn
#define lora_DIO2_PIN_Pin GPIO_PIN_1
#define lora_DIO2_PIN_GPIO_Port GPIOC
#define lora_DIO2_PIN_EXTI_IRQn EXTI1_IRQn
#define lora_DIO0_PIN_Pin GPIO_PIN_7
#define lora_DIO0_PIN_GPIO_Port GPIOC
#define lora_DIO0_PIN_EXTI_IRQn EXTI9_5_IRQn
#define lora_NSS_PIN_Pin GPIO_PIN_8
#define lora_NSS_PIN_GPIO_Port GPIOC
#define lora_Reset_PIN_Pin GPIO_PIN_9
#define lora_Reset_PIN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
