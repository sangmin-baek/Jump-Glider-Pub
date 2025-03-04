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
#define Joy_x_ADC1_IN6_Pin GPIO_PIN_6
#define Joy_x_ADC1_IN6_GPIO_Port GPIOA
#define Joy_y_ADC1_IN7_Pin GPIO_PIN_7
#define Joy_y_ADC1_IN7_GPIO_Port GPIOA
#define SW5_EXTI5_Pin GPIO_PIN_5
#define SW5_EXTI5_GPIO_Port GPIOC
#define POT_ADC1_IN8_Pin GPIO_PIN_0
#define POT_ADC1_IN8_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOB
#define SW4_EXTI6_Pin GPIO_PIN_6
#define SW4_EXTI6_GPIO_Port GPIOC
#define SW3_EXTI7_Pin GPIO_PIN_7
#define SW3_EXTI7_GPIO_Port GPIOC
#define SW2_EXTI8_Pin GPIO_PIN_8
#define SW2_EXTI8_GPIO_Port GPIOC
#define SW1_EXTI9_Pin GPIO_PIN_9
#define SW1_EXTI9_GPIO_Port GPIOC
#define SW_DIP_4_Input_Pin GPIO_PIN_10
#define SW_DIP_4_Input_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_12
#define LED5_GPIO_Port GPIOA
#define SW_DIP_3_Input_Pin GPIO_PIN_3
#define SW_DIP_3_Input_GPIO_Port GPIOB
#define SW_DIP_2_Input_Pin GPIO_PIN_4
#define SW_DIP_2_Input_GPIO_Port GPIOB
#define SW_DIP_1_Input_Pin GPIO_PIN_5
#define SW_DIP_1_Input_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
