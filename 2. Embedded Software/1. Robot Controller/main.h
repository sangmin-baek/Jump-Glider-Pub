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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor_Dep_A_GPIO_Out_Pin GPIO_PIN_0
#define Motor_Dep_A_GPIO_Out_GPIO_Port GPIOC
#define Motor_Dep_B_GPIO_Out_Pin GPIO_PIN_1
#define Motor_Dep_B_GPIO_Out_GPIO_Port GPIOC
#define Motor_Fold_A_GPIO_Out_Pin GPIO_PIN_2
#define Motor_Fold_A_GPIO_Out_GPIO_Port GPIOC
#define Motor_Fold_B_GPIO_Out_Pin GPIO_PIN_3
#define Motor_Fold_B_GPIO_Out_GPIO_Port GPIOC
#define Motor_Jump_A_TIM2_CH1_PWM_Pin GPIO_PIN_0
#define Motor_Jump_A_TIM2_CH1_PWM_GPIO_Port GPIOA
#define Motor_Jump_B_TIM2_CH2_PWM_Pin GPIO_PIN_1
#define Motor_Jump_B_TIM2_CH2_PWM_GPIO_Port GPIOA
#define Angle_HJ_ADC1_IN5_Pin GPIO_PIN_5
#define Angle_HJ_ADC1_IN5_GPIO_Port GPIOA
#define Angle_Yaw_ADC1_IN6_Pin GPIO_PIN_6
#define Angle_Yaw_ADC1_IN6_GPIO_Port GPIOA
#define IMU_SPI1_MOSI_Pin GPIO_PIN_7
#define IMU_SPI1_MOSI_GPIO_Port GPIOA
#define IMU_CS_GPIO_Out_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Out_GPIO_Port GPIOC
#define nRF_USART3_RX_Pin GPIO_PIN_5
#define nRF_USART3_RX_GPIO_Port GPIOC
#define nRF_USART3_TX_Pin GPIO_PIN_10
#define nRF_USART3_TX_GPIO_Port GPIOB
#define Motor_Yaw_A_TIM3_CH1_PWM_Pin GPIO_PIN_6
#define Motor_Yaw_A_TIM3_CH1_PWM_GPIO_Port GPIOC
#define Motor_Yaw_B_TIM3_CH2_PWM_Pin GPIO_PIN_7
#define Motor_Yaw_B_TIM3_CH2_PWM_GPIO_Port GPIOC
#define Enc_A_TIM1_CH1_Pin GPIO_PIN_8
#define Enc_A_TIM1_CH1_GPIO_Port GPIOA
#define Enc_B_TIM1_CH2_Pin GPIO_PIN_9
#define Enc_B_TIM1_CH2_GPIO_Port GPIOA
#define SDIO_CD_Pin GPIO_PIN_11
#define SDIO_CD_GPIO_Port GPIOA
#define LED_GPIO_Out_Pin GPIO_PIN_12
#define LED_GPIO_Out_GPIO_Port GPIOA
#define IMU_SPI1_SCK_Pin GPIO_PIN_3
#define IMU_SPI1_SCK_GPIO_Port GPIOB
#define IMU_SPI1_MISO_Pin GPIO_PIN_4
#define IMU_SPI1_MISO_GPIO_Port GPIOB
#define Motor_HJ_A_TIM4_CH1_PWM_Pin GPIO_PIN_6
#define Motor_HJ_A_TIM4_CH1_PWM_GPIO_Port GPIOB
#define Motor_HJ_B_TIM4_CH2_PWM_Pin GPIO_PIN_7
#define Motor_HJ_B_TIM4_CH2_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


#define JUMPER_STATE_E_STORE	0x01
#define JUMPER_STATE_TRIG		0x02
#define JUMPER_STATE_INIT		0x03

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
