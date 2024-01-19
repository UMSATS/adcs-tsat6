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
#include "stm32l4xx_hal.h"

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
#define WDI_Pin GPIO_PIN_0
#define WDI_GPIO_Port GPIOC
#define M_nRESET_Pin GPIO_PIN_1
#define M_nRESET_GPIO_Port GPIOC
#define PCB_TEMP_ADC_IN_Pin GPIO_PIN_2
#define PCB_TEMP_ADC_IN_GPIO_Port GPIOC
#define PWMSW1_Pin GPIO_PIN_0
#define PWMSW1_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_1
#define PWM1_GPIO_Port GPIOA
#define PWMSW2_Pin GPIO_PIN_2
#define PWMSW2_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOA
#define PWMSW3_Pin GPIO_PIN_4
#define PWMSW3_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_5
#define PWM3_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_11
#define LED3_GPIO_Port GPIOB
#define GYR2_nCS_Pin GPIO_PIN_12
#define GYR2_nCS_GPIO_Port GPIOB
#define GYR1_nCS_Pin GPIO_PIN_6
#define GYR1_nCS_GPIO_Port GPIOC
#define GYR2_INT1_Pin GPIO_PIN_7
#define GYR2_INT1_GPIO_Port GPIOC
#define GYR2_INT1_EXTI_IRQn EXTI9_5_IRQn
#define GYR2_INT2_Pin GPIO_PIN_8
#define GYR2_INT2_GPIO_Port GPIOA
#define GYR2_INT2_EXTI_IRQn EXTI9_5_IRQn
#define GYR1_INT1_Pin GPIO_PIN_9
#define GYR1_INT1_GPIO_Port GPIOA
#define GYR1_INT1_EXTI_IRQn EXTI9_5_IRQn
#define GYR1_INT2_Pin GPIO_PIN_10
#define GYR1_INT2_GPIO_Port GPIOA
#define MAG2_nCS_Pin GPIO_PIN_15
#define MAG2_nCS_GPIO_Port GPIOA
#define MAG1_nCS_Pin GPIO_PIN_2
#define MAG1_nCS_GPIO_Port GPIOD
#define MAG1_INT_Pin GPIO_PIN_5
#define MAG1_INT_GPIO_Port GPIOB
#define MAG1_INT_EXTI_IRQn EXTI9_5_IRQn
#define MAG2_INT_Pin GPIO_PIN_6
#define MAG2_INT_GPIO_Port GPIOB
#define MAG2_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
