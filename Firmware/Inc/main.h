/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l0xx_hal.h"

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
#define ADC_BAT_Pin GPIO_PIN_0
#define ADC_BAT_GPIO_Port GPIOA
#define ADS_SOIL1_Pin GPIO_PIN_1
#define ADS_SOIL1_GPIO_Port GPIOA
#define ADC_SOIL2_Pin GPIO_PIN_2
#define ADC_SOIL2_GPIO_Port GPIOA
#define ADC_SOIL3_Pin GPIO_PIN_3
#define ADC_SOIL3_GPIO_Port GPIOA
#define ADC_SOIL4_Pin GPIO_PIN_4
#define ADC_SOIL4_GPIO_Port GPIOA
#define PWM_LED1_Pin GPIO_PIN_5
#define PWM_LED1_GPIO_Port GPIOA
#define PWM_LED2_Pin GPIO_PIN_6
#define PWM_LED2_GPIO_Port GPIOA
#define PWM_LED3_Pin GPIO_PIN_7
#define PWM_LED3_GPIO_Port GPIOA
#define PWM_LED4_Pin GPIO_PIN_0
#define PWM_LED4_GPIO_Port GPIOB
#define SENS_ENA_Pin GPIO_PIN_1
#define SENS_ENA_GPIO_Port GPIOB
#define BAT_LO_Pin GPIO_PIN_8
#define BAT_LO_GPIO_Port GPIOA
#define OUT4_Pin GPIO_PIN_12
#define OUT4_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_15
#define OUT3_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_3
#define OUT2_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_4
#define OUT1_GPIO_Port GPIOB
#define COMM_ENA_Pin GPIO_PIN_5
#define COMM_ENA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
