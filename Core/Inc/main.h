/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

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
/* K3: 6 каналов АЦП (2 датчика: low/high + спичка + внутр.24В) */
#define MCU_K3_NUM_ADC_CHANNEL 6
#define MCU_K3_FILTERSIZE      128

extern ADC_HandleTypeDef hadc1;
extern uint16_t MCU_K3_ADC_VAL[MCU_K3_NUM_ADC_CHANNEL];

uint16_t ADC_GetLimit1LowFiltered(void);
uint16_t ADC_GetLimit1HighFiltered(void);
uint16_t ADC_GetLimit2LowFiltered(void);
uint16_t ADC_GetLimit2HighFiltered(void);
uint16_t ADC_GetIgniterFiltered(void);
uint16_t ADC_GetU24Filtered(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LINE2_EN_Pin GPIO_PIN_3
#define LINE2_EN_GPIO_Port GPIOA
#define LINE1_EN_Pin GPIO_PIN_0
#define LINE1_EN_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
