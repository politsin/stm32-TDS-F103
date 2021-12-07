/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
void trans_to_usart1(char *buf);

void temp_ds18b20();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC1_33_Volt_Pin GPIO_PIN_0
#define ADC1_33_Volt_GPIO_Port GPIOA
#define ADC1_2_Volt_Pin GPIO_PIN_1
#define ADC1_2_Volt_GPIO_Port GPIOA
#define ADC1_NTC_Pin GPIO_PIN_2
#define ADC1_NTC_GPIO_Port GPIOA
#define ADC2_EC_Pin GPIO_PIN_5
#define ADC2_EC_GPIO_Port GPIOA
#define DIO_Pin GPIO_PIN_10
#define DIO_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_11
#define CLK_GPIO_Port GPIOB
#define DS18B20_Pin GPIO_PIN_11
#define DS18B20_GPIO_Port GPIOA
#define NTC_Power_Pin GPIO_PIN_8
#define NTC_Power_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_9
#define LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/////////////// ВИДИМОСТЬ //////////////
#define DEBUG_USART1       1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
