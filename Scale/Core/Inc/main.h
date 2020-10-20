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
#include "lcd1602.h"
#include "hx711.h"
#include "calibration.h"
#include <string.h>
#include "bluetooth.h"
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
#define ALERM_LIGHT_Pin GPIO_PIN_0
#define ALERM_LIGHT_GPIO_Port GPIOA
#define calibration_Pin GPIO_PIN_1
#define calibration_GPIO_Port GPIOA
#define calibration_EXTI_IRQn EXTI1_IRQn
#define HX711_ADDO_Pin GPIO_PIN_3
#define HX711_ADDO_GPIO_Port GPIOA
#define HX711_ADSK_Pin GPIO_PIN_4
#define HX711_ADSK_GPIO_Port GPIOA
#define LCD_WR_Pin GPIO_PIN_5
#define LCD_WR_GPIO_Port GPIOA
#define LCD_RD_Pin GPIO_PIN_6
#define LCD_RD_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_7
#define LCD_E_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_15
#define LCD_D0_GPIO_Port GPIOA
#define LCD_D1_Pin GPIO_PIN_3
#define LCD_D1_GPIO_Port GPIOB
#define LCD_D2_Pin GPIO_PIN_4
#define LCD_D2_GPIO_Port GPIOB
#define LCD_D3_Pin GPIO_PIN_5
#define LCD_D3_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_6
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_7
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_8
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_9
#define LCD_D7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
