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
#include "stm32c0xx_hal.h"

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
#define LCD_TEAR_Pin GPIO_PIN_0
#define LCD_TEAR_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOA
#define LCD_SCK_Pin GPIO_PIN_5
#define LCD_SCK_GPIO_Port GPIOA
#define LCD_MISO_Pin GPIO_PIN_6
#define LCD_MISO_GPIO_Port GPIOA
#define LCD_MOSI_Pin GPIO_PIN_7
#define LCD_MOSI_GPIO_Port GPIOA
#define CTP_RST_Pin GPIO_PIN_5
#define CTP_RST_GPIO_Port GPIOC
#define MEM_SCK_Pin GPIO_PIN_13
#define MEM_SCK_GPIO_Port GPIOB
#define MEM_MISO_Pin GPIO_PIN_14
#define MEM_MISO_GPIO_Port GPIOB
#define MEM_MOSI_Pin GPIO_PIN_15
#define MEM_MOSI_GPIO_Port GPIOB
#define MEM_CS_Pin GPIO_PIN_8
#define MEM_CS_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_9
#define LCD_CS_GPIO_Port GPIOA
#define CTP_INT_Pin GPIO_PIN_10
#define CTP_INT_GPIO_Port GPIOA
#define CTP_INT_EXTI_IRQn EXTI4_15_IRQn
#define LCD_WRX_Pin GPIO_PIN_8
#define LCD_WRX_GPIO_Port GPIOC
#define LCD_BACKLIGHT_Pin GPIO_PIN_4
#define LCD_BACKLIGHT_GPIO_Port GPIOB
#define CTP_SCL_Pin GPIO_PIN_8
#define CTP_SCL_GPIO_Port GPIOB
#define CTP_SDA_Pin GPIO_PIN_9
#define CTP_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
