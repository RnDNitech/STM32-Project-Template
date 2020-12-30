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
#define D2_Pin GPIO_PIN_2
#define D2_GPIO_Port GPIOE
#define D3_Pin GPIO_PIN_3
#define D3_GPIO_Port GPIOE
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOE
#define EN_Pin GPIO_PIN_2
#define EN_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOE
#define RW_Pin GPIO_PIN_10
#define RW_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_11
#define RS_GPIO_Port GPIOB
#define LED_5_Pin GPIO_PIN_11
#define LED_5_GPIO_Port GPIOD
#define LED_4_Pin GPIO_PIN_12
#define LED_4_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_13
#define LED_3_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_14
#define LED_2_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOD
#define Key_1_Pin GPIO_PIN_3
#define Key_1_GPIO_Port GPIOD
#define Key_2_Pin GPIO_PIN_4
#define Key_2_GPIO_Port GPIOD
#define Key_3_Pin GPIO_PIN_5
#define Key_3_GPIO_Port GPIOD
#define Key_4_Pin GPIO_PIN_6
#define Key_4_GPIO_Port GPIOD
#define Key_5_Pin GPIO_PIN_7
#define Key_5_GPIO_Port GPIOD
#define D0_Pin GPIO_PIN_0
#define D0_GPIO_Port GPIOE
#define D1_Pin GPIO_PIN_1
#define D1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
