/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
void enter_sleep_mode(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define power_saving_light_Pin GPIO_PIN_13
#define power_saving_light_GPIO_Port GPIOC
#define timer_light_Pin GPIO_PIN_4
#define timer_light_GPIO_Port GPIOA
#define door_light_Pin GPIO_PIN_5
#define door_light_GPIO_Port GPIOA
#define start_button_Pin GPIO_PIN_0
#define start_button_GPIO_Port GPIOB
#define start_button_EXTI_IRQn EXTI0_IRQn
#define decrement_button_Pin GPIO_PIN_1
#define decrement_button_GPIO_Port GPIOB
#define decrement_button_EXTI_IRQn EXTI1_IRQn
#define power_saving_button_Pin GPIO_PIN_11
#define power_saving_button_GPIO_Port GPIOB
#define power_saving_button_EXTI_IRQn EXTI15_10_IRQn
#define LCD_D4_Pin GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_13
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_11
#define LCD_RS_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_12
#define LCD_EN_GPIO_Port GPIOA
#define reset_button_Pin GPIO_PIN_3
#define reset_button_GPIO_Port GPIOB
#define reset_button_EXTI_IRQn EXTI3_IRQn
#define increment_button_Pin GPIO_PIN_4
#define increment_button_GPIO_Port GPIOB
#define increment_button_EXTI_IRQn EXTI4_IRQn
#define test_light_Pin GPIO_PIN_8
#define test_light_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
