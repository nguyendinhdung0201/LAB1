/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
extern uint16_t step_position;
extern float percent_step;
extern volatile uint8_t state_motor_step;
extern volatile uint32_t last_time_step;
extern int16_t nhiet_do_bat_lam_mat;
extern int16_t nhiet_do_tat_lam_mat;
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
#define HOIVE_Pin GPIO_PIN_0
#define HOIVE_GPIO_Port GPIOA
#define DAUDAY_Pin GPIO_PIN_1
#define DAUDAY_GPIO_Port GPIOA
#define PL_Pin GPIO_PIN_2
#define PL_GPIO_Port GPIOA
#define PH_Pin GPIO_PIN_3
#define PH_GPIO_Port GPIOA
#define RUN_Pin GPIO_PIN_6
#define RUN_GPIO_Port GPIOA
#define RUN_Defrost_Pin GPIO_PIN_7
#define RUN_Defrost_GPIO_Port GPIOA
#define STEPPER_1_Pin GPIO_PIN_12
#define STEPPER_1_GPIO_Port GPIOB
#define STEPPER_2_Pin GPIO_PIN_13
#define STEPPER_2_GPIO_Port GPIOB
#define STEPPER_3_Pin GPIO_PIN_14
#define STEPPER_3_GPIO_Port GPIOB
#define STEPPER_4_Pin GPIO_PIN_15
#define STEPPER_4_GPIO_Port GPIOB
#define EWDG_Pin GPIO_PIN_5
#define EWDG_GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_8
#define RELAY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
