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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MR_LCD_Pin GPIO_PIN_2
#define MR_LCD_GPIO_Port GPIOB
#define dig_3_Pin GPIO_PIN_10
#define dig_3_GPIO_Port GPIOB
#define dig_2_Pin GPIO_PIN_11
#define dig_2_GPIO_Port GPIOB
#define dig_1_Pin GPIO_PIN_12
#define dig_1_GPIO_Port GPIOB
#define U_AB_Pin GPIO_PIN_8
#define U_AB_GPIO_Port GPIOA
#define U_ac_dc_Pin GPIO_PIN_9
#define U_ac_dc_GPIO_Port GPIOA
#define U_AB_29_Pin GPIO_PIN_10
#define U_AB_29_GPIO_Port GPIOA
#define U_AB_24_Pin GPIO_PIN_11
#define U_AB_24_GPIO_Port GPIOA
#define U_ac_dc_0_Pin GPIO_PIN_12
#define U_ac_dc_0_GPIO_Port GPIOA
#define T_high_Pin GPIO_PIN_15
#define T_high_GPIO_Port GPIOA
#define I_mor_20_Pin GPIO_PIN_3
#define I_mor_20_GPIO_Port GPIOB
#define Opt_Buffer_Pin GPIO_PIN_4
#define Opt_Buffer_GPIO_Port GPIOB
#define Discharge_AB_Pin GPIO_PIN_5
#define Discharge_AB_GPIO_Port GPIOB
#define Overpower_Pin GPIO_PIN_6
#define Overpower_GPIO_Port GPIOB
#define NetOn_Pin GPIO_PIN_7
#define NetOn_GPIO_Port GPIOB
#define button_Pin GPIO_PIN_8
#define button_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_9
#define buzzer_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
