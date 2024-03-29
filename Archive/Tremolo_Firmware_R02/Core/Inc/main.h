/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define TIM2_PERIOD 1023
#define TIM3_PERIOD 1023
#define pDOUT_MUTE_11_Pin GPIO_PIN_13
#define pDOUT_MUTE_11_GPIO_Port GPIOC
#define pDOUT_MUTE_2_Pin GPIO_PIN_14
#define pDOUT_MUTE_2_GPIO_Port GPIOC
#define pAIN_SUBDIV_Pin GPIO_PIN_0
#define pAIN_SUBDIV_GPIO_Port GPIOC
#define pAIN_EXP_Pin GPIO_PIN_1
#define pAIN_EXP_GPIO_Port GPIOC
#define pAIN_TRIM_1_Pin GPIO_PIN_2
#define pAIN_TRIM_1_GPIO_Port GPIOC
#define pAIN_TRIM_2_Pin GPIO_PIN_3
#define pAIN_TRIM_2_GPIO_Port GPIOC
#define pAIN_RATE_Pin GPIO_PIN_0
#define pAIN_RATE_GPIO_Port GPIOA
#define pAIN_DEPTH_Pin GPIO_PIN_1
#define pAIN_DEPTH_GPIO_Port GPIOA
#define pAIN_SHAPE_Pin GPIO_PIN_2
#define pAIN_SHAPE_GPIO_Port GPIOA
#define pAIN_OFFSET_Pin GPIO_PIN_3
#define pAIN_OFFSET_GPIO_Port GPIOA
#define pPWM_2_Pin GPIO_PIN_4
#define pPWM_2_GPIO_Port GPIOA
#define pPWM_VOL_1_Pin GPIO_PIN_5
#define pPWM_VOL_1_GPIO_Port GPIOA
#define pPWM_1_Pin GPIO_PIN_6
#define pPWM_1_GPIO_Port GPIOA
#define pDIN_HARM_MODE_1_Pin GPIO_PIN_7
#define pDIN_HARM_MODE_1_GPIO_Port GPIOA
#define pDIN_DIP_1_Pin GPIO_PIN_0
#define pDIN_DIP_1_GPIO_Port GPIOB
#define pDIN_DIP_2_Pin GPIO_PIN_1
#define pDIN_DIP_2_GPIO_Port GPIOB
#define pDIN_DIP_3_Pin GPIO_PIN_2
#define pDIN_DIP_3_GPIO_Port GPIOB
#define pDIN_HARM_MODE_2_Pin GPIO_PIN_10
#define pDIN_HARM_MODE_2_GPIO_Port GPIOB
#define pAIN_VOL_Pin GPIO_PIN_11
#define pAIN_VOL_GPIO_Port GPIOB
#define pDIN_PAN_MODE_2_Pin GPIO_PIN_12
#define pDIN_PAN_MODE_2_GPIO_Port GPIOB
#define pDIN_ENV_MODE_1_Pin GPIO_PIN_13
#define pDIN_ENV_MODE_1_GPIO_Port GPIOB
#define pDIN_ENV_MODE_2_Pin GPIO_PIN_14
#define pDIN_ENV_MODE_2_GPIO_Port GPIOB
#define pPWM_3_Pin GPIO_PIN_8
#define pPWM_3_GPIO_Port GPIOC
#define pPWM_4_Pin GPIO_PIN_9
#define pPWM_4_GPIO_Port GPIOC
#define pDIN_EXP_Pin GPIO_PIN_9
#define pDIN_EXP_GPIO_Port GPIOA
#define pDIN_DIP_4_Pin GPIO_PIN_10
#define pDIN_DIP_4_GPIO_Port GPIOA
#define pDIN_BYP_Pin GPIO_PIN_11
#define pDIN_BYP_GPIO_Port GPIOA
#define pDIN_TAP_Pin GPIO_PIN_12
#define pDIN_TAP_GPIO_Port GPIOA
#define pDOUT_RLY_SET_Pin GPIO_PIN_15
#define pDOUT_RLY_SET_GPIO_Port GPIOA
#define pDIN_JACK_SW_1_Pin GPIO_PIN_10
#define pDIN_JACK_SW_1_GPIO_Port GPIOC
#define pDIN_JACK_SW_2_Pin GPIO_PIN_11
#define pDIN_JACK_SW_2_GPIO_Port GPIOC
#define pDOUT_LED1_B_Pin GPIO_PIN_12
#define pDOUT_LED1_B_GPIO_Port GPIOC
#define pDOUT_LED1_G_Pin GPIO_PIN_2
#define pDOUT_LED1_G_GPIO_Port GPIOD
#define pPWM_VOL_2_Pin GPIO_PIN_3
#define pPWM_VOL_2_GPIO_Port GPIOB
#define pDOUT_LED1_R_Pin GPIO_PIN_4
#define pDOUT_LED1_R_GPIO_Port GPIOB
#define pDOUT_LED2_B_Pin GPIO_PIN_5
#define pDOUT_LED2_B_GPIO_Port GPIOB
#define pDOUT_LED2_G_Pin GPIO_PIN_6
#define pDOUT_LED2_G_GPIO_Port GPIOB
#define pDOUT_LED2_R_Pin GPIO_PIN_7
#define pDOUT_LED2_R_GPIO_Port GPIOB
#define pDIN_TAP_EXT_Pin GPIO_PIN_8
#define pDIN_TAP_EXT_GPIO_Port GPIOB
#define pDIN_HARM_MODE_1B9_Pin GPIO_PIN_9
#define pDIN_HARM_MODE_1B9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
