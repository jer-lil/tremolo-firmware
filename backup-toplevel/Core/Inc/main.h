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
#include "stm32f0xx_hal.h"

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
#define LED_Green_Pin GPIO_PIN_13
#define LED_Green_GPIO_Port GPIOC
#define LED_Red_Pin GPIO_PIN_14
#define LED_Red_GPIO_Port GPIOC
#define LDRV_1_Pin GPIO_PIN_0
#define LDRV_1_GPIO_Port GPIOC
#define LDRV_2_Pin GPIO_PIN_1
#define LDRV_2_GPIO_Port GPIOC
#define LDRV_3_Pin GPIO_PIN_2
#define LDRV_3_GPIO_Port GPIOC
#define LDRV_4_Pin GPIO_PIN_3
#define LDRV_4_GPIO_Port GPIOC
#define RATE_Pin GPIO_PIN_0
#define RATE_GPIO_Port GPIOA
#define DEPTH_Pin GPIO_PIN_1
#define DEPTH_GPIO_Port GPIOA
#define SHAPE_Pin GPIO_PIN_2
#define SHAPE_GPIO_Port GPIOA
#define OFFSET_Pin GPIO_PIN_3
#define OFFSET_GPIO_Port GPIOA
#define SUBDIV_Pin GPIO_PIN_4
#define SUBDIV_GPIO_Port GPIOA
#define GP_AIN1_Pin GPIO_PIN_6
#define GP_AIN1_GPIO_Port GPIOA
#define EXP_Pin GPIO_PIN_4
#define EXP_GPIO_Port GPIOC
#define EXP_SW_Pin GPIO_PIN_5
#define EXP_SW_GPIO_Port GPIOC
#define BYPASS_Pin GPIO_PIN_0
#define BYPASS_GPIO_Port GPIOB
#define TAP_Pin GPIO_PIN_1
#define TAP_GPIO_Port GPIOB
#define HARM_SW_Pin GPIO_PIN_2
#define HARM_SW_GPIO_Port GPIOB
#define BYP_RELAY_Pin GPIO_PIN_10
#define BYP_RELAY_GPIO_Port GPIOB
#define SPI_DC_Pin GPIO_PIN_11
#define SPI_DC_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOC
#define PWM3_Pin GPIO_PIN_8
#define PWM3_GPIO_Port GPIOC
#define PWM4_Pin GPIO_PIN_9
#define PWM4_GPIO_Port GPIOC
#define SPI_RST_Pin GPIO_PIN_8
#define SPI_RST_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_11
#define SPI_CS_GPIO_Port GPIOA
#define GPIO_5_Pin GPIO_PIN_12
#define GPIO_5_GPIO_Port GPIOA
#define GPIO_6_Pin GPIO_PIN_15
#define GPIO_6_GPIO_Port GPIOA
#define LED_RGB_Red_Pin GPIO_PIN_10
#define LED_RGB_Red_GPIO_Port GPIOC
#define LED_RGB_Green_Pin GPIO_PIN_11
#define LED_RGB_Green_GPIO_Port GPIOC
#define LED_RGB_Blue_Pin GPIO_PIN_12
#define LED_RGB_Blue_GPIO_Port GPIOC
#define PAN_SW_Pin GPIO_PIN_3
#define PAN_SW_GPIO_Port GPIOB
#define GP_SW_1_Pin GPIO_PIN_4
#define GP_SW_1_GPIO_Port GPIOB
#define GP_SW_2_Pin GPIO_PIN_5
#define GP_SW_2_GPIO_Port GPIOB
#define GP_SW_3_Pin GPIO_PIN_6
#define GP_SW_3_GPIO_Port GPIOB
#define GP_SW_4_Pin GPIO_PIN_7
#define GP_SW_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

// 2^10 bits resolution gives a switching frequency of 23kHz at 48MHz clock speed
#define PWM_PERIOD 0x400
#define WAVETABLE_WIDTH 0x400


#define MIN_LFO_PERIOD																																																																																	`



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
