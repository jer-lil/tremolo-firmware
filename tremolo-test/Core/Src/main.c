/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lib/sm_bypass.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void led_toggle_tick(uint32_t, GPIO_TypeDef*, uint16_t);
void generate_triangle_wave(uint16_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct __attribute__((packed)) {
	uint16_t Rate;
	uint16_t Depth;
	uint16_t Shape;
	uint16_t Offset;
	uint16_t Subdiv;
	uint16_t Exp;
	uint16_t Trim1;
	uint16_t Trim2;
	uint16_t Vol;
} Adc;


uint16_t dma_wavetable[WAVETABLE_WIDTH] = {0};
uint16_t new_wavetable[WAVETABLE_WIDTH] = {0};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* Iniitalize state machines */
  StateBypassSw state_bypass_sw = STATE_IDLE;
  StateEffect state_effect = STATE_BYPASS;
  StateRelayMute state_relay_mute = STATE_BYPASS_UNMUTE;

  Adc* adc_raw;


  //int i = 0;
  //for (int i; i<ADC_DMA_BUF_LENGTH; i++)

  /* Call state machines to action on initial states */
  /* TODO can this be done before peripheral initialization? */

  /* Initialize wavetable into RAM (TODO) */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */


  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw,
		  ADC_DMA_BUF_LENGTH) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK)
  {
	  Error_Handler();
  }

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim16, TIM_CHANNEL_1);
  HAL_DMA_Start_IT(&hdma_tim16_ch1_up, (uint32_t)dma_wavetable, (uint32_t)&(TIM3->CCR1), WAVETABLE_WIDTH);
  __HAL_TIM_ENABLE_DMA(&htim16, TIM_DMA_CC1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



	  // Toggle heartbeat LED
	  led_toggle_tick(HEARTBEAT_MS, pDOUT_LED1_R_GPIO_Port, pDOUT_LED1_R_Pin);

	  // Check for bypass switch state and run state machine
	  EventBypassSw event_bypass_sw = EVENT_RELEASED;
	  if (!HAL_GPIO_ReadPin(pDIN_BYP_GPIO_Port, pDIN_BYP_Pin)){
		  event_bypass_sw = EVENT_PRESSED;
	  }
	  sm_bypass_sw(&state_bypass_sw, event_bypass_sw, &state_effect);

	  EventRelayMute event_relay_mute = EVENT_BYPASS;
	  if (state_effect == STATE_EFFECT){
		  event_relay_mute = EVENT_EFFECT;
	  }

	  sm_relay_mute(&state_relay_mute, event_relay_mute);

	  // Adjust Volume PWM Outputs based on Vol knob
	  // TODO move to function
	  //TIM3->CCR1 = adc_raw->Vol;
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, adc_raw->Vol);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, adc_raw->Vol);
	  //LL_TIM_OC_SetCompareCH1(htim2->Instance, vol);

	  // Generate new triangle wave based on latest depth input
	  // TODO accommodate different shapes
	  generate_triangle_wave(adc_raw->Depth);

	  //HAL_Delay(100);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM16
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM2
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV8;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_PLLCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_PLLCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

}


/* Toggles LED if it's been longer than timout_ms since last toggle*/
void led_toggle_tick(uint32_t timeout_ms, GPIO_TypeDef* LED_Port, uint16_t LED_Pin){
	static uint32_t last_toggle_ms = 0;
	uint32_t tick = HAL_GetTick();

	if (tick - last_toggle_ms >= timeout_ms){
		HAL_GPIO_TogglePin(LED_Port, LED_Pin);
		last_toggle_ms = tick;
	}
}

void generate_triangle_wave(uint16_t depth){

	uint32_t f_depth = depth << SHIFT_AMOUNT;
	uint32_t midpoint = WAVETABLE_WIDTH >> 1;
	uint32_t f_max = WAVETABLE_DEPTH << SHIFT_AMOUNT;
	uint32_t f_min = (WAVETABLE_DEPTH - depth) << SHIFT_AMOUNT;
	uint32_t f_step = f_depth / midpoint;

	uint32_t f_val = f_min;
	uint32_t val;

	for (int i=0; i<WAVETABLE_WIDTH; i++){
		if (i < WAVETABLE_WIDTH>>1){
			val = f_val >> SHIFT_AMOUNT;
			dma_wavetable[i] = val;
			f_val = f_val+f_step;
		}
		else if (i == (WAVETABLE_WIDTH>>1)){
			dma_wavetable[i] = f_max >> SHIFT_AMOUNT;
		}
		else{
			val = f_val >> SHIFT_AMOUNT;
			dma_wavetable[i] = val;
			f_val = f_val-f_step;
		}
	}
}

void check_HAL_states(){
	HAL_DMA_StateTypeDef adc_dma_state = HAL_DMA_GetState(hadc1.DMA_Handle);
	if (adc_dma_state == HAL_DMA_STATE_RESET){
		// DMA not properly initialized
		Error_Handler();
	}

	uint32_t adc_dma_error = HAL_DMA_GetError (hadc1.DMA_Handle);
	if (adc_dma_error != HAL_DMA_ERROR_NONE){
		// Some sort of DMA error
		Error_Handler();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
