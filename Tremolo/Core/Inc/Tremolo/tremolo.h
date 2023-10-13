/*
 * tremolo.h
 *
 *  Created on: Oct 13, 2023
 *      Author: jeremiah
 */

#ifndef INC_TREMOLO_TREMOLO_H_
#define INC_TREMOLO_TREMOLO_H_

#include <main.h>
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


// TODO clean these up, rename
#define TIM2_PERIOD 1023
#define TIM3_PERIOD 1023
#define WAVETABLE_WIDTH 1024
#define WAVETABLE_DEPTH 1023
#define ADC_RESOLUTION 1023

#define ADC_DMA_BUF_LENGTH 9

/* Toggle heartbeat LED every 500ms */
#define HEARTBEAT_MS 500

/*
 *  Handle Assignment
 */


// Volume pwm channels
#define HTIM_VOL_A htim2
#define TIM_CH_VOL_A TIM_CHANNEL_1
#define HTIM_VOL_B htim2
#define TIM_CH_VOL_B TIM_CHANNEL_2
// LFO PWM timers
#define HTIM_PWM_A_LO htim3
#define TIM_CH_PWM_A_LO TIM_CHANNEL_1
#define HTIM_PWM_A_HI htim3
#define TIM_CH_PWM_A_HI TIM_CHANNEL_2
#define HTIM_PWM_B_LO htim3
#define TIM_CH_PWM_B_LO TIM_CHANNEL_3
#define HTIM_PWM_B_HI htim3
#define TIM_CH_PWM_B_HI TIM_CHANNEL_4

// DMA destinations, where wavetable values are loaded into
#define DMA_DST_PWM_A_LO (TIM3->CCR1)
#define DMA_DST_PWM_A_HI (TIM3->CCR2)
#define DMA_DST_PWM_B_LO (TIM3->CCR3)
#define DMA_DST_PWM_B_HI (TIM3->CCR4)

// LFO Waveform timers
#define HTIM_WVFM_A_LO htim8
#define TIM_CH_WVFM_A_LO TIM_CHANNEL_1
#define HTIM_WVFM_A_HI htim8
#define TIM_CH_WVFM_A_HI TIM_CHANNEL_2
#define HTIM_WVFM_B_LO htim8
#define TIM_CH_WVFM_B_LO TIM_CHANNEL_3
#define HTIM_WVFM_B_HI htim8
#define TIM_CH_WVFM_B_HI TIM_CHANNEL_4

// Waveform timer DMA handles
#define HDMA_WVFM_A_LO hdma_tim8_ch1
#define HDMA_WVFM_A_HI hdma_tim8_ch2
#define HDMA_WVFM_B_LO hdma_tim8_ch3_up
#define HDMA_WVFM_B_HI hdma_tim8_ch4_trig_com
// Waveform timer DMA trigger sources
#define DMA_TRIG_WVFM_A_LO TIM_DMA_CC1
#define DMA_TRIG_WVFM_A_HI TIM_DMA_CC2
#define DMA_TRIG_WVFM_B_LO TIM_DMA_CC3
#define DMA_TRIG_WVFM_B_HI TIM_DMA_CC4

// UART Handles
#define HUART huart1



#endif /* INC_TREMOLO_TREMOLO_H_ */
