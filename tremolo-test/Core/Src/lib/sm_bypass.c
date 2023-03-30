/*
 * sm_bypass.c
 *
 *  Created on: Mar 28, 2023
 *      Author: jeremiah
 */

#include "lib/sm_bypass.h"
#include "stm32f3xx_hal.h"
#include "main.h"

void sm_bypass_sw(StateBypassSw *state_switch, EventBypassSw event, StateEffect *state_effect){

	static uint32_t debounce_start = 0;
	const uint32_t debounce_time_ms = 100;

	switch (*state_switch) {
		case STATE_IDLE:
			if (event == EVENT_PRESSED) {
				*state_switch = STATE_DEBOUNCE_PRESS;
				sm_effect(state_effect, EVENT_TOGGLE);
				debounce_start = HAL_GetTick();
			}
			break;

		case STATE_DEBOUNCE_PRESS:
			if (HAL_GetTick() - debounce_start >= debounce_time_ms) {
				*state_switch = STATE_WAIT_RELEASE;
			}
			break;

		case STATE_WAIT_RELEASE:
			if (event == EVENT_RELEASED) {
				debounce_start = HAL_GetTick();

				*state_switch = STATE_DEBOUNCE_RELEASE;
			}
			break;

		case STATE_DEBOUNCE_RELEASE:
			if (HAL_GetTick() - debounce_start >= debounce_time_ms) {
				*state_switch = STATE_IDLE;
			}
			break;

		default:
			*state_switch = STATE_IDLE;
			break;
	}
}

void sm_effect(StateEffect *state, EventEffect event){
	switch (*state) {
		case STATE_BYPASS:
			if (event == EVENT_TOGGLE){
				*state = STATE_EFFECT;
				HAL_GPIO_WritePin(pDOUT_BYP_GPIO_Port, pDOUT_BYP_Pin,
						GPIO_PIN_RESET);
			}
			break;

		case STATE_EFFECT:
			if (event == EVENT_TOGGLE){
				*state = STATE_BYPASS;
				HAL_GPIO_WritePin(pDOUT_BYP_GPIO_Port, pDOUT_BYP_Pin,
						GPIO_PIN_SET);
			}
			break;

		default:
			*state = STATE_BYPASS;
			break;
	}
}
