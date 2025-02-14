/*
 * sm_bypass.c
 *
 *  Created on: Mar 28, 2023
 *      Author: jeremiah
 *
 *  TODO maybe can simplify by making a struct that contains states/events
 */

#include "Lib/sm_bypass.h"
#include "stm32f3xx_hal.h"
#include "main.h"

// State instances
StateBypSw state_bypass_sw = SW_UNPRESSED;
StateEffect state_effect = EFF_BYP;
StateRelayMute state_relay_mute = EFF_BYP_MUTE;


EventEffect sm_byp_sw(EventSw input){

	// TODO put debounce time in macro?
	const uint32_t debounce_time_ms = 10;
	const uint32_t hold_time_ms = 1000;
	static uint32_t last_changed_ms = 0;

	uint32_t time_since_changed_ms = HAL_GetTick() - last_changed_ms;

	switch (state_bypass_sw) {
		case SW_UNPRESSED:
			if (input == SW_NEW_PRESS) {
				state_bypass_sw = SW_DEBOUNCE_PRESS;
				last_changed_ms = HAL_GetTick();
				return TOGGLE;
			}
			break;

		case SW_DEBOUNCE_PRESS:
			if (time_since_changed_ms >= debounce_time_ms) {
				state_bypass_sw = SW_PRESSED;
			}
			if (input == SW_NEW_RELEASE) {
				state_bypass_sw = SW_DEBOUNCE_RELEASE;
				last_changed_ms = HAL_GetTick();
			}
			break;

		case SW_PRESSED:
			if (input == SW_NEW_RELEASE) {
				last_changed_ms = HAL_GetTick();
				state_bypass_sw = SW_DEBOUNCE_RELEASE;
			}
			if (time_since_changed_ms >= hold_time_ms) {
				state_bypass_sw = SW_HELD;
			}
			break;
		case SW_HELD:
			if (input == SW_NEW_RELEASE) {
				last_changed_ms = HAL_GetTick();
				state_bypass_sw = SW_DEBOUNCE_RELEASE;
			}
			break;
		case SW_DEBOUNCE_RELEASE:
			if (time_since_changed_ms >= debounce_time_ms) {
				state_bypass_sw = SW_UNPRESSED;
			}
			if (input == SW_NEW_PRESS) {
				state_bypass_sw = SW_DEBOUNCE_PRESS;
				last_changed_ms = HAL_GetTick();
			}
			break;
		default:
			state_bypass_sw = SW_UNPRESSED;
			break;
	}
	return IDLE;
}

void sm_effect(EventEffect input){
	switch (state_effect) {
		case EFF_BYP:
			if (input == TOGGLE){
				state_effect = EFF_ON;

			}
			break;
		case EFF_ON:
			if (input == TOGGLE){
				state_effect = EFF_BYP;
			}
			break;

		default:
			state_effect = EFF_BYP;
			break;
	}
}

void sm_relay_mute(StateEffect input, LED* LED_bypass)
{
	static uint32_t mute_start = 0;
	const uint32_t mute_time_ms = 10;

	uint32_t time_since_changed_ms = HAL_GetTick() - mute_start;

	switch (state_relay_mute) {
		case EFF_BYP_UNMUTE:
			if (input == EFF_ON){
				state_relay_mute = EFF_BYP_MUTE;
				mute_start = HAL_GetTick();
				set_LED_state(LED_bypass, ON);
				HAL_GPIO_WritePin(pDOUT_MUTE_1_GPIO_Port, pDOUT_MUTE_1_Pin,
					GPIO_PIN_SET);
				HAL_GPIO_WritePin(pDOUT_MUTE_2_GPIO_Port, pDOUT_MUTE_2_Pin,
					GPIO_PIN_SET);
			}
			break;
		case EFF_BYP_MUTE:
			if (time_since_changed_ms >= mute_time_ms){
				if (input == EFF_BYP){
					state_relay_mute = EFF_BYP_UNMUTE;
					HAL_GPIO_WritePin(pDOUT_MUTE_1_GPIO_Port, pDOUT_MUTE_1_Pin,
						GPIO_PIN_RESET);
					HAL_GPIO_WritePin(pDOUT_MUTE_2_GPIO_Port, pDOUT_MUTE_2_Pin,
						GPIO_PIN_RESET);
				}
				else if (input == EFF_ON) {
					state_relay_mute = EFF_ON_MUTE;
					mute_start = HAL_GetTick();
					HAL_GPIO_WritePin(pDOUT_RLY_SET_GPIO_Port, pDOUT_RLY_SET_Pin,
						GPIO_PIN_SET);
				}
			}
 			break;
		case EFF_ON_MUTE:
			if (time_since_changed_ms >= mute_time_ms){
				if (input == EFF_BYP){
					state_relay_mute = EFF_BYP_MUTE;
					mute_start = HAL_GetTick();
					HAL_GPIO_WritePin(pDOUT_RLY_SET_GPIO_Port, pDOUT_RLY_SET_Pin,
						GPIO_PIN_RESET);
				}
				else if (input == EFF_ON){
					state_relay_mute = EFF_ON_UNMUTE;
					HAL_GPIO_WritePin(pDOUT_MUTE_1_GPIO_Port, pDOUT_MUTE_1_Pin,
						GPIO_PIN_RESET);
					HAL_GPIO_WritePin(pDOUT_MUTE_2_GPIO_Port, pDOUT_MUTE_2_Pin,
						GPIO_PIN_RESET);
				}
			}
			break;
		case EFF_ON_UNMUTE:
			if (input == EFF_BYP){
				state_relay_mute = EFF_ON_MUTE;
				mute_start = HAL_GetTick();
				set_LED_state(LED_bypass, OFF);
				HAL_GPIO_WritePin(pDOUT_MUTE_1_GPIO_Port, pDOUT_MUTE_1_Pin,
					GPIO_PIN_SET);
				HAL_GPIO_WritePin(pDOUT_MUTE_2_GPIO_Port, pDOUT_MUTE_2_Pin,
					GPIO_PIN_SET);
			}
			break;
		default:
			state_relay_mute = EFF_BYP_MUTE;
			break;
	}
}


