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
StateBypSw state_tap_sw = SW_UNPRESSED;
StateEffect state_effect = EFF_BYP;
StateRelayMute state_relay_mute = EFF_BYP_MUTE;

#define TAP_LOG_SIZE 16


uint32_t tap_avg = 0;
// TODO move this into add_new_tap, just global for debugging
//uint32_t tap_diffs[TAP_LOG_SIZE] = {0};

void add_first_tap(uint32_t new_tap_us){
	return;
}

void add_new_tap(uint32_t new_tap_us, bool is_first_tap){
	//uint32_t tap_logs[TAP_LOG_SIZE] = {0};
	static uint32_t tap_diffs[TAP_LOG_SIZE] = {0};
	static uint32_t tap_index = 0;
	static uint32_t num_taps = 0;
	static uint32_t last_tap_us;

	uint32_t tap_sum = 0;

	if (is_first_tap) {
		for (int i=0; i<TAP_LOG_SIZE; i++){
			tap_diffs[i] = 0;
		}
		tap_avg = 0;
		tap_index = 0;
		num_taps = 1;
		last_tap_us = new_tap_us;
	}
	else {
		// Log new tap and save timestamp
		tap_diffs[tap_index] = new_tap_us - last_tap_us;
		last_tap_us = new_tap_us;
		// compute average

		uint32_t num_valid_taps = min(num_taps, TAP_LOG_SIZE);
		uint32_t new_diff = 0;
		for (int i=0; i<num_valid_taps; i++){
			new_diff = tap_diffs[i];
			tap_sum = tap_sum + new_diff;
		}
		tap_avg = (uint32_t) (tap_sum / (num_valid_taps));
		tap_index = (tap_index + 1) % TAP_LOG_SIZE;
		num_taps++;
	}
	return;
}


EventSwOutput sm_byp_sw(EventSw input){

	// TODO put debounce time in macro?
	const uint32_t debounce_time_ms = 10;
	const uint32_t hold_time_ms = 1000;
	static uint32_t last_changed_ms = 0;

	uint32_t now_ms = HAL_GetTick();
	uint32_t time_since_changed_ms = now_ms - last_changed_ms;

	switch (state_bypass_sw) {
		case SW_UNPRESSED:
			if (input == SW_NEW_PRESS) {
				state_bypass_sw = SW_DEBOUNCE_PRESS;
				last_changed_ms = now_ms;
				return TRIGGER_FIRST;
			}
			break;

		case SW_DEBOUNCE_PRESS:
			if (time_since_changed_ms >= debounce_time_ms) {
				state_bypass_sw = SW_PRESSED;
			}
			if (input == SW_NEW_RELEASE) {
				state_bypass_sw = SW_DEBOUNCE_RELEASE;
				last_changed_ms = now_ms;
			}
			break;

		case SW_PRESSED:
			if (input == SW_NEW_RELEASE) {
				last_changed_ms = now_ms;
				state_bypass_sw = SW_DEBOUNCE_RELEASE;
			}
			if (time_since_changed_ms >= hold_time_ms) {
				state_bypass_sw = SW_PRESSED_HELD;
			}
			break;
		case SW_DEBOUNCE_RELEASE:
			if (time_since_changed_ms >= debounce_time_ms) {
				state_bypass_sw = SW_UNPRESSED;
			}
			if (input == SW_NEW_PRESS) {
				state_bypass_sw = SW_DEBOUNCE_PRESS;
				last_changed_ms = now_ms;
			}
			break;
		case SW_PRESSED_HELD:
			if (input == SW_NEW_RELEASE) {
				last_changed_ms = now_ms;
				state_bypass_sw = SW_DEBOUNCE_RELEASE;
			}
			break;
		default:
			state_bypass_sw = SW_UNPRESSED;
			break;
	}
	return IDLE;
}

EventSwOutput sm_tap_sw(EventSw input, uint32_t new_tap_us){

	// TODO put debounce time in macro?
	const uint32_t debounce_time_ms = 10;
	const uint32_t hold_time_ms = 1000;
	static uint32_t last_changed_ms = 0;

	uint32_t now_ms = HAL_GetTick();
	uint32_t time_since_changed_ms = now_ms - last_changed_ms;

	switch (state_tap_sw) {
		case SW_UNPRESSED:
			if (input == SW_NEW_PRESS) {
				state_tap_sw = SW_DEBOUNCE_PRESS;
				last_changed_ms = now_ms;
				add_new_tap(new_tap_us, false);
				return TRIGGER_FIRST;
			}
			if (time_since_changed_ms >= hold_time_ms) {
				state_tap_sw = SW_UNPRESSED_HELD;
			}
			break;
		case SW_UNPRESSED_HELD:
			if (input == SW_NEW_PRESS) {
				state_tap_sw = SW_DEBOUNCE_PRESS;
				last_changed_ms = now_ms;
				add_new_tap(new_tap_us, true);
				// TODO sync waveform??
				return TRIGGER_CONTINUOUS;
			}
			break;
		case SW_DEBOUNCE_PRESS:
			if (input == SW_NEW_RELEASE) {
				state_tap_sw = SW_DEBOUNCE_RELEASE;
				last_changed_ms = now_ms;
			}
			if (time_since_changed_ms >= debounce_time_ms) {
				state_tap_sw = SW_PRESSED;
			}
			break;

		case SW_PRESSED:
			if (input == SW_NEW_RELEASE) {
				last_changed_ms = now_ms;
				state_tap_sw = SW_DEBOUNCE_RELEASE;
			}
			if (time_since_changed_ms >= hold_time_ms) {
				state_tap_sw = SW_PRESSED_HELD;
			}
			break;
		case SW_DEBOUNCE_RELEASE:
			if (input == SW_NEW_PRESS) {
				state_tap_sw = SW_DEBOUNCE_PRESS;
				last_changed_ms = now_ms;
			}
			if (time_since_changed_ms >= debounce_time_ms) {
				state_tap_sw = SW_UNPRESSED;
			}
			break;
		case SW_PRESSED_HELD:
			if (input == SW_NEW_RELEASE) {
				last_changed_ms = now_ms;
				state_tap_sw = SW_DEBOUNCE_RELEASE;
			}
			break;
		default:
			state_tap_sw = SW_UNPRESSED;
			break;
	}
	return IDLE;
}

void sm_effect(EventSwOutput input){
	switch (state_effect) {
		case EFF_BYP:
			if (input == TRIGGER_FIRST || input == TRIGGER_CONTINUOUS){
				state_effect = EFF_ON;

			}
			break;
		case EFF_ON:
			if (input == TRIGGER_FIRST || input == TRIGGER_CONTINUOUS){
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


