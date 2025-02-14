/*
 * sm_bypass.h
 *
 *  Created on: Mar 28, 2023
 *      Author: jeremiah
 */

#ifndef INC_LIB_SM_BYPASS_H_
#define INC_LIB_SM_BYPASS_H_

#include "Lib/led.h"
#include <stdbool.h>

typedef enum {
	SW_UNPRESSED,
	SW_UNPRESSED_HELD,
	SW_DEBOUNCE_PRESS,
	SW_PRESSED,
	SW_DEBOUNCE_RELEASE,
	SW_PRESSED_HELD,
} StateBypSw;

typedef enum {
	SW_IDLE,
	SW_NEW_PRESS,
	SW_NEW_RELEASE,
} EventSw;

typedef enum {
	EFF_ON,
	EFF_BYP,
} StateEffect;

typedef enum {
	TRIGGER_FIRST,
	TRIGGER_CONTINUOUS,
	IDLE,
} EventSwOutput;

typedef enum {
	EFF_ON_UNMUTE,
	EFF_ON_MUTE,
	EFF_BYP_UNMUTE,
	EFF_BYP_MUTE,
} StateRelayMute;

typedef enum {
	EFF_ENABLE,
	EFF_DISABLE,
} EventRelayMute;

// State machine functions
EventSwOutput sm_byp_sw(EventSw);
EventSwOutput sm_tap_sw(EventSw, uint32_t);
void sm_effect(EventSwOutput);
void sm_relay_mute(StateEffect, LED*);

// Helper functions
void add_first_tap(uint32_t);
void add_new_tap(uint32_t, bool);


#endif /* INC_LIB_SM_BYPASS_H_ */
