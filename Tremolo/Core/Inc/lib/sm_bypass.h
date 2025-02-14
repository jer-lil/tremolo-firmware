/*
 * sm_bypass.h
 *
 *  Created on: Mar 28, 2023
 *      Author: jeremiah
 */

#ifndef INC_LIB_SM_BYPASS_H_
#define INC_LIB_SM_BYPASS_H_

#include "Lib/led.h"

typedef enum {
	SW_UNPRESSED,
	SW_PRESSED,
	SW_HELD,
	SW_WAIT_RELEASE,
	SW_DEBOUNCE_PRESS,
	SW_DEBOUNCE_RELEASE,
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
	TOGGLE,
	IDLE,
} EventEffect;

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


EventEffect sm_byp_sw(EventSw);
void sm_effect(EventEffect);
void sm_relay_mute(StateEffect, LED*);

#endif /* INC_LIB_SM_BYPASS_H_ */
