/*
 * sm_bypass.h
 *
 *  Created on: Mar 28, 2023
 *      Author: jeremiah
 */

#ifndef INC_LIB_SM_BYPASS_H_
#define INC_LIB_SM_BYPASS_H_

typedef enum {
	STATE_IDLE,
	STATE_WAIT_RELEASE,
	STATE_DEBOUNCE_PRESS,
	STATE_DEBOUNCE_RELEASE,
} StateBypassSw;

typedef enum {
	EVENT_PRESSED,
	EVENT_RELEASED,
} EventBypassSw;

typedef enum {
	STATE_EFFECT,
	STATE_BYPASS,
} StateEffect;

typedef enum {
	EVENT_TOGGLE,
} EventEffect;

typedef enum {
	STATE_EFFECT_UNMUTE,
	STATE_EFFECT_MUTE,
	STATE_BYPASS_UNMUTE,
	STATE_BYPASS_MUTE,
} StateRelayMute;

typedef enum {
	EVENT_EFFECT,
	EVENT_BYPASS
} EventRelayMute;


void sm_bypass_sw(StateBypassSw*, EventBypassSw, StateEffect*);
void sm_effect(StateEffect*, EventEffect);
void sm_relay_mute(StateRelayMute*, EventRelayMute);


#endif /* INC_LIB_SM_BYPASS_H_ */
