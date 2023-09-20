/*
 * sm_led.h
 *
 *  Created on: May 28, 2023
 *      Author: jeremiah
 */

#ifndef INC_LIB_LED_H_
#define INC_LIB_LED_H_

#include "gpio.h"

// LEDs are default low by default. Main Program can overwrite if needed.
#ifndef LED_PIN_SET
#define LED_PIN_SET GPIO_PIN_RESET
#define LED_PIN_RESET GPIO_PIN_SET
#endif

// Enum definitions for LED object
typedef enum {
	RED,
	GREEN,
	BLUE,
} LEDColor;

typedef enum {
	OFF = 0,
	ON = 1,
} LEDOnState;

// LED Object
typedef struct {
	// State variables
	LEDColor Color;
	LEDOnState OnState;
	// GPIO port/pin defs
	GPIO_TypeDef* PortRed;
	uint16_t PinRed;
	GPIO_TypeDef* PortGreen;
	uint16_t PinGreen;
	GPIO_TypeDef* PortBlue;
	uint16_t PinBlue;
} LED;

LED defaultLED();
void initLED(LED*, LEDColor, LEDOnState, GPIO_TypeDef*, uint16_t,
		GPIO_TypeDef*, uint16_t, GPIO_TypeDef* , uint16_t);
void set_LED_color(LED*, LEDColor);
void set_LED_state(LED*, LEDOnState);
void led_toggle_tick_rgb(uint32_t, LED*, LEDColor);
void led_toggle_tick_single(uint32_t, GPIO_TypeDef*, uint16_t);

#endif /* INC_LIB_LED_H_ */
