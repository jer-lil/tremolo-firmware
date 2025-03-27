/*
 * sm_led.c
 *
 *  Created on: May 28, 2023
 *      Author: jeremiah
 *
 *  Defines functions to update the Color and OnState of an LED object.
 *
 *  TODO implement colors other than RGB
 *  -> May want to come up with a smarter way of setting GPIO pins.
 *  -> Maybe want another struct and/or an array of GPIO ports/pins?
 *
 *  TODO possibly implement dimmer functionality, would be pretty complex.
 */

#include <Lib/led.h>

LED defaultLED()
{
	LED led = {GREEN, OFF};
	return led;
}

/**
 * @brief Initializes LED object with GPIO pins/ports
 *
 * This feels a little hacky. Maybe I just wish I was using C++ though.
 *
 * @param led LED object to initialize
 * @param color Color to initialize LED to
 * @param on_state Initialize LED to either on or off
 * @param PortRed
 * @param PinRed
 * @param PortGreen
 * @param PinGreen
 * @param PortBlue
 * @param PinBlue
 */
void initLED(LED* led, LEDColor color, LEDOnState on_state,
		GPIO_TypeDef* PortRed, uint16_t PinRed,
		GPIO_TypeDef* PortGreen, uint16_t PinGreen,
		GPIO_TypeDef* PortBlue, uint16_t PinBlue)
{
	led->PortRed = PortRed;
	led->PinRed = PinRed;
	led->PortGreen = PortGreen;
	led->PinGreen = PinGreen;
	led->PortBlue = PortBlue;
	led->PinBlue = PinBlue;

	led->Color = color;
	led->OnState = on_state;

	set_LED_color(led, GREEN);
	set_LED_state(led, OFF);
}

/**
 * @brief Updates the color of the LED
 *
 * Only changes the GPIO state if the LED is already on.
 *
 * @param led
 * @param color
 */
void set_LED_color(LED* led, LEDColor color){
	led->Color = color;
	// Only update GPIO pins if LED is already on
	if (led->OnState == ON){
		set_LED_state(led, ON);
	}
}

/**
 * @brief Updates LED on-state and sets GPIO pins to match
 *
 * @param led LED object to set state of
 * @param on_state ON or OFF
 */
void set_LED_state(LED* led, LEDOnState on_state){
	led->OnState = on_state;
	if (on_state == ON){
		switch (led->Color) {
			case RED:
				HAL_GPIO_WritePin(led->PortRed, led->PinRed, LED_PIN_SET);
				HAL_GPIO_WritePin(led->PortGreen, led->PinGreen, LED_PIN_RESET);
				HAL_GPIO_WritePin(led->PortBlue, led->PinBlue, LED_PIN_RESET);
				break;
			case GREEN:
				HAL_GPIO_WritePin(led->PortRed, led->PinRed, LED_PIN_RESET);
				HAL_GPIO_WritePin(led->PortGreen, led->PinGreen, LED_PIN_SET);
				HAL_GPIO_WritePin(led->PortBlue, led->PinBlue, LED_PIN_RESET);
				break;
			case BLUE:
				HAL_GPIO_WritePin(led->PortRed, led->PinRed, LED_PIN_RESET);
				HAL_GPIO_WritePin(led->PortGreen, led->PinGreen, LED_PIN_RESET);
				HAL_GPIO_WritePin(led->PortBlue, led->PinBlue, LED_PIN_SET);
				break;
			default:
				break;
		}
	}
	else {
		HAL_GPIO_WritePin(led->PortRed, led->PinRed, LED_PIN_RESET);
		HAL_GPIO_WritePin(led->PortGreen, led->PinGreen, LED_PIN_RESET);
		HAL_GPIO_WritePin(led->PortBlue, led->PinBlue, LED_PIN_RESET);
	}
}

/* Toggles LED if it's been longer than timout_ms since last toggle
 * Uses RGB LED struct defined in led.h
 */
void led_toggle_tick_rgb(uint32_t timeout_ms, LED* led, LEDColor color)
{
	static uint32_t last_toggle_ms = 0;
	uint32_t tick = HAL_GetTick();

	if (tick - last_toggle_ms >= timeout_ms)
	{
		last_toggle_ms = tick;
		set_LED_state(led, ~led->OnState);
	}
	return;
}
/* Toggles LED if it's been longer than timout_ms since last toggle
 * Doesn't use LED struct defined in led.h
 */
void led_toggle_tick_single(uint32_t timeout_ms, GPIO_TypeDef* LED_Port,
		uint16_t LED_Pin)
{
	static uint32_t last_toggle_ms = 0;
	uint32_t tick = HAL_GetTick();

	if (tick - last_toggle_ms >= timeout_ms)
	{
		last_toggle_ms = tick;
		HAL_GPIO_TogglePin(LED_Port, LED_Pin);
	}
	return;
}


