/*
 * debug.h
 *
 *  Created on: Oct 13, 2023
 *      Author: jeremiah
 */

#ifndef INC_TREMOLO_DEBUG_H_
#define INC_TREMOLO_DEBUG_H_


// UART Debugging Functions
void transmit_wavetables(UART_HandleTypeDef*, uint16_t*[] ,uint16_t, uint16_t);
void transmit_wavetable(UART_HandleTypeDef*, uint16_t[], uint16_t, uint8_t);


#endif /* INC_TREMOLO_DEBUG_H_ */
