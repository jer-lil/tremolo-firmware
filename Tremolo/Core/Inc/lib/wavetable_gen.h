/*
 * wavetable_gen.h
 *
 *  Created on: Sep 20, 2023
 *      Author: jeremiah
 */

#ifndef INC_LIB_WAVETABLE_GEN_H_
#define INC_LIB_WAVETABLE_GEN_H_

#include <stdint.h>


/**
 * @brief Each value corresponds to an LFO wave shape
 *
 */
typedef enum {
	ERR = 0,
	TRI = 1, /**< TRI */
	SINE = 2,/**< SINE */
	SQUR = 3,/**< SQUR */
} Shape;

void wavetable_gen(
	Shape,
	float,
	float,
	float,
	uint16_t*,
	uint16_t,
	uint16_t);

void wavetable_gen_tri(
	Shape,
	float,
	float,
	float,
	uint16_t*,
	uint16_t,
	uint16_t);

void wavetable_gen_sine(
	Shape,
	float,
	float,
	float,
	uint16_t*,
	uint16_t,
	uint16_t);

void wavetable_gen_square(
	Shape,
	float,
	float,
	float,
	uint16_t*,
	uint16_t,
	uint16_t);

float rate_map(uint16_t, uint16_t);

#endif /* INC_LIB_WAVETABLE_GEN_H_ */
