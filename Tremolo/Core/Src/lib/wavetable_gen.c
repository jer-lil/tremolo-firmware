/**
 * wavetable_gen.c
 * @file wavetable_gen.c
 * @brief Functions to generate wavetables
 *
 *  Created on: Sep 20, 2023
 *      Author: jeremiah
 */


#include <lib/wavetable_gen.h>

/**
 * @brief Generates a single wavetable
 * It stores the table in the memory location specified
 *
 * TODO should depth/offset be referenced to 1023 or to table_depth?
 *
 * @param depth 0-1 How deep (close to 0) the table values go. 0 = Flat.
 * @param offset 0-1 Offsets the center of the wave from left (0) to right (1)
 * @param phase 0-1
 * @param table Pointer to wavetable array location
 * @param table_length The number of entries in the table == Array size
 * @param table_depth The max value of each table entry. This should
 * 			be equivalent to the max PWM
 *
 *	Examples:
 *
 *	/\ -- shape=TRI, depth=1, offset=0.5, phase=0
 *	|\ -- shape=TRI, depth=1, offset=0, phase=0
 *	\/ -- shape=TRI, depth=1, offset=0.5, phase=0.5
 *
 */
void wavetable_gen(
	Shape shape,
	float depth,
	float offset,
	float phase,
	uint16_t* table,
	uint16_t table_width,
	uint16_t table_depth)
{
	// TODO bounds checking
	// TODO any pre-calcs to do for all 3 shapes?
	switch (shape)
	{
		case TRI:
			wavetable_gen_tri(shape, depth, offset, phase,
					table, table_width, table_depth);
			break;
		case SINE:
			wavetable_gen_sine(shape, depth, offset, phase,
					table, table_width, table_depth);
			break;
		case SQUR:
			wavetable_gen_square(shape, depth, offset, phase,
					table, table_width, table_depth);
			break;
		default:
			// TODO error
			break;
	}
}

void wavetable_gen_tri(
	Shape shape,
	float depth,
	float offset,
	float phase,
	uint16_t* table,
	uint16_t table_width,
	uint16_t table_depth)
{
	// Starting index in the table; based on phase
	uint16_t start_index = (uint16_t)(table_width-1)*phase;
	// Midpoint_rel is where the triangle "peak" would be if phase=0
	uint16_t midpoint_rel = (uint16_t)(table_width-1)*offset;
	// midpoint_abs accounts for phase offset
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// Index is the current table index
	uint16_t index = start_index;
	// Val is the current table value; it starts at the lowest value
	float val = (float)table_depth - ((float)table_depth*depth);
	// The amount by which to increase/decrease val between entries
	float step_up;
	float step_down;

	table[index] = (uint16_t)val;
	index = (index + 1) % (table_width);

	// Rising slope of triangle. Skip if offset is all the way left.
	if (midpoint_rel > 0)
	{
		step_up = (float)depth*table_depth / (float)midpoint_rel;
		while (index != midpoint_abs)
		{
			val = val+step_up;
			table[index] = (uint16_t)val;
			index = (index + 1) % (table_width);
		}
	}
	// Falling slope of triangle. Skip if offset is all the way left.

	step_down = (float)depth*table_depth / (float)(table_width-midpoint_rel);
	val = table_depth;
	while (index!=start_index)
	{
		val = val-step_down;
		table[index] = (uint16_t)val;
		index = (index + 1) % (table_width);
	}

	return;
}

void wavetable_gen_sine(
	Shape shape,
	float depth,
	float offset,
	float phase,
	uint16_t* table,
	uint16_t table_width,
	uint16_t table_depth)
{
	return;
}

void wavetable_gen_square(
	Shape shape,
	float depth,
	float offset,
	float phase,
	uint16_t* table,
	uint16_t table_width,
	uint16_t table_depth)
{
	// Starting index in the table; based on phase
	uint16_t start_index = (uint16_t)(table_width-1)*phase;
	// Midpoint_rel is where the triangle "peak" would be if phase=0
	uint16_t midpoint_rel = (uint16_t)(table_width-1)*offset;
	// midpoint_abs accounts for phase offset
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// Index is the current table index
	uint16_t index = start_index;
	// Val is the current table value; it starts at the lowest value
	uint16_t low_val = table_depth - ((float)table_depth*depth);
	uint16_t high_val = table_depth;

	table[index] = low_val;
	index = (index + 1) % (table_width);

	// Low portion of square wave
	if (midpoint_rel > 0)
	{
		while (index != midpoint_abs)
		{
			table[index] = low_val;
			index = (index + 1) % (table_width);
		}
	}
	// High portion of square wave
	while (index!=start_index)
	{
		table[index] = high_val;
		index = (index + 1) % (table_width);
	}
	return;
}


/**
 * @brief maps raw rate input to subdivided rate in Hz
 *
 * @param rate_base 0-1023 raw rate input
 * @param integer divisor of rate input, e.g. quarter note = 4
 */
float rate_map(uint16_t rate_base, uint16_t subdiv){
	float rate_hz = (float)rate_base;
	// do some mapping
	return rate_hz;
}






