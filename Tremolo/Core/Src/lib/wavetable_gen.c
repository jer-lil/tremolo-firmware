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

	// TODO WIP moving some variable calcs to top level
	// Starting index in the table; based on phase
	uint16_t start_index = (uint16_t)(table_width-1)*phase;
	// Midpoint_rel is where the triangle "peak" would be if phase=0
	uint16_t midpoint_rel = (uint16_t)(table_width-1)*offset;
	// Max value - min value; the actual depth of the table
	float ampl = (float)depth*table_depth;





	switch (shape)
	{
		case TRI:
			wavetable_gen_tri(start_index, midpoint_rel, ampl,
					table, table_width, table_depth);
			break;
		case SINE:
			wavetable_gen_sine(start_index, midpoint_rel, ampl,
					table, table_width, table_depth);
			break;
		case SQUR:
			wavetable_gen_square(start_index, midpoint_rel, ampl,
					table, table_width, table_depth);
			break;
		default:
			// TODO error
			break;
	}



}

void wavetable_gen_tri(
	uint16_t start_index,
	uint16_t midpoint_rel,
	float ampl,
	uint16_t* table,
	uint16_t table_width,
	uint16_t table_depth)
{
	// midpoint_abs accounts for phase offset
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// Index is the current table index
	uint16_t index = start_index;
	// Val is the current table value; it starts at the lowest value
	float val = (float)table_depth - ampl;
	// The amount by which to increase/decrease val between entries
	float step_up;
	float step_down;

	table[index] = (uint16_t)val;
	index = (index + 1) % (table_width);

	// Rising slope of triangle. Skip if offset is all the way left.
	if (midpoint_rel > 0)
	{
		step_up = ampl / (float)midpoint_rel;
		while (index != midpoint_abs)
		{
			val = val+step_up;
			table[index] = (uint16_t)val;
			index = (index + 1) % (table_width);
		}
	}
	// Falling slope of triangle. Skip if offset is all the way left.

	step_down = ampl / (float)(table_width-midpoint_rel);
	val = table_depth;
	while (index!=start_index)
	{
		val = val-step_down;
		table[index] = (uint16_t)val;
		index = (index + 1) % (table_width);
	}

	return;
}

// TODO make this more efficient
// TODO phase does not work
void wavetable_gen_sine(
		uint16_t start_index,
		uint16_t midpoint_rel,
		float ampl,
		uint16_t* table,
		uint16_t table_width,
		uint16_t table_depth)
{
	// midpoint_abs accounts for phase offset
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// Index is the current table index
	uint16_t index = start_index;
	// Val is the current table value; it starts at the max val for cosine
	float val = (float)table_depth;
	float ampl_div = ampl / 2;
	uint16_t offset = (uint16_t)(table_depth - ampl_div);
	// Effective period of the sine wave in counts
	float period;

	table[index] = (uint16_t)val;
	index = (index + 1) % (table_width);

	// Rising slope of sine. Skip if offset is all the way left.
	if (midpoint_rel > 0)
	{
		period = 2 * midpoint_rel;
		while (index != midpoint_abs)
		{

			table[index] =  (uint16_t)(offset + ampl_div *
					cosf(2 * PI * ((float)index / (float)period)));
			index = (index + 1) % (table_width);
		}
	}
	// Second half of (co)sine
	period = 2 * (table_width - midpoint_rel);
	while (index!=start_index)
	{
		table[index] =  (uint16_t)(offset + ampl_div *
				cosf(PI - (2 * PI * (((float)index - midpoint_rel) / (float)period))));
		index = (index + 1) % (table_width);
	}

	return;
}

void wavetable_gen_square(
		uint16_t start_index,
		uint16_t midpoint_rel,
		float ampl,
		uint16_t* table,
		uint16_t table_width,
		uint16_t table_depth)
{
	// midpoint_abs accounts for phase offset
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// Index is the current table index
	uint16_t index = start_index;
	// Val is the current table value; it starts at the lowest value
	uint16_t low_val = table_depth - ampl;
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






