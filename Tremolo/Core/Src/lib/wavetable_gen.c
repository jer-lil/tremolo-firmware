/**
 * wavetable_gen.c
 * @file wavetable_gen.c
 * @brief Functions to generate wavetables
 *
 *  Created on: Sep 20, 2023
 *      Author: jeremiah
 */


#include <Lib/wavetable_gen.h>

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

	switch (shape)
	{
		case TRI:
			wavetable_gen_tri(start_index, midpoint_rel, depth,
					table, table_width, table_depth);
			break;
		case SINE:
			wavetable_gen_sine(start_index, midpoint_rel, depth,
					table, table_width, table_depth);
			break;
		case SQUR:
			wavetable_gen_square(start_index, midpoint_rel, depth,
					table, table_width, table_depth);
			break;
		default:
			Error_Handler();
			break;
	}
}

void wavetable_gen_tri(
	uint16_t start_index,
	uint16_t midpoint_rel,
	float depth,
	uint16_t* table,
	uint16_t table_width,
	uint16_t table_depth)
{
	// midpoint_abs accounts for phase offset
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// Index is the current table index
	uint16_t index = start_index;
	// Max value - min value; the actual depth of the table
	float ampl = (float)depth*table_depth;
	// Val is the current table value; it starts at the lowest value
	float val = (float)table_depth;
	// The amount by which to increase/decrease val between entries
	float step_size;

	table[index] = (uint16_t)val;
	index = (index + 1) % (table_width);

	// Falling slope of triangle. Skip if offset is all the way left.
	if (midpoint_rel > 0)
	{
		step_size = ampl / (float)midpoint_rel;
		while (index != midpoint_abs)
		{
			val = val-step_size;
			table[index] = (uint16_t)val;
			index = (index + 1) % (table_width);
		}
	}
	// Rising slope of triangle. Skip if offset is all the way right.
	if (midpoint_rel < table_width)
	{
		step_size = ampl / (float)(table_width-midpoint_rel);
		val = (float)table_depth - ampl;
		while (index!=start_index)
		{
			val = val+step_size;
			table[index] = (uint16_t)val;
			index = (index + 1) % (table_width);
		}
	}


	return;
}

void wavetable_gen_sine(
		uint16_t start_index,
		uint16_t midpoint_rel,
		float depth,
		uint16_t* table,
		uint16_t table_width,
		uint16_t table_depth)
{
	// midpoint_abs accounts for phase offset
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// dest_index is the current index in the destination table
	uint16_t dest_index = start_index;
	// current index in the source table (which is not the same size as the destination table)
	float source_index = 0;
	// amount to step through source table for each increment in dest table
	// float because this will usually be a non-integer, rounded to the nearest int
	float step_size = 1;
	float min_val = (float)table_depth * (1 - depth);
	// Manually grab first table value, to make the if/while logic work
	float src_val_raw = sineLookupTable[(uint16_t)source_index];
	table[dest_index] = (uint16_t)(min_val + (depth * src_val_raw));

	// if statements protect from dividing by 0
	if (midpoint_rel > 0) {
		// first half of table; skip if midpoint all the way to the left
		step_size = 0.5 * (((float)sineTableSize) / (float)midpoint_rel);
		while (dest_index != midpoint_abs) {
			// Increment source/destination indices
			source_index += step_size;
			dest_index = (dest_index + 1) % table_width;
			// Populate wavetable with next val from lookup, scaled for depth
			table[dest_index] = (uint16_t)(min_val + \
								(depth * sineLookupTable[(uint16_t)source_index]));

		}
	}
	if (midpoint_rel < table_width) {
		// second half of table; skip if midpoint all the way to the right
		step_size = 0.5 * ((float)sineTableSize) / (float)(table_width - midpoint_rel);
		source_index = ((float)sineTableSize / 2) - 1;
		do {
			source_index += step_size;
			dest_index = (dest_index + 1) % table_width;
			table[dest_index] = (uint16_t)(min_val + \
					(depth * sineLookupTable[(uint16_t)source_index]));

		} while (dest_index != start_index);
	}
}

void wavetable_gen_square(
		uint16_t start_index,
		uint16_t midpoint_rel,
		float depth,
		uint16_t* table,
		uint16_t table_width,
		uint16_t table_depth)
{
	// midpoint_abs accounts for phase offset
	start_index = (start_index + (uint16_t)(3*table_width/4)) % table_width;
	uint16_t midpoint_abs = (midpoint_rel + start_index) % (table_width);
	// Index is the current table index
	uint16_t index = start_index;
	// Max value - min value; the actual depth of the table
	uint16_t ampl = (uint16_t)(depth*table_depth);
	// Val is the current table value; it starts at the lowest value
	uint16_t low_val = table_depth - ampl;
	uint16_t high_val = table_depth;

	table[index] = high_val;
	index = (index + 1) % (table_width);


	// High portion of square wave
	if (midpoint_rel > 0)
	{
		while (index != midpoint_abs)
		{
			table[index] = high_val;
			index = (index + 1) % (table_width);
		}
	}
	// Low portion of square wave
	while (index!=start_index)
	{
		table[index] = low_val;
		index = (index + 1) % (table_width);
	}
	return;
}






