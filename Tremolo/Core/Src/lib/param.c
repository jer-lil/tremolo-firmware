/*
 * param_map.c
 *
 *  Created on: Sep 24, 2023
 *      Author: jeremiah
 */

#include <lib/param.h>

// TODO make other map function types that can be passed in
struct Param param_init(uint32_t* val, float map_min, float map_max,
		uint32_t val_max, uint32_t invert)
{
	struct Param param = {val, map_rate_pseudo_log, val_max, map_min, map_max, invert};
	return param;
}

// TODO rename, make this generic and be able to call other mapping functions
float map_lin(struct Param* self)
{
	float val = (float)(*self->val);
	float val_max = (float)self->val_max;
	float map_min = (float)self->map_min;
	float map_max = (float)self->map_max;
	uint32_t invert = self->invert;
	float frac;
	// Cap value at max value
	if (val > val_max)
	{
		val = val_max;
	}
	// Flip polarity if needed
	// TODO this can probably be done in a smarter way
	if (invert)
	{
		frac = 1 - (val / val_max);
	}
	else
	{
		frac = (val / val_max);
	}
	// Map {0 to val_max} onto {map_min to map_max}
	return map_min + (frac * (map_max - map_min));
}


float map_rate_pseudo_log(struct Param* self)
{
	float val = (float)(*self->val);
	float val_max = (float)self->val_max;
	float map_min = (float)self->map_min;
	float map_max = (float)self->map_max;
	uint32_t invert = self->invert;
	float frac;
	// Cap value at max value
	if (val > val_max)
	{
		val = val_max;
	}
	// Flip polarity if needed
	// TODO this can probably be done in a smarter way
	if (invert)
	{
		frac = 1 - (val / val_max);
	}
	else
	{
		frac = (val / val_max);
	}

	// Everything above here is the same as map_lin ^^
	// TODO magic number, pass this in as variable
	float midpoint_offset = 0.2;
	float midpoint_val = map_min + (midpoint_offset *(map_max - map_min));
	float midpoint_index = 0.2;
	float result;

	if (frac <= midpoint_index)
	{
		result = map_min + ((frac/midpoint_offset) * (midpoint_val - map_min));
	}
	else
	{
		result = midpoint_val + ((frac/midpoint_offset) * (map_max - midpoint_val));
	}
	return result;
}

/*
float map_rate_log(struct Param* self)
{

}
*/

