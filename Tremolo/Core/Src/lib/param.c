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
	struct Param param = {val, map_lin, val_max, map_min, map_max, invert};
	return param;
}


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
