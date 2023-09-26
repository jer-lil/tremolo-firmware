/*
 * param_map.c
 *
 *  Created on: Sep 24, 2023
 *      Author: jeremiah
 */

#include <math.h>

#include <lib/param.h>


// TODO make other map function types that can be passed in
struct Param param_init(
		uint32_t* val,
		ParamMap map_func,
		uint32_t val_min,
		uint32_t val_max,
		float map_min,
		float map_max,
		uint32_t invert)
{
	struct Param param = {val, map_func, val_max, map_min, map_max, invert};
	return param;
}


/*
 * Mapping functions specific to certain parameters
 */

float map_param_lin(struct Param* self)
{
	float ret = map_lin(
			*self->val,
			self->val_min,
			self->val_max,
			self->map_min,
			self->map_max);
	return ret;
}

float map_rate_log(struct Param* self)
{

	float ret = map_log(
			*self->val,
			0,
			self->val_max,
			self->map_min,
			self->map_max);
	return ret;
}

float map_rate_pseudo_log(struct Param* self)
{
	float val = (float)(*self->val);
	float val_max = (float)self->val_max;
	// Cap value at max value
	val = fminf(val, val_max);
	// TODO magic number
	float val_min = 0;
	float val_offset = 0.5;
	//float val_midpoint = (val_max + val_min) / 2;
	float val_midpoint = val_min + (val_offset * (val_max - val_min));
	float map_min = (float)self->map_min;
	float map_max = (float)self->map_max;

	// TODO magic number. How offset/kinked the map is
	float offset = 0.5;
	float map_midpoint = map_min + (offset *(map_max - map_min));
	if (val < val_midpoint)
	{
		val_min = val_midpoint;
		map_max = map_midpoint;
	}
	else
	{
		val_max = val_midpoint;
		map_min = map_midpoint;
	}
	float ret = map_lin(val, val_max, val_min, map_min, map_max);
	return ret;
}

/*
 * Generic mapping functions
 * TODO put these in their own file eventually
 */

/**
 * @brief Generic linear mapping function
 *
 * Maps val from range {val_min to val_max} onto {map_min to map_max}
 * To invert polarity, simply swap val_max and val_min.
 *
 * @param val
 * @param val_max
 * @param val_min
 * @param map_min
 * @param map_max
 * @param invert If invert = 1, inverts so that val_max maps to map_min
 * @return
 */
float map_lin(float val, float val_min, float val_max,
		float map_min, float map_max)
{
	// Percentage of val across val range
	float frac = (val - val_min) / (val_max - val_min);
	float ret = map_min + frac * (map_max - map_min);
	return ret;
}

float map_log(float val, float val_min, float val_max,
		float map_min, float map_max)
{
	// TODO define these as constants later
	float min_pow = log2f(map_min);
	float max_pow = log2f(map_max);
	float diff_pow = max_pow - min_pow;
	float frac = (val - val_min) / (val_max - val_min);
	float ret = map_min*powf(2, diff_pow*(1-frac));
	return ret;
}



























