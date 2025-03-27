/*
 * param_map.h
 *
 *  Created on: Sep 24, 2023
 *      Author: jeremiah
 */

#ifndef INC_LIB_PARAM_H_
#define INC_LIB_PARAM_H_

#include <stdint.h>

// Forward declaration for typedef below
struct Param;

typedef float (*ParamMap)(struct Param* param);

struct Param{
	uint32_t* val;
	ParamMap map_func;
	uint32_t val_min;
	uint32_t val_max;
	float map_min;
	float map_max;
	uint32_t invert;
};

struct Param param_init(
		uint32_t*,
		ParamMap,
		uint32_t,
		uint32_t,
		float,
		float,
		uint32_t);


// Mapping functions for specific parameters
float map_param_lin(struct Param*);
float map_rate_log(struct Param*);
float map_rate_pseudo_log(struct Param*);


// Generic mapping functions
// TODO move these to separate file
float map_lin(float, float, float, float, float);
float map_log(float, float, float, float, float);



#endif /* INC_LIB_PARAM_H_ */
