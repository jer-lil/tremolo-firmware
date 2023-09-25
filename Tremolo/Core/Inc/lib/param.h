/*
 * param_map.h
 *
 *  Created on: Sep 24, 2023
 *      Author: jeremiah
 */

#ifndef INC_LIB_PARAM_H_
#define INC_LIB_PARAM_H_

#include <stdint.h>


struct Param{
	uint32_t* val;
	float (*map_val)(struct Param* self);
	uint32_t val_max;
	float map_min;
	float map_max;
	uint32_t invert;
};

struct Param param_init(uint32_t*, float, float, uint32_t, uint32_t);
float map_lin(struct Param*);


#endif /* INC_LIB_PARAM_H_ */
