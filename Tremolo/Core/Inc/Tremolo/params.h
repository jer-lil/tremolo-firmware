/*
 * params.h
 *
 *  Created on: Feb 11, 2025
 *      Author: jeremiah
 */

#ifndef INC_TREMOLO_PARAMS_H_
#define INC_TREMOLO_PARAMS_H_

/**
 * @brief Enum for discrete phase states, for toggle switch control
 *
 * Not currently planning to use this, but keeping for future use.
 *
 */
typedef enum {
	STD, /**< STATE_STD */
	HARM,/**< STATE_HARM */
	PAN, /**< STATE_PAN */
} PhaseToggle;

struct params {
	struct Param rate;
	struct Param depth;
	struct Param offset;
	struct Param phase;
	struct Param vol;
} ;

struct subdiv {
	uint32_t num;
	uint32_t denom;
};



// PARAM GETTERS
float get_rate(uint16_t);
float get_depth(uint16_t);
float get_offset(uint16_t);
float get_phase_knob(uint16_t);
PhaseToggle get_phase_toggle();
struct subdiv get_subdiv();
float get_volume(uint16_t);
Shape get_shape();
float get_sense(uint16_t);
float get_env(uint16_t);
float get_exp(uint16_t);
float get_phase(float, uint16_t);



// PARAM SETTERS
void set_rate(float, struct subdiv);
void set_volume(float);

void update_lfo_waveform(uint16_t*[], uint16_t, Shape, float, float, float);

#endif /* INC_TREMOLO_PARAMS_H_ */
