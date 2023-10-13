/*
 * params.h
 *
 *  Created on: Oct 13, 2023
 *      Author: jeremiah
 */

#ifndef INC_TREMOLO_PARAMS_H_
#define INC_TREMOLO_PARAMS_H_

// Base prescaler value with quarter note subdiv
#define PWM_TIM_PRSCLR_BASE 24
// Used for prescaler normalization to quarter notes
#define QUARTER 4
#define TRIPLET 6
#define EIGHTH 8

// TODO define these programmatically based on min/max LFO rates
#define RATE_ARR_MIN 256
#define RATE_ARR_MAX 1024

#define VOL_MAP_MAX 1023
#define VOL_MAP_MIN 100

// 0-2.5V (about the max I've seen the circuit output)
#define ENV_MIN 0
#define ENV_MAX 775

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



// PARAM SETTERS
void set_rate(float, struct subdiv);
void set_volume(float);

void update_lfo_waveform(uint16_t*[], uint16_t, Shape, float, float, float);

#endif /* INC_TREMOLO_PARAMS_H_ */
