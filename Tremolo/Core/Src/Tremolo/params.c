/*
 * params.c
 *
 *  Created on: Oct 13, 2023
 *      Author: jeremiah
 */

#include <Lib/param.h>
#include <Lib/wavetable_gen.h>
#include <Tremolo/params.h>
#include <Tremolo/tremolo.h>
#include <main.h>

/*
 * 	GET FUNCTIONS
 */

/**
 * @brief Returns 0-1 value for rate parameter
 *
 * Gets value from one of many sources depending on current state/mode
 * TODO Add sources other than ADC input
 * TODO change mapping to log / pseudo log
 *
 * @param adc_rate
 * @return
 */
float get_rate(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}

float get_volume(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}

float get_depth(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}

float get_offset(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}

float get_phase_knob(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}

float get_sense(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}

float get_env(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}

float get_exp(uint16_t adc_val)
{
	float map_val = map_lin(adc_val, 0, ADC_RESOLUTION, 0, 1);
	return map_val;
}


struct subdiv get_subdiv()
{
	struct subdiv subdiv;
	uint32_t ph_right = HAL_GPIO_ReadPin(pDIN_HARM_MODE_1_GPIO_Port,
			  pDIN_HARM_MODE_1_Pin);
	uint32_t ph_left = HAL_GPIO_ReadPin(pDIN_HARM_MODE_2_GPIO_Port,
			  pDIN_HARM_MODE_2_Pin);
	if (!ph_left)
	{
		subdiv.num = 1;
		subdiv.denom = QUARTER;
	}
	else if (!ph_right)
	{
		subdiv.num = 1;
		subdiv.denom = EIGHTH;
	}
	else
	{
		subdiv.num = 1;
		subdiv.denom = TRIPLET;
	}
	return subdiv;
}

/**
 * @brief Returns phase state based on toggle switch input
 *
 * Not currently planning to use but keeping for future use.
 *
 * @return
 */
PhaseToggle get_phase_toggle(){
	  uint32_t ph_right = HAL_GPIO_ReadPin(pDIN_HARM_MODE_1_GPIO_Port,
			  pDIN_HARM_MODE_1_Pin);
	  uint32_t ph_left = HAL_GPIO_ReadPin(pDIN_HARM_MODE_2_GPIO_Port,
			  pDIN_HARM_MODE_2_Pin);
	  if (!ph_left){ return PAN; }
	  else if (!ph_right){ return HARM; }
	  else { return STD; }
}

float* get_phases(float phase, float phases[4])
{
	phases[0] = 0;
	phases[1] = phase;
	phases[2] = fmin(0.5, phase);
	phases[3] = fmax(0, phase-0.5);
	return phases;
}


Shape get_shape()
{
	uint32_t right = HAL_GPIO_ReadPin(pDIN_PAN_MODE_1_GPIO_Port,
			pDIN_PAN_MODE_1_Pin);
	uint32_t left = HAL_GPIO_ReadPin(pDIN_PAN_MODE_2_GPIO_Port,
			pDIN_PAN_MODE_2_Pin);
	if (!left) { return SINE; }
	else if (!right) { return SQUR; }
	else { return TRI; }
}




/*
 * 	SET FUNCTIONS
 */

/**
 * @brief Sets timer registers based on rate & subdivision inputs
 *
 * Prescaler and ARR get set 4x for the sake of code portability.
 * Could be made more efficient with some clever macros.
 *
 * @param rate Base quarter note LFO rate
 * @param subdiv Note subdivision, e.g. quarter, eighth, triplet
 */
void set_rate(float rate, struct subdiv subdiv){
	// Set prescaler based on subdiv
	uint32_t prsclr = PWM_TIM_PRSCLR_BASE * QUARTER * subdiv.num / subdiv.denom;
	__HAL_TIM_SET_PRESCALER(&HTIM_WVFM_A_LO, prsclr);
	__HAL_TIM_SET_PRESCALER(&HTIM_WVFM_A_HI, prsclr);
	__HAL_TIM_SET_PRESCALER(&HTIM_WVFM_B_LO, prsclr);
	__HAL_TIM_SET_PRESCALER(&HTIM_WVFM_B_HI, prsclr);
	// Set ARR based on rate
	__HAL_TIM_SET_AUTORELOAD(&HTIM_WVFM_A_LO, (uint32_t)rate);
	__HAL_TIM_SET_AUTORELOAD(&HTIM_WVFM_A_HI, (uint32_t)rate);
	__HAL_TIM_SET_AUTORELOAD(&HTIM_WVFM_B_LO, (uint32_t)rate);
	__HAL_TIM_SET_AUTORELOAD(&HTIM_WVFM_B_HI, (uint32_t)rate);
}

void set_volume(float vol){
	__HAL_TIM_SET_COMPARE(&HTIM_VOL_A, TIM_CH_VOL_A, (uint16_t)vol);
	__HAL_TIM_SET_COMPARE(&HTIM_VOL_B, TIM_CH_VOL_B, (uint16_t)vol);
}

/**
 * @brief Generates wavetables based on specified parameters
 *
 * @param tables
 * @param table_qty
 * @param shape
 * @param depth
 * @param offset
 * @param phase
 */
void update_lfo_waveform(uint16_t* tables[], uint16_t table_qty, Shape shape, float depth, float offset,
		float phase)
{
	// Derive 4x phase offsets from single phase input
	// TODO move inside wavetable_gen function
	float phases[] = {0};
	get_phases(phase, phases);

	for (int i=0; i<table_qty; i++){
		wavetable_gen(shape, depth,offset, phases[i],
				tables[i], WAVETABLE_WIDTH, WAVETABLE_DEPTH);
	}
	return;
}
