/** ***************************************************************************
 * @file
 * @brief provides functions to calculate different values
 *
 * Different functions for calculating RMS values, moving averages and angle.
 * Look up tables used for estimating the current and distance of a cable.
 * Moving averages are used to stabilize and de-noise the signal.
 *
 * ----------------------------------------------------------------------------
 * @author Linus Leuch, leuchlin@students.zhaw.ch,
 * @n Simon Meli, melisim1@students.zhaw.ch
 * @date 22.12.2022
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "stm32f4xx_hal.h"
#include "arm_cfft_init_f32.h"
#include "arm_math.h"

#include "display.h"
#include "calculation.h"
#include "measuring.h"
#include "main.h"
#include "render.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Variables
 *****************************************************************************/
uint32_t fft_positive_out[DOPP_ADC_SAMPLES / 2]; ///< samples for channel 1 (PAD 1)
uint32_t fft_negative_out[DOPP_ADC_SAMPLES / 2]; ///< samples for channel 2 (PAD 2)
uint32_t raw_PC1_data[DOPP_ADC_SAMPLES/2];			  ///< samples for channel 3 (PAD 3)
uint32_t raw_PC3_data[DOPP_ADC_SAMPLES/2];			  ///< samples for channel 4 (PAD 4)

uint8_t batt_lvl; ///< battery voltage level

arm_cfft_instance_f32 cfft_instance;

/******************************************************************************
 * Functions
 *****************************************************************************/

/** ***************************************************************************
 * @brief separates DMA samples into five Arrays for each corresponding channel
 * @param array of samples
 * @note only works for 5 channel array
 *****************************************************************************/
void CALC_separate_data(uint32_t samples[])
{
	for (int i = 0; i < ADC_NUMS; i++)
	{
		fft_positive_out[i] = samples[i * NUM_CHANNEL];
		fft_negative_out[i] = samples[i * NUM_CHANNEL + 1];
		raw_PC1_data[i] = samples[i * NUM_CHANNEL + 2];
		raw_PC3_data[i] = samples[i * NUM_CHANNEL + 3];
	}
}

void init_cfft(void)
{
	arm_cfft_init_f32(&cfft_instance, DOPP_ADC_SAMPLES);
}

void CALC_DOPP_data(void)
{
	// NOTE the magic is here
	
	init_cfft(); // NOTE idk why i need to init each time for it to work
	// current theory is something is overwriting the memory for some reason

	float sample_adc1;
	float sample_adc2;
	float cfft_inout[DOPP_ADC_SAMPLES * 2];
	for (uint32_t n = 0; n < DOPP_ADC_SAMPLES; n++)
	{
		sample_adc1 = (float32_t)(ADC_DOPP_samples[n * 2]);
		sample_adc2 = (float32_t)(ADC_DOPP_samples[n * 2 + 1]);

		// TODO remove before showing off board
		// fake data using sin/cos (to test the directionality)
		if (use_fake_dopp_data) {
			const float freq = 500;
			sample_adc1 = 1000+1000*cos(2.0f*3.14159f*freq*(float)n/(float)DOPP_ADC_FS);
			sample_adc2 = 1000+1000*sin(2.0f*3.14159f*freq*(float)n/(float)DOPP_ADC_FS);
		}

		cfft_inout[n * 2] = sample_adc1;
		cfft_inout[n * 2 + 1] = sample_adc2;
		if (n < DOPP_ADC_SAMPLES / 2) {
			raw_PC1_data[n] = sample_adc1;
			raw_PC3_data[n] = sample_adc2;
		}
	}
	arm_cfft_f32(&cfft_instance, cfft_inout, 0, 1);
	
	float cmplx_mag_out[DOPP_ADC_SAMPLES];
	arm_cmplx_mag_f32(cfft_inout, cmplx_mag_out, DOPP_ADC_SAMPLES);

	for (int i = 0; i < DOPP_ADC_SAMPLES/2; i++)
	{
		fft_positive_out[i] = cmplx_mag_out[i];
		fft_negative_out[i] = cmplx_mag_out[i+DOPP_ADC_SAMPLES/2];
	}
}

int CALC_DOPP_cfft_peak(bool full_spectrum) {
	float peak_val = 0;
	int peak_idx = -1;

	for (int i=1; i<DOPP_ADC_SAMPLES/2; i++) { // ignore 0hz bin, since that has a high value
		if (fft_positive_out[i]  > peak_val) {
			peak_val = fft_positive_out[i];
			peak_idx = i;
		}
	}

	if (!full_spectrum) {
		if (peak_idx == -1) {
			return 0;
		}
		return peak_idx * DOPP_FREQ_BIN_SIZE; // TODO convert to frequency
	}

	for (int i=DOPP_ADC_SAMPLES/2+1; i<DOPP_ADC_SAMPLES; i++) {
		if (fft_positive_out[i]  > peak_val) {
			peak_val = fft_positive_out[i];
			peak_idx = i-DOPP_ADC_SAMPLES;
		}
	}
	return peak_idx * DOPP_FREQ_BIN_SIZE;
}

float CALC_DOPP_cfft_speed(int peak_freq) {
	// Doppler freq:
	// f_d = 2 * v/wavelength
	// Wavelength = c/f_c
	// f_d = 2v*f_c/c
	// v = f_d * c / (2*f_c)

	float vel = (float)peak_freq * 300000000.0f / (2.0f * 24150000000.0f);
	return vel;
}

/** ***************************************************************************
 * @brief calculates the battery level
 * @param variable with sampled value from ADC
 *
 * The approximated percentage is stored in a global variable
 *****************************************************************************/
void CALC_battery_level(uint16_t batt_sample)
{
	if (batt_sample <= 2600)
	{
		batt_lvl = 100;
	}
	else if (batt_sample <= 2000)
	{
		batt_lvl = 0;
	}
	else
	{
		batt_lvl = (batt_sample - 2000) / 6;
	}
}
