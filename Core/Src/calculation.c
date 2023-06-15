/*
 * DSP processing and FFT handling for measured data
 *
 * Author:	John Kesler	<keslejoh@students.zhaw.ch>
 * 			Linus Leuch	<leuchlin@students.zhaw.ch>
 * 			Simon Meli	<melisim1@students.zhaw.ch>
 */

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

uint32_t fft_positive_out[DOPP_ADC_SAMPLES];
uint32_t fft_negative_out[DOPP_ADC_SAMPLES / 2];
uint32_t raw_PC1_data[DOPP_ADC_SAMPLES / 2];
uint32_t raw_PC3_data[DOPP_ADC_SAMPLES / 2];

uint32_t fft_avg_vec[DOPP_ADC_SAMPLES];

uint8_t batt_lvl;

arm_cfft_instance_f32 cfft_instance;
arm_rfft_fast_instance_f32 fmcw_rfft_fast_instance;

void init_cfft(void)
{
	arm_cfft_init_f32(&cfft_instance, DOPP_ADC_SAMPLES);
	arm_rfft_fast_init_f32(&fmcw_rfft_fast_instance, DOPP_ADC_SAMPLES);
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

void FMCW_calc_data(void)
{
	init_cfft(); // TODO idk why i need to init each time for it to work
	// current theory is something is overwriting the memory for some reason

	float sample_adc1;
	float rfft_in[DOPP_ADC_SAMPLES];
	for (uint32_t n = 0; n < DOPP_ADC_SAMPLES; n++)
	{
		sample_adc1 = (float32_t)(ADC_DOPP_samples[n * 2]);
		rfft_in[n] = sample_adc1;

		if (n < DOPP_ADC_SAMPLES / 2)
		{
			raw_PC1_data[n] = sample_adc1;
			raw_PC3_data[n] = 0;
		}
	}
	float rfft_out[DOPP_ADC_SAMPLES];
	arm_rfft_fast_f32(&fmcw_rfft_fast_instance, rfft_in, rfft_out, 0);

	float cmplx_mag_out[DOPP_ADC_SAMPLES / 2];
	arm_cmplx_mag_f32(rfft_out, cmplx_mag_out, DOPP_ADC_SAMPLES / 2);

	for (int i = 0; i < DOPP_ADC_SAMPLES; i++)
	{
		fft_positive_out[i] = cmplx_mag_out[i] / 3;
		fft_negative_out[i] = 0;
	}

	float log_mag[DOPP_ADC_SAMPLES / 2];
	for (int i = 0; i < DOPP_ADC_SAMPLES / 2; i++)
	{
		log_mag[i] = 20.0f * log10(cmplx_mag_out[i]);
	}

	float scaled_log_mag[DOPP_ADC_SAMPLES / 2];
	for (int i = 0; i < DOPP_ADC_SAMPLES / 2; i++)
	{
		// apply gain based on frequency (linear w/ freq)
		scaled_log_mag[i] = (20.0f * log10((float32_t)i) + log_mag[i]) * 80.0f - 5000.0f;
	}

	// run avg
	for (int i = 0; i < DOPP_ADC_SAMPLES; i++)
	{
		fft_avg_vec[i] = (uint32_t)(((float)fft_avg_vec[i]) * AVG_WEIGHT_OLD + ((float)scaled_log_mag[i]) * AVG_WEIGHT_NEW);
	}
}

float FMCW_calc_peak()
{
	float peak_val = 0;
	int peak_idx = -1;
	int start_idx = 3;

	for (int i = start_idx; i < DOPP_ADC_SAMPLES; i++)
	{ // ignore 0hz bin, since that has a high value
		if (fft_avg_vec[i] > peak_val)
		{
			peak_val = fft_avg_vec[i];
			peak_idx = i;
		}
	}

	if (peak_idx == -1)
	{
		return 0;
	}
	return peak_idx * DOPP_FREQ_BIN_SIZE * 2;
}

float FMCW_calc_distance(float peak_freq)
{
	float dist = peak_freq / 1734.5f; // TODO scaling calculated from excel
	return dist;
}
