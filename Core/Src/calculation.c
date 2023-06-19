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

uint32_t raw_PC1_stream[DOPP_ADC_SAMPLES];
uint32_t raw_PC3_stream[DOPP_ADC_SAMPLES];
uint32_t raw_PC5_stream[FMCW_ADC_SAMPLE_COUNT];

uint32_t fft_avg_vec_dopp[FMCW_ADC_SAMPLE_COUNT];
uint32_t fft_avg_vec_fmcw[FMCW_ADC_SAMPLE_COUNT / 2];

uint32_t fft_positive_out[DOPP_ADC_SAMPLES / 2];
uint32_t fft_negative_out[DOPP_ADC_SAMPLES / 2];

uint8_t batt_lvl;

arm_cfft_instance_f32 dopp_cfft_instance;
arm_rfft_fast_instance_f32 fmcw_rfft_fast_instance;

void init_cfft(void)
{
	arm_cfft_init_f32(&dopp_cfft_instance, DOPP_ADC_SAMPLES);
	arm_rfft_fast_init_f32(&fmcw_rfft_fast_instance, FMCW_ADC_SAMPLE_COUNT);
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
	init_cfft();

	float fmcw_sample;
	float rfft_in[FMCW_ADC_SAMPLE_COUNT];
	for (uint32_t n = 0; n < FMCW_ADC_SAMPLE_COUNT; n++)
	{
		fmcw_sample = (float32_t)(FMCW_ADC_samples[n * 2]);
		rfft_in[n] = fmcw_sample;
		raw_PC5_stream[n] = fmcw_sample;
	}
	float rfft_out[FMCW_ADC_SAMPLE_COUNT];
	arm_rfft_fast_f32(&fmcw_rfft_fast_instance, rfft_in, rfft_out, 0);

	float cmplx_mag_out[FMCW_ADC_SAMPLE_COUNT / 2];
	arm_cmplx_mag_f32(rfft_out, cmplx_mag_out, FMCW_ADC_SAMPLE_COUNT / 2);

	float log_mag[FMCW_ADC_SAMPLE_COUNT / 2];
	for (int i = 0; i < FMCW_ADC_SAMPLE_COUNT / 2; i++)
	{
		log_mag[i] = 20.0f * log10(cmplx_mag_out[i]);
	}

	float scaled_log_mag[FMCW_ADC_SAMPLE_COUNT / 2];
	for (int i = 0; i < FMCW_ADC_SAMPLE_COUNT / 2; i++)
	{
		// apply gain based on frequency (linear w/ freq)
		scaled_log_mag[i] = (20.0f * log10((float32_t)i) + log_mag[i]) * 80.0f - 5000.0f;
	}

	// run avg
	for (int i = 0; i < FMCW_ADC_SAMPLE_COUNT / 2; i++)
	{
		fft_avg_vec_fmcw[i] = (uint32_t)(((float)fft_avg_vec_fmcw[i]) * AVG_WEIGHT_OLD + ((float)scaled_log_mag[i]) * AVG_WEIGHT_NEW);
	}
}

float FMCW_calc_peak()
{
	float peak_val = 0;
	int peak_idx = -1;
	int start_idx = 1;
	int stop_idx = FMCW_MAX_BIN+1;

	for (int i = start_idx; i < stop_idx; i++)
	{ // ignore 0hz bin, since that has a high value
		if (fft_avg_vec_fmcw[i] > peak_val)
		{
			peak_val = fft_avg_vec_fmcw[i];
			peak_idx = i;
		}
	}

	if (peak_idx == -1)
	{
		return 0;
	}
	return peak_idx * FMCW_FREQ_BIN_SIZE * 2;
}

float FMCW_calc_distance(float peak_freq)
{
	float dist = peak_freq / 1734.5f; // TODO scaling calculated from excel
	return dist;
}

void CALC_DOPP_data(void)
{
	init_cfft();

	float sample_adc1;
	float sample_adc2;
	float cfft_inout[DOPP_ADC_SAMPLES * 2];
	for (uint32_t n = 0; n < DOPP_ADC_SAMPLES; n++)
	{
		sample_adc1 = (float32_t)(ADC_DOPP_samples[n * 2]);
		sample_adc2 = (float32_t)(ADC_DOPP_samples[n * 2 + 1]);

		// const float freq = 500;
		// sample_adc1 = 1000+1000*cos(2.0f*3.14159f*freq*(float)n/(float)DOPP_ADC_FS);
		// sample_adc2 = 1000+1000*sin(2.0f*3.14159f*freq*(float)n/(float)DOPP_ADC_FS);

		cfft_inout[n * 2] = sample_adc1;
		cfft_inout[n * 2 + 1] = sample_adc2;
		
		raw_PC1_stream[n] = sample_adc1;
		raw_PC3_stream[n] = sample_adc2;
	}
	arm_cfft_f32(&dopp_cfft_instance, cfft_inout, 0, 1);
	
	float cmplx_mag_out[DOPP_ADC_SAMPLES];
	arm_cmplx_mag_f32(cfft_inout, cmplx_mag_out, DOPP_ADC_SAMPLES);

	for (int i = 0; i < DOPP_ADC_SAMPLES/2; i++)
	{
		fft_positive_out[i] = (uint32_t)(log((double)cmplx_mag_out[i])*20)*LOG_GRAPH_SCALING;
		fft_negative_out[i] = (uint32_t)(log((double)cmplx_mag_out[i+DOPP_ADC_SAMPLES/2])*20)*LOG_GRAPH_SCALING;
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
