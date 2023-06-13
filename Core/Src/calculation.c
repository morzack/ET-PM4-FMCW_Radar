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

uint32_t fft_positive_out[DOPP_ADC_SAMPLES];	 ///< samples for channel 1 (PAD 1)
uint32_t fft_negative_out[DOPP_ADC_SAMPLES / 2]; ///< samples for channel 2 (PAD 2)
uint32_t raw_PC1_data[DOPP_ADC_SAMPLES / 2];	 ///< samples for channel 3 (PAD 3)
uint32_t raw_PC3_data[DOPP_ADC_SAMPLES / 2];	 ///< samples for channel 4 (PAD 4)
uint32_t raw_PC4_data[FMCW_ADC_SAMPLES];

uint32_t fft_avg_vec[DOPP_ADC_SAMPLES];

uint8_t batt_lvl; ///< battery voltage level

arm_cfft_instance_f32 cfft_instance;
arm_rfft_instance_f32 fmcw_rfft_instance;
arm_rfft_fast_instance_f32 fmcw_rfft_fast_instance;

void init_cfft(void)
{
	arm_cfft_init_f32(&cfft_instance, DOPP_ADC_SAMPLES);
	//	arm_rfft_fast_init_f32(&fmcw_rfft_instance, FMCW_ADC_SAMPLES);
	arm_rfft_fast_init_f32(&fmcw_rfft_fast_instance, FMCW_ADC_SAMPLES);
}

void CALC_DOPP_data(void)
{
	// TODO check memory and see why this needs to be called on each iter to work
	// current theory is something is overwriting the memory for some reason
	init_cfft();

	float sample_adc1;
	float sample_adc2;
	float cfft_inout[DOPP_ADC_SAMPLES * 2];
	for (uint32_t n = 0; n < DOPP_ADC_SAMPLES; n++)
	{
		sample_adc1 = (float32_t)(ADC_DOPP_samples[n * 2]);
		// sample_adc2 = (float32_t)(ADC_DOPP_samples[n * 2 + 1]);
		sample_adc2 = 0;

		cfft_inout[n * 2] = sample_adc1;
		cfft_inout[n * 2 + 1] = sample_adc2;
		if (n < DOPP_ADC_SAMPLES / 2)
		{
			raw_PC1_data[n] = sample_adc1;
			raw_PC3_data[n] = sample_adc2;
		}
	}
	arm_cfft_f32(&cfft_instance, cfft_inout, 0, 1);

	float cmplx_mag_out[DOPP_ADC_SAMPLES];
	arm_cmplx_mag_f32(cfft_inout, cmplx_mag_out, DOPP_ADC_SAMPLES);

	for (int i = 0; i < DOPP_ADC_SAMPLES / 2; i++)
	{
		fft_positive_out[i] = cmplx_mag_out[i];
		fft_negative_out[i] = cmplx_mag_out[i + DOPP_ADC_SAMPLES / 2];
	}

	// run avg
	for (int i = 0; i < DOPP_ADC_SAMPLES / 2; i++)
	{
		fft_avg_vec[i] = (uint32_t)(((float)fft_avg_vec[i]) * AVG_WEIGHT_OLD + ((float)fft_positive_out[i]) * AVG_WEIGHT_NEW);
	}

	for (int i = 0; i < DOPP_ADC_SAMPLES / 2; i++)
	{
		raw_PC1_data[i] *= 1.2f;
	}
}

int CALC_DOPP_cfft_peak(bool full_spectrum)
{
	float peak_val = 0;
	int peak_idx = -1;
	int start_idx = 3; // ignore low bins bc/ DC freq + high return

	for (int i = start_idx; i < DOPP_ADC_SAMPLES / 2; i++)
	{
		if (fft_positive_out[i] > peak_val)
		{
			peak_val = fft_positive_out[i];
			peak_idx = i;
		}
	}

	if (!full_spectrum)
	{
		if (peak_idx == -1)
		{
			return 0;
		}
		return peak_idx * DOPP_FREQ_BIN_SIZE;
	}

	// fallthrough code for if we want to consider negative frequencies (ex: dopp w/ neg velocities)
	for (int i = DOPP_ADC_SAMPLES / 2 + 1; i < DOPP_ADC_SAMPLES; i++)
	{
		if (fft_positive_out[i] > peak_val)
		{
			peak_val = fft_positive_out[i];
			peak_idx = i - DOPP_ADC_SAMPLES;
		}
	}
	return peak_idx * DOPP_FREQ_BIN_SIZE;
}

float CALC_DOPP_cfft_speed(int peak_freq)
{
	// Doppler freq:
	// f_d = 2 * v/wavelength
	// Wavelength = c/f_c
	// f_d = 2v*f_c/c
	// v = f_d * c / (2*f_c)

	float vel = (float)peak_freq * 300000000.0f / (2.0f * 24150000000.0f);
	return vel;
}

/*
 * calculates battery level and stores in *global* variable
 */
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

// void FMCW_calc_data(void) {
// 	init_cfft();

// 	float sample;
// 	float rfft_in[FMCW_ADC_SAMPLES];
// 	for (uint32_t n=0; n<FMCW_ADC_SAMPLES; n++) {
// 		sample = (float32_t)(ADC_FMCW_samples[n]);
// 		rfft_in[n] = sample;
// 		raw_PC4_data[n] = sample;
// 		raw_PC1_data[n] = sample;
// 		raw_PC3_data[n] = 0;
// 	}
// 	float rfft_out[FMCW_ADC_SAMPLES];
// 	arm_rfft_fast_f32(&fmcw_rfft_instance, rfft_in, rfft_out, 0);

// 	float cmplx_mag_out[FMCW_ADC_SAMPLES];
// 	arm_cmplx_mag_f32(rfft_out, cmplx_mag_out, FMCW_ADC_SAMPLES);

// 	for (int i=0; i<FMCW_ADC_SAMPLES/2; i++) {
// 		fft_positive_out[i] = cmplx_mag_out[i];
// 		fft_negative_out[i] = 0;
// 	}
// }

// TODO refactor, redule duplicate code w/ DOPP
void FMCW_calc_data(void)
{
	init_cfft(); // NOTE idk why i need to init each time for it to work
	// current theory is something is overwriting the memory for some reason

	float sample_adc1;
	float sample_adc2;
	float rfft_in[DOPP_ADC_SAMPLES];
	for (uint32_t n = 0; n < DOPP_ADC_SAMPLES; n++)
	{
		sample_adc1 = (float32_t)(ADC_DOPP_samples[n * 2]);
		// sample_adc2 = (float32_t)(ADC_DOPP_samples[n * 2 + 1]);
		sample_adc2 = 0;

		// TODO remove before showing off board
		// fake data using sin/cos (to test the directionality)
		if (use_fake_dopp_data)
		{
			const float freq = 500;
			sample_adc1 = 1000 + 1000 * cos(2.0f * 3.14159f * freq * (float)n / (float)DOPP_ADC_FS);
			sample_adc2 = 1000 + 1000 * sin(2.0f * 3.14159f * freq * (float)n / (float)DOPP_ADC_FS);
		}

		rfft_in[n] = sample_adc1;
		// cfft_inout[n * 2 + 1] = sample_adc2;
		if (n < DOPP_ADC_SAMPLES / 2)
		{
			raw_PC1_data[n] = sample_adc1;
			raw_PC3_data[n] = sample_adc1;
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
	float avg_mag = 0;
	for (int i = 0; i < DOPP_ADC_SAMPLES / 2; i++)
	{
		scaled_log_mag[i] = (20.0f * log10((float32_t)i) + log_mag[i]) * 80.0f - 5000.0f; //+500.0f;
		avg_mag += scaled_log_mag[i];
	}
	// avg_mag /= DOPP_ADC_SAMPLES/2;
	// for (int i=0; i<DOPP_ADC_SAMPLES/2; i++) {
	// 	// scaled_log_mag[i] -= avg_mag/2;
	// 	scaled_log_mag[i] *= 100.0f;
	// }

	// for (int i=0; i<DOPP_ADC_SAMPLES/2; i++) {
	// 	scaled_log_mag[i] = (20.0f*log10((float32_t)i) + log_mag[i]);
	// }

	// run avg
	for (int i = 0; i < DOPP_ADC_SAMPLES; i++)
	{
		// fft_avg_vec[i] = (uint32_t) (((float) fft_avg_vec[i]) * AVG_WEIGHT_OLD + ((float) fft_positive_out[i]) * AVG_WEIGHT_NEW);
		fft_avg_vec[i] = (uint32_t)(((float)fft_avg_vec[i]) * AVG_WEIGHT_OLD + ((float)scaled_log_mag[i]) * AVG_WEIGHT_NEW);
	}
}

float FMCW_calc_peak()
{
	float peak_val = 0;
	int peak_idx = -1;
	int start_idx = 3;

	for (int i = start_idx; i < FMCW_ADC_SAMPLES / 2; i++)
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
	return peak_idx * FMCW_FREQ_BIN_SIZE * 2;
}

float FMCW_calc_distance(float peak_freq)
{
	float dist = peak_freq / 1734.5f;
	return dist;
}
