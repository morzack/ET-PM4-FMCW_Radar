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
#include "menu.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Variables
 *****************************************************************************/
PAD_measurment_t PAD;	///< struct for storing Pad measurement results
COIL_measurment_t COIL; ///< struct for storing Coil measurement results

uint32_t channel1[_DOPP_ADC_SAMPLES / 2]; ///< samples for channel 1 (PAD 1)
uint32_t channel2[_DOPP_ADC_SAMPLES / 2]; ///< samples for channel 2 (PAD 2)
uint32_t channel3[_DOPP_ADC_SAMPLES/2];			  ///< samples for channel 3 (PAD 3)
uint32_t channel4[_DOPP_ADC_SAMPLES/2];			  ///< samples for channel 4 (PAD 4)
uint32_t channel5[ADC_NUMS];			  ///< samples for channel 5 (COIL)

uint32_t float_avg_array[FLOAT_AVG_LENGTH];	 ///< array used for moving average
uint32_t float_avg_array2[FLOAT_AVG_LENGTH]; ///< array used for moving average of Pad2
uint32_t float_avg_array4[FLOAT_AVG_LENGTH]; ///< array used for moving average of Pad4

uint32_t float_avg_angle2; ///< moving average of Pad2
uint32_t float_avg_angle4; ///< moving average of Pad4

uint8_t batt_lvl; ///< battery voltage level

arm_cfft_instance_f32 cfft_instance;
arm_rfft_fast_instance_f32 rfft_instance;

const uint32_t LUT_Pad1[] = {
#include "LUTpad3.csv"
};

const uint32_t LUT_Pad2[] = {
#include "LUTpad2.csv"
};

const uint32_t LUT_Pad4[] = {
#include "LUTpad4.csv"
};

const uint32_t LUT_coil[] = {
#include "lutCurrent.csv"
};
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
		channel1[i] = samples[i * NUM_CHANNEL];
		channel2[i] = samples[i * NUM_CHANNEL + 1];
		channel3[i] = samples[i * NUM_CHANNEL + 2];
		channel4[i] = samples[i * NUM_CHANNEL + 3];
		channel5[i] = samples[i * NUM_CHANNEL + 4];
	}
}

void init_cfft(void)
{
	arm_cfft_init_f32(&cfft_instance, _DOPP_ADC_SAMPLES);
	arm_rfft_fast_init_f32(&rfft_instance, _DOPP_ADC_SAMPLES);
}

void CALC_DOPP_data(void)
{
	// do FFT
	//	for (int i=0; i<_DOPP_ADC_SAMPLES; i++) {
	//		channel1[i] = ADC_DOPP_samples[i*2];
	//		channel2[i] = ADC_DOPP_samples[i*2+1];
	//	}
	
	init_cfft(); // NOTE idk why i need to init each time for it to work
	// current theory is something is overwriting the memory for some reason

	float sample_adc1;
	float sample_adc2;
	float cfft_inout[_DOPP_ADC_SAMPLES * 2];
	//	float32_t rfft_inout[_DOPP_ADC_SAMPLES];
	//	float32_t rfft_cout[_DOPP_ADC_SAMPLES/2];
	for (uint32_t n = 0; n < _DOPP_ADC_SAMPLES; n++)
	{
		sample_adc1 = (float32_t)(ADC_DOPP_samples[n * 2]);
		sample_adc2 = (float32_t)(ADC_DOPP_samples[n * 2 + 1]);
		// todo : fill into the inout buffer for the FFT
		// format : { real[0], imag[0], real[1], imag[1], real[2], imag[2] ... }
		cfft_inout[n * 2] = sample_adc1;
		//		cfft_inout[n*2+1] = sample_adc2;
		cfft_inout[n * 2 + 1] = sample_adc2;
		//		rfft_inout[n] = sample_adc1;
		if (n < _DOPP_ADC_SAMPLES / 2) {
			channel3[n] = sample_adc1;
			channel4[n] = sample_adc2;
		}
	}
	//	arm_cfft_f32(&cfft_instance, cfft_inout, IFFT_FLAG, DO_B
	arm_cfft_f32(&cfft_instance, cfft_inout, 0, 1);
	//	arm_rfft_fast_f32(&rfft_instance, rfft_inout, rfft_cout, 0);

	//	float32_t cmplx_mag[_DOPP_ADC_SAMPLES];
	//	for (int i=0; i<_DOPP_ADC_SAMPLES/2; i++) {
	//		cmplx_mag[2*i] = cfft_inout[i];
	//		cmplx_mag[2*i+1] = cfft_inout[i];
	//	}
	float cmplx_mag_out[_DOPP_ADC_SAMPLES];
	arm_cmplx_mag_f32(cfft_inout, cmplx_mag_out, _DOPP_ADC_SAMPLES);

	for (int i = 0; i < _DOPP_ADC_SAMPLES/2; i++)
	{
		// channel1[i] = cmplx_mag_out[i]; // TODO fake data
		channel1[i] = cmplx_mag_out[i];
		channel2[i] = cmplx_mag_out[i+_DOPP_ADC_SAMPLES/2];
		// channel1[i] = i*100;
		//		channel1[i] = cfft_inout[i];
		// channel2[i] = cmplx_mag_out[i];
		//		channel2[i] = cfft_inout[i*2+1];
	}
	//	for (int i=0; i<_DOPP_ADC_SAMPLES/2; i++) {
	//		channel1[i] = cmplx_mag_out[i];
	//		channel2[i] = 0;
	//	}
}

int CALC_DOPP_cfft_peak(bool full_spectrum) {
	float peak_val = 0;
	int peak_idx = -1;

	for (int i=1; i<_DOPP_ADC_SAMPLES/2; i++) { // ignore 0hz bin, since that has a high value
		if (channel1[i]  > peak_val) {
			peak_val = channel1[i];
			peak_idx = i;
		}
	}

	if (!full_spectrum) {
		if (peak_idx == -1) {
			return 0;
		}
		return peak_idx * _DOPP_FREQ_BIN_SIZE; // TODO convert to frequency
	}

	for (int i=_DOPP_ADC_SAMPLES/2+1; i<_DOPP_ADC_SAMPLES; i++) {
		if (channel1[i]  > peak_val) {
			peak_val = channel1[i];
			peak_idx = i-_DOPP_ADC_SAMPLES;
		}
	}
	return peak_idx * _DOPP_FREQ_BIN_SIZE;
}

float CALC_DOPP_cfft_speed(int peak_freq) {
	// Doppler freq:
	// f_d = 2 * v/wavelength
	// Wavelength = c/f_c
	// f_d = 2v*f_c/c
	// v = f_d * c / (2*f_c)

	float vel = (float)peak_freq * 300000000.0f / (2.0 * 24150000000.0f);
	return vel;
}

/** ***************************************************************************
 * @brief resets arrays used for averaging
 *
 * @note sets all elements to zero
 ******************************************************************************/
void CALC_reset_average(void)
{
	for (int i = 0; i < FLOAT_AVG_LENGTH; i++)
	{
		float_avg_array[i] = 0;
		float_avg_array2[i] = 0;
		float_avg_array4[i] = 0;
	}
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

/** ***************************************************************************
 * @brief calculates a moving average over a predefined number of measurements
 * @param new sample for moving average
 *
 * @note Number of samples used for the moving average defined in FLOAT_AVG_LENGTH
 ******************************************************************************/
uint32_t CALC_floating_average(uint32_t new_value)
{
	uint32_t float_avg;

	for (int i = 0; i < FLOAT_AVG_LENGTH; i++)
	{
		float_avg_array[FLOAT_AVG_LENGTH - i] = float_avg_array[FLOAT_AVG_LENGTH - i - 1];
	}

	float_avg_array[0] = new_value;

	for (int i = 0; i < FLOAT_AVG_LENGTH; i++)
	{
		float_avg += float_avg_array[i];
	}

	return float_avg = float_avg / FLOAT_AVG_LENGTH;
}

/** ***************************************************************************
 * @brief calculates a moving average over a predefined number of measurements for angle
 * @param [in] 2 new samples for moving average
 *
 * @note Number of samples used for the moving average defined in FLOAT_AVG_LENGTH
 ******************************************************************************/
void CALC_floating_average_angle(uint32_t new_value2, uint32_t new_value4)
{
	float_avg_angle2 = 0;
	float_avg_angle4 = 0;

	for (int i = 0; i < FLOAT_AVG_LENGTH; i++)
	{
		float_avg_array2[FLOAT_AVG_LENGTH - i] = float_avg_array2[FLOAT_AVG_LENGTH - i - 1];
		float_avg_array4[FLOAT_AVG_LENGTH - i] = float_avg_array4[FLOAT_AVG_LENGTH - i - 1];
	}

	float_avg_array2[0] = new_value2;
	float_avg_array4[0] = new_value4;

	for (int i = 0; i < FLOAT_AVG_LENGTH; i++)
	{
		float_avg_angle2 += float_avg_array2[i];
		float_avg_angle4 += float_avg_array4[i];
	}

	float_avg_angle2 = float_avg_angle2 / FLOAT_AVG_LENGTH;
	float_avg_angle4 = float_avg_angle4 / FLOAT_AVG_LENGTH;
}

/** ***************************************************************************
 * @brief calculates the root mean square of the measured samples
 * @param desired channel for root mean square
 *
 * @note Only works with an array of ADC_NUMS size
 ******************************************************************************/
uint32_t CALC_rms_value(uint32_t channel[])
{
	float mean = 0;
	uint32_t square = 0;

	for (int i = 0; i < ADC_NUMS; i++)
	{
		mean += channel[i];
	}
	mean = mean / ADC_NUMS;

	for (int i = 0; i < ADC_NUMS; i++)
	{
		square += pow(channel[i] - mean, 2);
	}
	mean = square / ADC_NUMS;
	return sqrt(mean);
}

/** ***************************************************************************
 * @brief searches LUT for appropriate distance value
 * @param Root Mean Square value of Pad 1
 * @return distance in mm
 *
 * @note only works for Pad 1
 ******************************************************************************/
uint32_t CALC_pad_distance(uint32_t rms_val)
{
	uint32_t size = sizeof(LUT_Pad1) / sizeof(uint32_t);

	if (rms_val > LUT_Pad1[0])
	{
		return 0;
	}
	else if (rms_val < LUT_Pad1[size - 1])
	{
		return 9999;
	}
	else
	{
		for (int i = 0; i < size; i++)
		{
			if ((rms_val <= LUT_Pad1[i] && rms_val > LUT_Pad1[i + 1]) || (rms_val == LUT_Pad1[i] && rms_val == LUT_Pad1[i + 1]))
			{
				return i;
			}
		}
	}
}

/** ***************************************************************************
 * @brief calculates angle
 * @param Root Mean Square value of Pad 2
 * @param Root Mean Square value of Pad 4
 * @return angle in deg
 *
 * @note angle is a linear approximation by multiplying the value with a scaling factor
 ******************************************************************************/
int32_t CALC_pad_angle(uint32_t value2, uint32_t value4)
{
	float temp;
	float data2 = value2;
	float data4 = value4;
	int32_t angle;

	temp = 0.15 * (data2 - data4);
	angle = temp;

	return angle;
}

/******************************************************************************
 * @brief searches LUT for appropriate current value
 * @param Root Mean Square value of Coil
 * @return current in mA
 *
 * @note Only works for a current between 0 and 10 Amperes
 ******************************************************************************/
uint32_t CALC_current(uint32_t rms_val)
{
	uint32_t size = sizeof(LUT_coil) / sizeof(uint32_t);

	if (rms_val < LUT_coil[0])
	{
		return 0;
	}
	else if (rms_val > LUT_coil[size - 1])
	{
		return 10;
	}
	else
	{
		for (int i = 0; i < size; i++)
		{
			if ((rms_val >= LUT_coil[i] && rms_val < LUT_coil[i + 1]))
			{
				return i;
			}
		}
	}
}

/******************************************************************************
 * @brief calls all necessary functions that are needed for a pad measurement
 *
 * @note all results are stored in a global struct
 ******************************************************************************/
void CALC_pad_measurement(void)
{
	CALC_separate_data(ADC_samples);

	uint32_t rms_PAD1 = CALC_rms_value(channel1);
	uint32_t rms_PAD2 = CALC_rms_value(channel2);
	uint32_t rms_PAD3 = CALC_rms_value(channel3);
	uint32_t rms_PAD4 = CALC_rms_value(channel4);
	uint32_t average;

	if (single)
	{
		PAD.distance = CALC_pad_distance(rms_PAD1);
		PAD.angle = CALC_pad_angle(rms_PAD2, rms_PAD4);
	}
	else
	{
		average = CALC_floating_average(rms_PAD1);
		CALC_floating_average_angle(rms_PAD2, rms_PAD4);

		PAD.distance = CALC_pad_distance(average);
		PAD.angle = CALC_pad_angle(float_avg_angle2, float_avg_angle4);
	}
}

/******************************************************************************
 * @brief calls all necessary functions that are needed for a coil measurement
 *
 * @note all results are stored in a global struct
 ******************************************************************************/
void CALC_coil_measurement(void)
{
	CALC_separate_data(ADC_samples);
	uint32_t rms_coil = CALC_rms_value(channel5);
	uint32_t average;

	if (single)
	{
		COIL.current = CALC_current(rms_coil);
	}
	else
	{
		average = CALC_floating_average(rms_coil);
		COIL.current = CALC_current(average);
	}
}
