#ifndef CALCULATION_H_
#define CALCULATION_H_

#include "measuring.h"

#include <stdbool.h>

#define AVG_WEIGHT_OLD 0.95
#define AVG_WEIGHT_NEW (1 - AVG_WEIGHT_OLD)

extern uint8_t batt_lvl;

extern uint32_t raw_PC1_stream[DOPP_ADC_SAMPLES];
extern uint32_t raw_PC3_stream[DOPP_ADC_SAMPLES];
extern uint32_t raw_PC5_stream[FMCW_ADC_SAMPLE_COUNT];
extern uint32_t fft_positive_out[DOPP_ADC_SAMPLES / 2];
extern uint32_t fft_negative_out[DOPP_ADC_SAMPLES / 2];
extern uint32_t fft_avg_vec_fmcw[FMCW_ADC_SAMPLE_COUNT / 2];

void CALC_battery_level(uint16_t batt_sample);

void init_cfft(void);

void FMCW_calc_data(void);
float FMCW_calc_peak();
float FMCW_calc_distance(float peak_freq);

void CALC_DOPP_data(void);
int CALC_DOPP_cfft_peak(bool full_spectrum);
float CALC_DOPP_cfft_speed(int peak_freq);

#endif
