#ifndef CALCULATION_H_
#define CALCULATION_H_

#include "measuring.h"

#include <stdbool.h>

#define AVG_WEIGHT_OLD 0.95
#define AVG_WEIGHT_NEW (1 - AVG_WEIGHT_OLD)

extern uint8_t batt_lvl;

extern uint32_t raw_PC1_data[DOPP_ADC_SAMPLES / 2];
extern uint32_t raw_PC3_data[DOPP_ADC_SAMPLES / 2];
extern uint32_t fft_avg_vec[DOPP_ADC_SAMPLES];

void CALC_battery_level(uint16_t batt_sample);

void init_cfft(void);

void FMCW_calc_data(void);
float FMCW_calc_peak();
float FMCW_calc_distance(float peak_freq);

#endif
