/** ***************************************************************************
 * @file
 * @brief See calculation.c
 *
 * Prefix CALC
 *
 *****************************************************************************/

#ifndef CALCULATION_H_
#define CALCULATION_H_
/******************************************************************************
 * Includes
 *****************************************************************************/
#include "measuring.h"

#include <stdbool.h>

/******************************************************************************
 * Types
 *****************************************************************************/

/******************************************************************************
 * Defines
 *****************************************************************************/
extern uint8_t batt_lvl;

extern uint32_t fft_positive_out[DOPP_ADC_SAMPLES];    ///< samples for channel 1 (PAD 1)
extern uint32_t fft_negative_out[DOPP_ADC_SAMPLES/2];    ///< samples for channel 2 (PAD 2)
extern uint32_t raw_PC1_data[DOPP_ADC_SAMPLES/2];    ///< samples for channel 3 (PAD 3)
extern uint32_t raw_PC3_data[DOPP_ADC_SAMPLES/2];    ///< samples for channel 4 (PAD 4)
extern uint32_t raw_PC4_data[FMCW_ADC_SAMPLES];

extern uint32_t fft_avg_vec[DOPP_ADC_SAMPLES];

#define AVG_WEIGHT_OLD 0.95
#define AVG_WEIGHT_NEW (1-AVG_WEIGHT_OLD)

#define FLOAT_AVG_LENGTH 20
 /******************************************************************************
 * Functions
 *****************************************************************************/
void CALC_battery_level(uint16_t batt_sample);
void init_cfft(void);

void FMCW_calc_data(void);
float FMCW_calc_peak();
float FMCW_calc_distance(float peak_freq);

#endif
