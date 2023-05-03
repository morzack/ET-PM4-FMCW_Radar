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

extern uint32_t fft_positive_out[DOPP_ADC_SAMPLES/2];    ///< samples for channel 1 (PAD 1)
extern uint32_t fft_negative_out[DOPP_ADC_SAMPLES/2];    ///< samples for channel 2 (PAD 2)
extern uint32_t raw_PC1_data[DOPP_ADC_SAMPLES/2];    ///< samples for channel 3 (PAD 3)
extern uint32_t raw_PC3_data[DOPP_ADC_SAMPLES/2];    ///< samples for channel 4 (PAD 4)

extern uint32_t raw_PC4_data[FMCW_ADC_SAMPLES];

#define FLOAT_AVG_LENGTH 20
 /******************************************************************************
 * Functions
 *****************************************************************************/
void CALC_battery_level(uint16_t batt_sample);
void CALC_DOPP_data(void);
int CALC_DOPP_cfft_peak(bool full_spectrum);
float CALC_DOPP_cfft_speed(int peak_freq);
void init_cfft(void);

void FMCW_calc_data(void);

#endif
