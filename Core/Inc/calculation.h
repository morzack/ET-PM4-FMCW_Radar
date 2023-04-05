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

/******************************************************************************
 * Types
 *****************************************************************************/
 typedef struct {
    uint32_t distance;
    int32_t angle;
 } PAD_measurment_t;

 typedef struct {
    uint32_t current;
 } COIL_measurment_t;

/******************************************************************************
 * Defines
 *****************************************************************************/
extern uint8_t batt_lvl;

extern PAD_measurment_t PAD;
extern COIL_measurment_t COIL;

extern uint32_t channel1[_DOPP_ADC_SAMPLES/2];    ///< samples for channel 1 (PAD 1)
extern uint32_t channel2[_DOPP_ADC_SAMPLES/2];    ///< samples for channel 2 (PAD 2)
extern uint32_t channel3[ADC_NUMS];    ///< samples for channel 3 (PAD 3)
extern uint32_t channel4[ADC_NUMS];    ///< samples for channel 4 (PAD 4)
extern uint32_t channel5[ADC_NUMS];    ///< samples for channel 5 (COIL)

#define FLOAT_AVG_LENGTH 20
 /******************************************************************************
 * Functions
 *****************************************************************************/
void CALC_battery_level(uint16_t batt_sample);
void CALC_pad_measurement(void);
void CALC_coil_measurement(void);
void CALC_DOPP_data(void);
int CALC_DOPP_cfft_peak(void);
void init_cfft(void);

#endif
