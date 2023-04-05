/** ***************************************************************************
 * @file
 * @brief See measuring.c
 *
 * Prefixes MEAS, ADC, DAC
 *
 *****************************************************************************/

#ifndef MEAS_H_
#define MEAS_H_


/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdbool.h>
#include <stdio.h>


/******************************************************************************
 * Defines
 *****************************************************************************/
#define ADC_NUMS        64      ///< Number of samples
#define NUM_CHANNEL     5       ///< Number of channels
#define ADC_DAC_RES     12      ///< Resolution
extern bool MEAS_data1_ready;
extern bool MEAS_data3_ready;
extern uint32_t MEAS_input_count;
extern bool DAC_active;
extern bool ACC_data_rdy;
extern uint32_t ADC_samples[NUM_CHANNEL * ADC_NUMS];
extern uint16_t batt_sample;

// DOPP defines
// #define _DOPP_ADC_FS 			60000 // 64kHz for FMCW
#define _DOPP_ADC_FS            4000 // 4kHz for DOPP
//#define _DOPP_ADC_CLOCK
//#define _DOPP_ADC_CLOCK_PS
#define _DOPP_TIM_CLOCK			84000000
#define _DOPP_TIM_TOP			9
#define _DOPP_TIM_PRESCALE		(_DOPP_TIM_CLOCK/_DOPP_ADC_FS/(_DOPP_TIM_TOP+1)-1)
#define _DOPP_ADC_SAMPLES		256
#define _DOPP_ADC_SAMPLES_ARR	256
#define _DOPP_ADC_SAMPLES_ZPAD	_DOPP_ADC_SAMPLES_ARR-_DOPP_ADC_SAMPLES
#define _DOPP_FREQ_BIN_SIZE     _DOPP_ADC_FS/_DOPP_ADC_SAMPLES_ARR

extern bool MEAS_DOPP_ready;
extern uint32_t MEAS_DOPP_input_count;
extern uint32_t ADC_DOPP_samples[_DOPP_ADC_SAMPLES * 2]; // needs zero padding later


/******************************************************************************
 * Functions
 *****************************************************************************/
void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(void);
void DAC_reset(void);
void DAC_init(void);
void DAC_increment(void);
void ADC_reset(void);
void ADC2_IN15_single_init(void);
void ADC2_IN15_single_read(void);
void ADC1_IN5_IN13_scan_init(void);
void ADC1_IN5_IN13_scan_start(void);
void ADC3_IN4_IN11_IN13_scan_init(void);
void ADC3_IN4_IN11_IN13_scan_start(void);
void reset_data(void);
void copy_data(void);
bool batteryStatus(void);

void ADC_DOPP_scan_init(void);
void ADC_DOPP_scan_start(void);
void DOPP_copy_data(void);
void DOPP_reset_data(void);


#endif
