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
extern bool MEAS_data1_ready; // PC1
extern bool MEAS_data3_ready; // PC3
extern uint32_t MEAS_input_count;
extern bool DAC_active;
extern bool ACC_data_rdy;
extern uint32_t ADC_samples[NUM_CHANNEL * ADC_NUMS];
extern uint16_t batt_sample;

// DOPP defines
// #define DOPP_ADC_FS 			60000 // 64kHz for FMCW
#define DOPP_ADC_FS             4000 // 4kHz for DOPP
#define DOPP_TIM_CLOCK			84000000
#define DOPP_TIM_TOP			9
#define DOPP_TIM_PRESCALE		(DOPP_TIM_CLOCK/DOPP_ADC_FS/(DOPP_TIM_TOP+1)-1)
#define DOPP_ADC_SAMPLES		128 // TODO only sampling this high for firmware test
#define DOPP_ADC_SAMPLES_ARR	128
#define DOPP_ADC_SAMPLES_ZPAD   DOPP_ADC_SAMPLES_ARR-DOPP_ADC_SAMPLES
#define DOPP_FREQ_BIN_SIZE      DOPP_ADC_FS/DOPP_ADC_SAMPLES_ARR

extern bool MEAS_DOPP_ready;
extern uint32_t MEAS_DOPP_input_count;
extern uint32_t ADC_DOPP_samples[DOPP_ADC_SAMPLES * 2]; // needs zero padding later

extern bool use_fake_dopp_data; // TODO for firmware test only


/******************************************************************************
 * Functions
 *****************************************************************************/
void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(void);
void DAC_reset(void);
void DAC_init(void);
void DAC_increment(void);
void ADC_reset(void);
void reset_data(void);
void copy_data(void);
bool batteryStatus(void);

void ADC_DOPP_scan_init(void);
void ADC_DOPP_scan_start(void);
void DOPP_copy_data(void);
void DOPP_reset_data(void);


#endif
