#ifndef MEAS_H_
#define MEAS_H_

#include <stdbool.h>
#include <stdio.h>

#define ADC_NUMS 64           // Number of samples
#define NUM_CHANNEL 5         // Number of channels
#define ADC_DAC_RES 12        // Resolution
extern bool MEAS_data1_ready; // PC1
extern bool MEAS_data2_ready; // PC4/ADC1
extern bool MEAS_data3_ready; // PC3
extern uint32_t MEAS_input_count;
extern bool DAC_active;
extern bool ACC_data_rdy;
extern uint32_t ADC_samples[NUM_CHANNEL * ADC_NUMS];
extern uint16_t batt_sample;

// DOPP defines (for ACD sampling rates)
// #define DOPP_ADC_FS 			60000 // 64kHz for FMCW
#define DOPP_ADC_FS 6000 // 4kHz for DOPP
#define DOPP_TIM_CLOCK 84000000
#define DOPP_TIM_TOP 9
#define DOPP_TIM_PRESCALE (DOPP_TIM_CLOCK / DOPP_ADC_FS / (DOPP_TIM_TOP + 1) - 1)
#define DOPP_ADC_SAMPLES 128 // TODO only sampling this high for firmware test
#define DOPP_ADC_SAMPLES_ARR 128
#define DOPP_ADC_SAMPLES_ZPAD DOPP_ADC_SAMPLES_ARR - DOPP_ADC_SAMPLES
#define DOPP_FREQ_BIN_SIZE DOPP_ADC_FS / DOPP_ADC_SAMPLES_ARR

// defines handling DAC sweep for FMCW
#define DAC_INCREMENT 20
#define DAC_N_SAMPLES (1UL << ADC_DAC_RES) / DAC_INCREMENT
#define DAC_SWEEP_TIME 0.25 // seconds TODO make correct
#define DAC_TIME_PER_SAMPLE DAC_SWEEP_TIME / DAC_N_SAMPLES
#define DAC_SAMPLE_FREQ 1 / DAC_TIME_PER_SAMPLE
#define DAC_TIME_CLOCK 84000000
#define DAC_TIM_TOP 204 * 2
#define DAC_TIM_PRESCALE 0

// FMCW defines (like DOPP, defines sampling rates)
#define FMCW_ADC_FS 64000
#define FMCW_TIM_TOP (635 * 2) / 2
#define FMCW_TIM_PRESCALE 1
#define FMCW_ADC_SAMPLES (32 * 2)
#define FMCW_FREQ_BIN_SIZE FMCW_ADC_FS / FMCW_ADC_SAMPLES

// TODO remove (hacked together to allow for FMCW testing without changing much code)
#define DOPP_ADC_FS FMCW_ADC_FS
#define DOPP_TIM_TOP FMCW_TIM_TOP
#define DOPP_TIM_PRESCALE FMCW_TIM_PRESCALE
#define DOPP_ADC_SAMPLES FMCW_ADC_SAMPLES
#define DOPP_FREQ_BIN_SIZE FMCW_FREQ_BIN_SIZE

// buffers for DOPP ADC samples
extern bool MEAS_DOPP_ready;
extern uint32_t MEAS_DOPP_input_count;
extern uint32_t ADC_DOPP_samples[DOPP_ADC_SAMPLES * 2]; // needs zero padding later

// buffers for FMCW ADC samples
extern bool MEAS_FMCW_ready;
extern uint32_t MEAS_FMCW_input_count;
extern uint32_t ADC_FMCW_samples[FMCW_ADC_SAMPLES * 2];

extern bool DOPP_active;
extern bool FMCW_active;

extern bool use_fake_dopp_data; // TODO for firmware test only

void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(void);
void DAC_reset(void);
void DAC_init(void);
void DAC_increment(void);
void ADC_reset(void);
void reset_data(void);
bool batteryStatus(void);

void ADC_DOPP_scan_init(void);
void ADC_DOPP_scan_start(void);
void DOPP_reset_data(void);

void ADC_FMCW_scan_init(void);
void ACD_FMCW_scan_start(void);
void FMCW_reset_data(void);

void DAC_sweep_init(void);
void DAC_sweep_start(void);

#endif
