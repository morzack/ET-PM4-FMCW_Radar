#ifndef MEAS_H_
#define MEAS_H_

#include <stdbool.h>
#include <stdio.h>

// DAC params are calculated and set to do 1ms sweep
#define ADC_DAC_RES 12 ///< Resolution
#define DAC_INCREMENT 20
#define DAC_N_SAMPLES (1UL << ADC_DAC_RES) / DAC_INCREMENT
#define DAC_SWEEP_TIME 0.25 // seconds TODO make correct
#define DAC_TIME_PER_SAMPLE DAC_SWEEP_TIME / DAC_N_SAMPLES
#define DAC_SAMPLE_FREQ 1 / DAC_TIME_PER_SAMPLE
#define DAC_TIME_CLOCK 84000000
#define DAC_TIM_TOP 408
#define DAC_TIM_PRESCALE 0

// FMCW ACD sampling frequency set to sample 64 times in 1ms (i.e. 64khz)
#define DOPP_ADC_FS 64000
#define DOPP_TIM_TOP 655 // NOTE was 635??
#define DOPP_TIM_PRESCALE 1
#define DOPP_ADC_SAMPLES 64
#define DOPP_FREQ_BIN_SIZE DOPP_ADC_FS / DOPP_ADC_SAMPLES

extern bool MEAS_DOPP_ready;
extern uint32_t ADC_DOPP_samples[DOPP_ADC_SAMPLES * 2]; // *2 to handle interleaving

void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(void);
void DAC_reset(void);
void DAC_init(void);
void DAC_increment(void);
void ADC_reset(void);
bool batteryStatus(void);

void ADC_DOPP_scan_init(void);
void ADC_DOPP_scan_start(void);

void DAC_sweep_start(void);

#endif
