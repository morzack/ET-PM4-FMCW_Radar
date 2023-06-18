#ifndef MEAS_H_
#define MEAS_H_

#include <stdbool.h>
#include <stdio.h>

// all timer values are calculated using script at:
// https://github.com/v0idv0id/STM32-Scaler/blob/master/timscale.py
// cmdline format like: python timer_calc.py --freq 204800 --max=5 --error 1
// (204800 is calculated timer frequency needed for 1ms DAC sweep)

#define ADC_RESOLUTION 12

// DAC params are calculated and set to do 1ms sweep
#define DAC_CLOCK_FREQUENCY 84000000
#define DAC_RESOLUTION 12 ///< Resolution
#define DAC_INCREMENT 20
#define DAC_TIM_TOP 408
#define DAC_TIM_PRESCALE 0

// FMCW ACD sampling frequency set to sample 64 times in 1ms (i.e. 64khz)
#define FMCW_ADC_SAMPLING_FREQ 64000
#define FMCW_TIM_TOP 655 // NOTE was 635?? i think that was a typo
#define FMCW_TIM_PRESCALER 1
#define FMCW_ADC_SAMPLE_COUNT 64
#define FMCW_FREQ_BIN_SIZE FMCW_ADC_SAMPLING_FREQ / FMCW_ADC_SAMPLE_COUNT

extern bool FMCW_MEAS_ready;
extern uint32_t FMCW_ADC_samples[FMCW_ADC_SAMPLE_COUNT * 2]; // *2 to handle interleaving for dual mode (that we don't use)

void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(void);
void DAC_reset(void);
void DAC_init(void);
void DAC_increment(void);
void ADC_reset(void);
bool batteryStatus(void);

void FMCW_ADC_scan_init(void);
void FMCW_ADC_scan_start(void);

void DAC_sweep_start(void);

#endif
