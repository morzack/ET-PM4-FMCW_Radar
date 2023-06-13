#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdbool.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

void DISPLAY_battery(void);
void DISPLAY_FFT_diagnosis(void);
void DISPLAY_graph_FFT_data(void);

#endif
