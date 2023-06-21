#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdbool.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

typedef enum
{
    MODE_SPLASH = 0,
    MODE_DOPP = 1,
    MODE_FMCW = 2,
} DISPLAY_state_t;

extern DISPLAY_state_t current_display_mode;

extern bool just_changed_mode;

void DISPLAY_battery(void);

void DISPLAY_MODE_SPLASH(void);
void DISPLAY_MODE_DOPP(void);
void DISPLAY_MODE_FMCW(void);

#endif
