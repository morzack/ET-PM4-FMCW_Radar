/** ***************************************************************************
 * @file
 * @brief See display.c
 *
 * Prefix DISPLAY
 *
 *****************************************************************************/

#ifndef DISPLAY_H_
#define DISPLAY_H_


/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdbool.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

/******************************************************************************
 * Types
 *****************************************************************************/

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
void DISPLAY_battery(void);
void DISPLAY_main(void);
void DISPLAY_voltage(void);
void DISPLAY_voltage_single(void);
void DISPLAY_voltage_accurate(void);
void DISPLAY_data_voltage(void);
void DISPLAY_current(void);
void DISPLAY_current_single(void);
void DISPLAY_current_accurate(void);
void DISPLAY_data_current(void);


#endif
