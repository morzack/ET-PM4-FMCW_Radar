/*
 * Handling of menu state and rendering of modals
 *
 * Author:	John Kesler	<keslejoh@students.zhaw.ch>
 * 			Linus Leuch	<leuchlin@students.zhaw.ch>
 * 			Simon Meli	<melisim1@students.zhaw.ch>
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

#include "display.h"
#include "main.h"
#include "render.h"
#include "calculation.h"
#include "measuring.h"

static bool data_rdy = false;

/** ***************************************************************************
 * @brief Display Battery level
 *
 * @n Approximated percentage of charge is shown
 * @n Charging status is shown
 * @note Displays percentage as symbol
 * @note Text background is green when battery is charging.
 *****************************************************************************/
void DISPLAY_battery(void)
{
    char str[16];
    bool charging;

    ADC2_IN15_single_init();
    ADC2_IN15_single_read();
    charging = batteryStatus();

    MENU_text[0].text_size = &Font16;
    MENU_text[0].pos[0] = 180;
    MENU_text[0].pos[1] = (50 - (MENU_text[0].text_size->Height)) / 2;
    MENU_text[0].max_length = 5;

    if (charging)
    {
        MENU_text[0].background_color = 0xFF7FFF00;
    }
    else
    {
        MENU_text[0].background_color = 0xFFFFFFFF;
    }

    MENU_text[0].text_color = 0xFF000000;
    snprintf(str, 15, "[%ld%%]", batt_lvl);
    strcpy(MENU_text[0].text_line, str);

    MENU_draw_text(MENU_text[0], RIGHT);
}

/** ***************************************************************************
 * @brief Draw the single voltage menu onto the display.
 *
 * Define the single voltage menu entry and text attributes
 * @n These attributes are stored in the variable MENU_entry[] and MENU_text[]
 *****************************************************************************/
void DISPLAY_FFT_diagnosis(void)
{
    uint16_t buttonWidth = 220;
    uint16_t buttonHeight = 50;
    char str[16];

    MENU_clear_screen();
    MENU_draw_header("Dopp Peak");

    MENU_text[0].pos[0] = 10;
    MENU_text[0].pos[1] = HEADER_HEIGHT + 10;
    MENU_text[0].max_length = 7;
    MENU_text[0].text_size = &Font16;
    MENU_text[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_text[0].background_color = 0xFF000000;
    
    // float cfft_peak_freq = FMCW_calc_peak();
    float cfft_peak_freq = CALC_DOPP_cfft_peak(true);
    snprintf(str, 15, "Peak: %.1fHz", cfft_peak_freq);
    
    strcpy(MENU_text[0].text_line, str);
    MENU_draw_text(MENU_text[0], LEFT);

    // MENU_text[1].pos[0] = 10;
    // MENU_text[1].pos[1] = HEADER_HEIGHT + MENU_text[0].text_size->Height + 10;
    // MENU_text[1].max_length = 7;
    // MENU_text[1].text_size = &Font16;
    // MENU_text[1].text_color = MENU_BUTTON_TEXT_COLOR;
    // MENU_text[1].background_color = 0xFF000000;
    // snprintf(str, 15, "Speed: %.1fm/s", CALC_DOPP_cfft_speed(cfft_peak_freq));
    // strcpy(MENU_text[1].text_line, str);

    MENU_text[1].pos[0] = 10;
    MENU_text[1].pos[1] = HEADER_HEIGHT + MENU_text[0].text_size->Height + 10;
    MENU_text[1].max_length = 7;
    MENU_text[1].text_size = &Font16;
    MENU_text[1].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_text[1].background_color = 0xFF000000;
    
    snprintf(str, 15, "vel: %.1fm/s", CALC_DOPP_cfft_speed(cfft_peak_freq));
    // snprintf(str, 15, "dist: %.1fm", FMCW_calc_distance(cfft_peak_freq));
    
    strcpy(MENU_text[1].text_line, str);

    MENU_draw_text(MENU_text[1], LEFT);
    // MENU_draw_text(MENU_text[1], LEFT);

    DISPLAY_graph_FFT_data_DOPP();
}

void DISPLAY_graph_FFT_data_FMCW(void)
{
    char str[16];

    while (!FMCW_MEAS_ready)
    {
        ;
    }
    if (FMCW_MEAS_ready)
    {
        FMCW_MEAS_ready = false;
        // while (!MEAS_FMCW_ready) { ; }
        // if (MEAS_FMCW_ready) {
        //     MEAS_FMCW_ready = false;
        // FMCW_active = false;
        // CALC_DOPP_data();
        FMCW_calc_data();
        // FMCW_calc_data();

        // draw FFT data
        // MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40, 220, 100, &fft_avg_vec_fmcw, FMCW_ADC_SAMPLE_COUNT/2, 0xFF0000FF, true);
        MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40, 220, 100, &fft_avg_vec_fmcw, FMCW_MAX_BIN+1, 0xFF0000FF, true);

        // draw raw voltage data
        MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40 + 100 + 10, 220, 100, &raw_PC5_stream, FMCW_ADC_SAMPLE_COUNT, 0xFF0000FF, true);
    }
}

void DISPLAY_graph_FFT_data_DOPP(void)
{
    char str[16];

    while (!MEAS_DOPP_ready)
    {
        ;
    }
    if (MEAS_DOPP_ready)
    {
        MEAS_DOPP_ready = false;
        CALC_DOPP_data();

        // draw FFT data
        // MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40, 220, 100, &fft_avg_vec_fmcw, FMCW_ADC_SAMPLE_COUNT/2, 0xFF0000FF, true);
        MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40, 220, 100, &fft_positive_out, DOPP_ADC_SAMPLES/2, 0xFF0000FF, true);
        MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40, 220, 100, &fft_negative_out, DOPP_ADC_SAMPLES/2, 0xFF00FF00, false);

        // draw raw voltage data
        MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40 + 100 + 10, 220, 100, &raw_PC1_stream, DOPP_ADC_SAMPLES, 0xFF0000FF, true);
        MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40 + 100 + 10, 220, 100, &raw_PC3_stream, DOPP_ADC_SAMPLES, 0xFF00FF00, false);
    }
}
