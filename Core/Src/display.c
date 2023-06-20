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

DISPLAY_state_t current_display_mode = MODE_SPLASH;

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

/*
 * displays a splash screen w/ credits or something
 * TODO
 */
void DISPLAY_MODE_SPLASH(void)
{
}

/*
 * display the doppler data + computed peak speed
 */
void DISPLAY_MODE_DOPP(void)
{
    char str[16];

    ADC_DOPP_scan_init();
    ADC_DOPP_scan_start();
    while (!DOPP_MEAS_ready) {;}
    DOPP_MEAS_ready = false;
    DOPP_calc_data();

    MENU_clear_screen();
    MENU_draw_header("DOPP");

    MENU_text[0].pos[0] = 10;
    MENU_text[0].pos[1] = HEADER_HEIGHT + 10;
    MENU_text[0].max_length = 16;
    MENU_text[0].text_size = &Font16;
    MENU_text[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_text[0].background_color = 0xFF000000;
    
    float dopp_peak_freq = DOPP_calc_peak(true);
    float dopp_speed = DOPP_calc_speed(dopp_peak_freq);
    float dopp_speed_km = dopp_speed*3.6f;
    // snprintf(str, 15, "%.1f m/s %.0f km/h", dopp_speed, dopp_speed_km);
    // TODO km/h
    snprintf(str, 15, "%.1f m/s", dopp_speed);
    strcpy(MENU_text[0].text_line, str);

    MENU_draw_text(MENU_text[0], LEFT);
    
    // FFT data
    MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40, 220, 100, &fft_positive_out, DOPP_ADC_SAMPLE_COUNT / 2, 0xFF0000FF, true);
    MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40, 220, 100, &fft_negative_out, DOPP_ADC_SAMPLE_COUNT / 2, 0xFF00FF00, false);

    // draw raw voltage data
    MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40 + 100 + 10, 220, 100, &raw_PC1_stream, DOPP_ADC_SAMPLE_COUNT, 0xFF0000FF, true);
    MENU_draw_graph_ptr(10, HEADER_HEIGHT + 40 + 100 + 10, 220, 100, &raw_PC3_stream, DOPP_ADC_SAMPLE_COUNT, 0xFF00FF00, false);
}

/*
 * display the FMCW data + computed position + distance graph from video demo
 */
void DISPLAY_MODE_FMCW(void)
{
    char str[16];

    FMCW_ADC_scan_init();
    DAC_sweep_start();
    FMCW_ADC_scan_start();
    while (!FMCW_MEAS_ready) {;}
    FMCW_MEAS_ready = false;
    FMCW_calc_data();

    MENU_clear_screen();
    MENU_draw_header("FMCW");

    MENU_text[0].pos[0] = 10;
    MENU_text[0].pos[1] = HEADER_HEIGHT + 10;
    MENU_text[0].max_length = 16;
    MENU_text[0].text_size = &Font16;
    MENU_text[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_text[0].background_color = 0xFF000000;
    
    float fmcw_peak_freq = FMCW_calc_peak();
    float fmcw_dist = FMCW_calc_distance(fmcw_peak_freq);
    int fmcw_block_draw = (int) ((fmcw_dist+1.15f/2.0f)/1.15f) - 1;
    snprintf(str, 15, "%.1f m", fmcw_dist);
    strcpy(MENU_text[0].text_line, str);

    MENU_draw_text(MENU_text[0], LEFT);
    
    // FFT data
    MENU_draw_graph_ptr(FMCW_GRAPH_X, HEADER_HEIGHT + 40, FMCW_GRAPH_WIDTH, 100, &fft_avg_vec_fmcw, FMCW_MAX_BIN + 1, 0xFF0000FF, true);

    // draw raw voltage data
    MENU_draw_graph_ptr(FMCW_GRAPH_X, HEADER_HEIGHT + 40 + 100 + 10, FMCW_GRAPH_WIDTH, 100, &raw_PC5_stream, FMCW_ADC_SAMPLE_COUNT, 0xFF0000FF, true);

    // draw distance graph
    ALERT_draw_blocks(DIST_BLOCK_N - fmcw_block_draw);
}
