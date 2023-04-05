/** ***************************************************************************
 * @file
 * @brief Displaying data on the LCD
 *
 *
 * Functions for displaying data of different measurements
 * ==============================================================
 *
 * -Battery level display
 * -Main menu display
 * -function for single Pad measurement
 * -function for accurate Pad measurement
 * -function for single Coil measurement
 * -function for accurate Coil measurement
 * -function with graphs and sample values
 *
 *
 * @author Linus Leuch, leuchlin@students.zhaw.ch,
 * @n Simon Meli, melisim1@students.zhaw.ch
 * @date 23.12.2022
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

#include "display.h"
#include "main.h"
#include "menu.h"
#include "calculation.h"
#include "measuring.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
* Variables
*****************************************************************************/
static bool data_rdy = false;

/******************************************************************************
 * Functions
 *****************************************************************************/



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

    if (ACC_data_rdy)
    {
        ACC_data_rdy = false;
        CALC_battery_level(batt_sample);
    }
    MENU_text[0].text_size = &Font16;
    MENU_text[0].pos[0] = 180;
    MENU_text[0].pos[1] = (50 - (MENU_text[0].text_size->Height)) /2;
    MENU_text[0].max_length = 5;
    
    if(charging) {
        MENU_text[0].background_color = 0xFF7FFF00;
        
    } else {
        MENU_text[0].background_color = 0xFFFFFFFF;
    }

    MENU_text[0].text_color = 0xFF000000;
    snprintf(str, 15, "[%ld%%]", batt_lvl);
    strcpy(MENU_text[0].text_line, str);

    MENU_draw_text(MENU_text[0], RIGHT);
}

/** ***************************************************************************
 * @brief Draw the main menu onto the display.
 *
 * Define the main menu entry and text attributes
 * @n These attributes are stored in the variable MENU_entry[] and MENU_text[]
 *****************************************************************************/
void DISPLAY_main(void)
{
    uint16_t buttonWidth = 120;
    uint16_t buttonHeight = 50;
    uint16_t smallButtonWidth = 70;
    uint16_t n = 4;                 //Number of Buttons (back button not included)

    MENU_clear_screen();

    char str[16];
    strcpy(str, "HOME");
    uint16_t length = strlen(str);
    uint16_t text_pos[2];
	uint16_t text_width = (HEADER_TEXT_SIZE)->Width;
	uint16_t text_height = (HEADER_TEXT_SIZE)->Height;

	text_pos[0]= (240 - length * text_width) / 2;  //x pos
	text_pos[1]= (HEADER_HEIGHT - text_height) / 2;    //y pos

	BSP_LCD_SetTextColor(HEADER_BACKGROUND_COLOR);
	BSP_LCD_FillRect(0, 0, 240 , HEADER_HEIGHT);
	BSP_LCD_SetBackColor(HEADER_BACKGROUND_COLOR);
	BSP_LCD_SetTextColor(HEADER_TEXT_COLOR);
	BSP_LCD_SetFont(HEADER_TEXT_SIZE);
	BSP_LCD_DisplayStringAt(60 , text_pos[1], str, LEFT_MODE);


	MENU_text[1].pos[0] = 30;
	MENU_text[1].pos[1] = HEADER_HEIGHT + 10;
	MENU_text[1].max_length = 13;
	MENU_text[1].text_size = &Font20;
	MENU_text[1].text_color = MENU_BUTTON_TEXT_COLOR;
	MENU_text[1].background_color = 0xFF000000;
	snprintf(str, 15, "Cable Monitor");
	strcpy(MENU_text[1].text_line, str);
	MENU_draw_text(MENU_text[1], LEFT);


	MENU_text[2] = MENU_text[1];
	MENU_text[2].pos[1] += MENU_text[1].text_size->Height;
	MENU_text[2].max_length = 14;
	MENU_text[2].text_size = &Font12;
	snprintf(str, 15, "by Linus Leuch");
	strcpy(MENU_text[2].text_line, str);
	MENU_draw_text(MENU_text[2], LEFT);

	MENU_text[3] = MENU_text[2];
	MENU_text[3].pos[0] += MENU_text[2].max_length * MENU_text[2].text_size->Width;
	MENU_text[3].max_length = 13;
	snprintf(str, 15, " & Simon Meli");
	strcpy(MENU_text[3].text_line, str);
	MENU_draw_text(MENU_text[3], LEFT);


	MENU_text[4].pos[0] = 5;
	MENU_text[4].pos[1] = 160;
	MENU_text[4].max_length = 7;
	MENU_text[4].text_size = &Font16;
	MENU_text[4].text_color = MENU_BUTTON_TEXT_COLOR;
	MENU_text[4].background_color = 0xFF000000;
	snprintf(str, 15, "Select");
	strcpy(MENU_text[4].text_line, str);
	MENU_draw_text(MENU_text[4], LEFT);

	MENU_text[5] = MENU_text[4];
	MENU_text[5].pos[0] += (MENU_text[4].max_length * MENU_text[4].text_size->Width);
	MENU_text[5].max_length = 10;
	snprintf(str, 15, "measuring ");
	strcpy(MENU_text[5].text_line, str);
	MENU_draw_text(MENU_text[5], LEFT);

	MENU_text[6] = MENU_text[5];
	MENU_text[6].pos[0] += MENU_text[5].max_length * MENU_text[4].text_size->Width;
	MENU_text[6].max_length = 4;
	snprintf(str, 15, "mode");
	strcpy(MENU_text[6].text_line, str);
	MENU_draw_text(MENU_text[6], LEFT);

    strcpy(MENU_entry[0].text_line, "OFF");
    strcpy(MENU_entry[1].text_line, "VOLTAGE");
    strcpy(MENU_entry[2].text_line, "CURRENT");
    strcpy(MENU_entry[3].text_line, "SOUND");
    

    MENU_entry[0].pos[0] = 15;
    MENU_entry[0].pos[1] = 190;
    MENU_entry[0].size[0] = smallButtonWidth;
    MENU_entry[0].size[1] = buttonHeight;
    MENU_entry[0].idle_color = BACK_BUTTON_COLOR_IDLE;
    MENU_entry[0].pressed_color = BACK_BUTTON_COLOR_PRESSED;
    MENU_entry[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[0].text_size = MENU_BUTTON_TEXT_SIZE;

    MENU_entry[1].pos[0] = smallButtonWidth + 35;
    MENU_entry[1].pos[1] = 190;
    MENU_entry[1].size[0] = buttonWidth;
    MENU_entry[1].size[1] = buttonHeight;
    MENU_entry[1].idle_color = MENU_BUTTON_IDLE_COLOR;
    MENU_entry[1].pressed_color = MENU_BUTTON_PRESSED_COLOR;
    MENU_entry[1].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[1].text_size = MENU_BUTTON_TEXT_SIZE;

    MENU_entry[2].pos[0] = smallButtonWidth + 35;
    MENU_entry[2].pos[1] = 250;
    MENU_entry[2].size[0] = buttonWidth;
    MENU_entry[2].size[1] = buttonHeight;
    MENU_entry[2].idle_color = MENU_BUTTON_IDLE_COLOR;
    MENU_entry[2].pressed_color = MENU_BUTTON_PRESSED_COLOR;
    MENU_entry[2].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[2].text_size = MENU_BUTTON_TEXT_SIZE;

    MENU_entry[3].pos[0] = 15;
    MENU_entry[3].pos[1] = 250;
    MENU_entry[3].size[0] = smallButtonWidth;
    MENU_entry[3].size[1] = buttonHeight;
    if (buzzer) {
        MENU_entry[3].text_color = 0xFF000000;
        MENU_entry[3].idle_color =0xFF7FFF00;
        MENU_entry[3].pressed_color = 0xFF98FB98;
    } else {
        MENU_entry[3].text_color = 0xFFFFFFFF;
        MENU_entry[3].idle_color = 0xFFFF0000;
        MENU_entry[3].pressed_color = 0xFFFFB3B3;
    }
    MENU_entry[3].text_size = MENU_BUTTON_TEXT_SIZE;

    for(uint8_t i=0; i < n; i++) {

        MENU_draw_button(MENU_entry[i]);
    }

    MENU_state = MENU_MAIN;
}


/** ***************************************************************************
 * @brief Draw the voltage menu onto the display.
 *
 * Define the voltage menu entry attributes
 * @n These attributes are stored in the variable MENU_entry[]
 *****************************************************************************/
void DISPLAY_voltage(void)
{
    uint16_t buttonWidth = 120;
    uint16_t buttonHeight = 50;
    uint16_t n = 2;                 //Number of Buttons (back button not included)
    
    data_rdy = false;
    count = 0;

    MENU_clear_screen();
    MENU_draw_header("VOLTAGE");

    MENU_entry[0].pos[0] = 0;
    MENU_entry[0].pos[1] = 0;
    MENU_entry[0].size[0] = HEADER_HEIGHT;
    MENU_entry[0].size[1] = HEADER_HEIGHT;
    MENU_entry[0].idle_color = BACK_BUTTON_COLOR_IDLE;
    MENU_entry[0].pressed_color = BACK_BUTTON_COLOR_PRESSED;
    MENU_entry[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[0].text_size = MENU_BUTTON_TEXT_SIZE;
    strcpy(MENU_entry[0].text_line, "BACK");
    
    MENU_draw_button(MENU_entry[0]);
    
    strcpy(MENU_entry[1].text_line, "SINGLE");
    strcpy(MENU_entry[2].text_line, "ACCURATE");
    for (uint8_t i=1; i <= n; i++) {
        MENU_entry[i].pos[0] = (240 - buttonWidth) / 2;
        MENU_entry[i].pos[1] = HEADER_HEIGHT + ((320 - HEADER_HEIGHT) * i) / (n + 1) - buttonHeight / 2;
        MENU_entry[i].size[0] = buttonWidth;
        MENU_entry[i].size[1] = buttonHeight;
        MENU_entry[i].idle_color = MENU_BUTTON_IDLE_COLOR;
        MENU_entry[i].pressed_color = MENU_BUTTON_PRESSED_COLOR;
        MENU_entry[i].text_color = MENU_BUTTON_TEXT_COLOR;
        MENU_entry[i].text_size = MENU_BUTTON_TEXT_SIZE;

        MENU_draw_button(MENU_entry[i]);
    }

    MENU_state = MENU_VOLTAGE;
}


/** ***************************************************************************
 * @brief Draw the single voltage menu onto the display.
 *
 * Define the single voltage menu entry and text attributes
 * @n These attributes are stored in the variable MENU_entry[] and MENU_text[]
 *****************************************************************************/
void DISPLAY_voltage_single(void)
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
	snprintf(str, 15, "Peak: %dHz", CALC_DOPP_cfft_peak());
	strcpy(MENU_text[0].text_line, str);
	MENU_draw_text(MENU_text[0], LEFT);

    DISPLAY_data_voltage();

    MENU_state = MENU_VOLTAGE_SINGLE;
}

/** ***************************************************************************
 * @brief Draw the accurate voltage menu onto the display.
 *
 * Define the accurate voltage menu entry and text attributes
 * @n These attributes are stored in the variable MENU_entry[] and MENU_text[]
 *****************************************************************************/
void DISPLAY_voltage_accurate(void)
{
    uint16_t buttonWidth = 220;
    uint16_t buttonHeight = 50;
    char str[16];

    MENU_clear_screen();
    MENU_draw_header("V accurate");

    MENU_entry[0].pos[0] = 0;
    MENU_entry[0].pos[1] = 0;
    MENU_entry[0].size[0] = HEADER_HEIGHT;
    MENU_entry[0].size[1] = HEADER_HEIGHT;
    MENU_entry[0].idle_color = BACK_BUTTON_COLOR_IDLE;
    MENU_entry[0].pressed_color = BACK_BUTTON_COLOR_PRESSED;
    MENU_entry[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[0].text_size = MENU_BUTTON_TEXT_SIZE;
    strcpy(MENU_entry[0].text_line, "BACK");
    MENU_draw_button(MENU_entry[0]);

    MENU_entry[1].pos[0] = 10;
    MENU_entry[1].pos[1] = 255;
    MENU_entry[1].size[0] = buttonWidth;
    MENU_entry[1].size[1] = buttonHeight;
    MENU_entry[1].idle_color = MENU_BUTTON_IDLE_COLOR;
    MENU_entry[1].pressed_color = MENU_BUTTON_PRESSED_COLOR;
    MENU_entry[1].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[1].text_size = MENU_BUTTON_TEXT_SIZE;
    strcpy(MENU_entry[1].text_line, "MEASURE");
    MENU_draw_button(MENU_entry[1]);

    MENU_text[0].pos[0] = 10;
    MENU_text[0].pos[1] = HEADER_HEIGHT + 10;
    MENU_text[0].max_length = 7;
    MENU_text[0].text_size = &Font16;
    MENU_text[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_text[0].background_color = 0xFF000000;
    snprintf(str, 15, "Cable: ");
    strcpy(MENU_text[0].text_line, str);
    MENU_draw_text(MENU_text[0], LEFT);

    MENU_text[1] = MENU_text[0];
    MENU_text[1].pos[0] = MENU_text[0].max_length * MENU_text[0].text_size->Width;
    MENU_text[1].max_length = 13;
    snprintf(str, 15, "NULL");
    strcpy(MENU_text[1].text_line, str);
    MENU_draw_text(MENU_text[1], LEFT);

    MENU_text[2] = MENU_text[0];
    MENU_text[2].pos[1] += MENU_text[0].text_size->Height;
    MENU_text[2].max_length = 10;
    snprintf(str, 15, "Distance: ");
    strcpy(MENU_text[2].text_line, str);
    MENU_draw_text(MENU_text[2], LEFT);

    MENU_text[3] = MENU_text[1];
    MENU_text[3].pos[0] = MENU_text[2].max_length * MENU_text[0].text_size->Width;
    MENU_text[3].pos[1] += MENU_text[0].text_size->Height;
    MENU_text[3].max_length = 7;
    snprintf(str, 15, "%d mm", 0);
    strcpy(MENU_text[3].text_line, str);
    MENU_draw_text(MENU_text[3], LEFT);

    MENU_text[4] = MENU_text[2];
    MENU_text[4].pos[1] += MENU_text[0].text_size->Height;
    MENU_text[4].max_length = 7;
    snprintf(str, 15, "Angle: ");
    strcpy(MENU_text[4].text_line, str);
    MENU_draw_text(MENU_text[4], LEFT);

    MENU_text[5] = MENU_text[3];
    MENU_text[5].pos[0] = MENU_text[4].max_length * MENU_text[0].text_size->Width;
    MENU_text[5].pos[1] += MENU_text[0].text_size->Height;
    MENU_text[5].max_length = 7;
    snprintf(str, 15, "%ld deg", 0);
    strcpy(MENU_text[5].text_line, str);
    MENU_draw_text(MENU_text[5], LEFT);

    MENU_state = MENU_VOLTAGE_ACCURATE;
}

const uint32_t channel_test[ADC_NUMS] = {0,1,2,5,10,20,50,100,200,500,1000,2000,5000,0,1,2,5,10,20,50,100,200,500,1000,2000,5000,0,1,2,5,10,20,50,100,200,500,1000,2000,5000,0};


/** ***************************************************************************
 * @brief Start Pad Measurement and display data
 *
 * @n Process measured data
 * @n A graph of all four capacitive Pads is displayed
 * @n Distance in mm and Angle in deg are displayed as numbers
 * @n Additionally a text is displayed if a cable is present or not.
 *****************************************************************************/
void DISPLAY_data_voltage(void)
{
	char str[16];

//    ADC1_IN5_IN13_scan_init();
//    ADC1_IN5_IN13_scan_start();
//    ADC3_IN4_IN11_IN13_scan_init();
//    ADC3_IN4_IN11_IN13_scan_start();
	ADC_DOPP_scan_init();
	ADC_DOPP_scan_start();

    if (MEAS_DOPP_ready) {
//        copy_data();
//        MEAS_data1_ready = false;
//        MEAS_data3_ready = false;
        MEAS_DOPP_ready = false;
//        CALC_pad_measurement();
        CALC_DOPP_data();

        // if(single || (!single && count == 20) ||(!single && data_rdy)) {
        // 	count = 0;
            // data_rdy = true;

//        	if(PAD.distance == 9999){
//        		MENU_text[1].text_color = 0xFF00FF00;
//        		snprintf(str, 15, "not detected");
//        		strcpy(MENU_text[1].text_line, str);
//
//        		snprintf(str, 15, ">200 mm");
//        		strcpy(MENU_text[3].text_line, str);
//        	} else {
//        		MENU_text[1].text_color = 0xFFFF0000;
//        		snprintf(str, 15, "detected");
//				strcpy(MENU_text[1].text_line, str);
//
//				snprintf(str, 15, "%lu mm", PAD.distance);
//				strcpy(MENU_text[3].text_line, str);
//        	}


//			snprintf(str, 15, "%ld deg", PAD.angle);
//			strcpy(MENU_text[5].text_line, str);
//
//			MENU_draw_text(MENU_text[1], LEFT);
//			MENU_draw_text(MENU_text[3], LEFT);
//			MENU_draw_text(MENU_text[5], LEFT);

			// TODO hacky fft poc, move later
//			arm_cfft_init_f32();

//			MENU_draw_graph(10, HEADER_HEIGHT+70, 220, 130, channel1, 0xFF0000FF, true);

        MENU_draw_graph_log(10, HEADER_HEIGHT + 70, 220, 130, channel1, 0xFF0000FF, true);
//			MENU_draw_graph(10, HEADER_HEIGHT + 70, 220, 130, channel2, 0xFFFF0000, false);
//			MENU_draw_graph(10, HEADER_HEIGHT + 70, 220, 130, channel3, 0xFF00FF00, false);
//			MENU_draw_graph(10, HEADER_HEIGHT + 70, 220, 130, channel4, 0xFFFFA500, false);
    }
}


/** ***************************************************************************
 * @brief Draw the Current menu onto the display.
 *
 * Define the current menu entry attributes
 * @n These attributes are stored in the variable MENU_entry[]
 *****************************************************************************/
void DISPLAY_current(void)
{
    uint16_t buttonWidth = 120;
    uint16_t buttonHeight = 50;
    uint16_t n = 2;

    MENU_clear_screen();
    MENU_draw_header("CURRENT");
    
    MENU_entry[0].pos[0] = 0;
    MENU_entry[0].pos[1] = 0;
    MENU_entry[0].size[0] = HEADER_HEIGHT;
    MENU_entry[0].size[1] = HEADER_HEIGHT;
    MENU_entry[0].idle_color = BACK_BUTTON_COLOR_IDLE;
    MENU_entry[0].pressed_color = BACK_BUTTON_COLOR_PRESSED;
    MENU_entry[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[0].text_size = MENU_BUTTON_TEXT_SIZE;
    strcpy(MENU_entry[0].text_line, "BACK");

    MENU_draw_button(MENU_entry[0]);

    strcpy(MENU_entry[1].text_line, "SINGLE");
    strcpy(MENU_entry[2].text_line, "ACCURATE");
    for (uint8_t i=1; i <= n; i++) {
        MENU_entry[i].pos[0] = (240 - buttonWidth) / 2;
        MENU_entry[i].pos[1] = HEADER_HEIGHT + ((320 - HEADER_HEIGHT) * i) / (n + 1) - buttonHeight / 2;
        MENU_entry[i].size[0] = buttonWidth;
        MENU_entry[i].size[1] = buttonHeight;
        MENU_entry[i].idle_color = MENU_BUTTON_IDLE_COLOR;
        MENU_entry[i].pressed_color = MENU_BUTTON_PRESSED_COLOR;
        MENU_entry[i].text_color = MENU_BUTTON_TEXT_COLOR;
        MENU_entry[i].text_size = MENU_BUTTON_TEXT_SIZE;

        MENU_draw_button(MENU_entry[i]);
    }
    MENU_state = MENU_CURRENT;
}

/** ***************************************************************************
 * @brief Draw the single current menu onto the display.
 *
 * Define the single voltage menu entry and text attributes
 * @n These attributes are stored in the variable MENU_entry[] and MENU_text[]
 *****************************************************************************/
void DISPLAY_current_single(void)
{
    uint16_t buttonWidth = 220;
    uint16_t buttonHeight = 50;
    char str[16];

    MENU_clear_screen();
    MENU_draw_header("A single");

    MENU_entry[0].pos[0] = 0;
    MENU_entry[0].pos[1] = 0;
    MENU_entry[0].size[0] = HEADER_HEIGHT;
    MENU_entry[0].size[1] = HEADER_HEIGHT;
    MENU_entry[0].idle_color = BACK_BUTTON_COLOR_IDLE;
    MENU_entry[0].pressed_color = BACK_BUTTON_COLOR_PRESSED;
    MENU_entry[0].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[0].text_size = MENU_BUTTON_TEXT_SIZE;
    strcpy(MENU_entry[0].text_line, "BACK");
    MENU_draw_button(MENU_entry[0]);

    MENU_entry[1].pos[0] = 10;
    MENU_entry[1].pos[1] = 255;
    MENU_entry[1].size[0] = buttonWidth;
    MENU_entry[1].size[1] = buttonHeight;
    MENU_entry[1].idle_color = MENU_BUTTON_IDLE_COLOR;
    MENU_entry[1].pressed_color = MENU_BUTTON_PRESSED_COLOR;
    MENU_entry[1].text_color = MENU_BUTTON_TEXT_COLOR;
    MENU_entry[1].text_size = MENU_BUTTON_TEXT_SIZE;
    strcpy(MENU_entry[1].text_line, "MEASURE");
    MENU_draw_button(MENU_entry[1]);

    MENU_text[0].pos[0] = 10;
	MENU_text[0].pos[1] = HEADER_HEIGHT + 10;
	MENU_text[0].max_length = 9;
	MENU_text[0].text_size = &Font16;
	MENU_text[0].text_color = MENU_BUTTON_TEXT_COLOR;
	MENU_text[0].background_color = 0xFF000000;
	snprintf(str, 15, "Current: ");
	strcpy(MENU_text[0].text_line, str);
	MENU_draw_text(MENU_text[0], LEFT);

	MENU_text[1] = MENU_text[0];
	MENU_text[1].pos[0] = MENU_text[0].max_length * MENU_text[0].text_size->Width;
	MENU_text[1].max_length = 8;
	snprintf(str, 15, "%d A", 0);
	strcpy(MENU_text[1].text_line, str);
	MENU_draw_text(MENU_text[1], LEFT);

    MENU_state = MENU_CURRENT_SINGLE;
}

/** ***************************************************************************
 * @brief Draw the accurate current menu onto the display.
 *
 * Define the accurate current menu entry and text attributes
 * @n These attributes are stored in the variable MENU_entry[] and MENU_text[]
 *****************************************************************************/
void DISPLAY_current_accurate(void)
{
	uint16_t buttonWidth = 220;
	uint16_t buttonHeight = 50;
	char str[16];

	MENU_clear_screen();
	MENU_draw_header("A accurate");

	MENU_entry[0].pos[0] = 0;
	MENU_entry[0].pos[1] = 0;
	MENU_entry[0].size[0] = HEADER_HEIGHT;
	MENU_entry[0].size[1] = HEADER_HEIGHT;
	MENU_entry[0].idle_color = BACK_BUTTON_COLOR_IDLE;
	MENU_entry[0].pressed_color = BACK_BUTTON_COLOR_PRESSED;
	MENU_entry[0].text_color = MENU_BUTTON_TEXT_COLOR;
	MENU_entry[0].text_size = MENU_BUTTON_TEXT_SIZE;
	strcpy(MENU_entry[0].text_line, "BACK");
	MENU_draw_button(MENU_entry[0]);

	MENU_entry[1].pos[0] = 10;
	MENU_entry[1].pos[1] = 255;
	MENU_entry[1].size[0] = buttonWidth;
	MENU_entry[1].size[1] = buttonHeight;
	MENU_entry[1].idle_color = MENU_BUTTON_IDLE_COLOR;
	MENU_entry[1].pressed_color = MENU_BUTTON_PRESSED_COLOR;
	MENU_entry[1].text_color = MENU_BUTTON_TEXT_COLOR;
	MENU_entry[1].text_size = MENU_BUTTON_TEXT_SIZE;
	strcpy(MENU_entry[1].text_line, "MEASURE");
	MENU_draw_button(MENU_entry[1]);

	MENU_text[0].pos[0] = 10;
	MENU_text[0].pos[1] = HEADER_HEIGHT + 10;
	MENU_text[0].max_length = 9;
	MENU_text[0].text_size = &Font16;
	MENU_text[0].text_color = MENU_BUTTON_TEXT_COLOR;
	MENU_text[0].background_color = 0xFF000000;
	snprintf(str, 15, "Current: ");
	strcpy(MENU_text[0].text_line, str);
	MENU_draw_text(MENU_text[0], LEFT);

	MENU_text[1] = MENU_text[0];
	MENU_text[1].pos[0] = MENU_text[0].max_length * MENU_text[0].text_size->Width;
	MENU_text[1].max_length = 8;
	snprintf(str, 15, "%d A", 0);
	strcpy(MENU_text[1].text_line, str);
	MENU_draw_text(MENU_text[1], LEFT);

	MENU_state = MENU_CURRENT_ACCURATE;
}

/** ***************************************************************************
 * @brief Start coil Measurement and display data
 *
 * @n Process measured data
 * @n A graph of the coil is displayed
 * @n The current is displayed as number
 *****************************************************************************/
void DISPLAY_data_current(void)
{
	char str[16];

    ADC1_IN5_IN13_scan_init();
    ADC1_IN5_IN13_scan_start();

    if (MEAS_data1_ready) {
        copy_data();
        MEAS_data1_ready = false;
        CALC_coil_measurement();

        if(single || (!single && count == 20) ||(!single && data_rdy)) {
        	count = 0;
        	data_rdy = true;

        	snprintf(str, 15, "%lu A", COIL.current);
			strcpy(MENU_text[1].text_line, str);

			MENU_draw_text(MENU_text[1], LEFT);

			MENU_draw_graph(10, HEADER_HEIGHT + 70, 220, 130, channel5, 0xFFFFFF00, true);
        }
    }
}
