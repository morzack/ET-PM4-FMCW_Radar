/** ***************************************************************************
 * @file
 * @brief See menu.c
 *
 * Prefix MENU
 *
 *****************************************************************************/

#ifndef MENU_H_
#define MENU_H_


/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdbool.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include "measuring.h"

/******************************************************************************
 * Types
 *****************************************************************************/
/** Enumeration of possible menu items */
typedef enum {
    MENU_MAIN = 0,
    MENU_VOLTAGE,
    MENU_CURRENT,
	MENU_VOLTAGE_SINGLE,
	MENU_VOLTAGE_ACCURATE,
	MENU_CURRENT_SINGLE,
	MENU_CURRENT_ACCURATE,
} MENU_item_t;

/** Enumeration of possible text alignments */
typedef enum {
    LEFT = 0,
    CENTER = 1,
    RIGHT = 2
} MENU_text_align_t;

/** Struct with fields of a menu entry */
typedef struct {
    uint16_t pos[2];            //xy position
    uint16_t size[2];           //xy size
    uint32_t idle_color;
    uint32_t pressed_color;
    uint32_t text_color;
    sFONT* text_size;
    char text_line[12];
    bool state;
    bool pressed;
    bool rise_edge;
    bool fall_edge;
} MENU_entry_t;

/** Struct with fields of a text entry */
typedef struct {
    uint16_t pos[2];
    uint16_t max_length;
    sFONT* text_size;
    uint32_t text_color;
    uint32_t background_color;
    char text_line[20];
} MENU_text_t;
/******************************************************************************
 * Defines
 *****************************************************************************/
extern MENU_item_t MENU_state;
extern MENU_entry_t MENU_entry[10];
extern MENU_text_t MENU_text[10];
extern TS_StateTypeDef TS_State;

#define HEADER_HEIGHT                   50                  ///< Height of header
#define HEADER_WIDTH                    240-HEADER_HEIGHT   ///< Width of header
#define HEADER_TEXT_SIZE                &Font24             ///< Possible font sizes: 8 12 16 20 24
#define HEADER_BACKGROUND_COLOR         0xFFFFFFFF          ///< Header background color in HEX value
#define HEADER_TEXT_COLOR               0xFF000000          ///< Header text color in HEX value

#define MENU_BACKGROUND_COLOR           0xFF000000          ///< Menu background color in HEX value
#define MENU_BUTTON_PRESSED_COLOR       0xFFAEAEAE          ///< Menu button color when pressed in HEX value
#define MENU_BUTTON_IDLE_COLOR          0xFF595959          ///< Menu button color when not pressed in HEX value
#define MENU_BUTTON_TEXT_SIZE           &Font16             ///< Possible font sizes: 8 12 16 20 24
#define MENU_BUTTON_TEXT_COLOR          0xFFFFFFFF          ///< Button text color in HEX value

#define BACK_BUTTON_COLOR_PRESSED       0xFF87CEEB          ///< "BACK" button color when pressed in HEX value
#define BACK_BUTTON_COLOR_IDLE          0xFF00008B

/******************************************************************************
 * Functions
 *****************************************************************************/
void MENU_draw_header(char* header);
void MENU_draw_button(MENU_entry_t button);
void MENU_draw_text(MENU_text_t text, MENU_text_align_t align);
void MENU_draw_graph_grid(uint32_t pos_x, uint32_t pos_y, uint32_t size_x, uint32_t size_y, uint32_t div_x, uint32_t div_y);
void MENU_draw_graph(uint32_t pos_x, uint32_t pos_y, uint32_t size_x, uint32_t size_y, uint32_t samples[_DOPP_ADC_SAMPLES], uint32_t color, bool clear);
void MENU_draw_graph_log(uint32_t pos_x, uint32_t pos_y, uint32_t size_x, uint32_t size_y, uint32_t samples[_DOPP_ADC_SAMPLES], uint32_t color, bool clear);
void MENU_clear_screen(void);
void MENU_update_buttons(TS_StateTypeDef TS_State, uint16_t n);
void MENU_check_transition(void);

#endif
