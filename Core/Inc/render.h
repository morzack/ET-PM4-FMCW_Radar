#ifndef MENU_H_
#define MENU_H_

#include <stdbool.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include "measuring.h"

#define HEADER_HEIGHT 50                   ///< Height of header
#define HEADER_WIDTH 240 ///< Width of header
#define HEADER_TEXT_SIZE &Font24           ///< Possible font sizes: 8 12 16 20 24
#define HEADER_BACKGROUND_COLOR 0xFFFFFFFF ///< Header background color in HEX value
#define HEADER_TEXT_COLOR 0xFF000000       ///< Header text color in HEX value

#define MENU_BACKGROUND_COLOR 0xFF000000     ///< Menu background color in HEX value
#define MENU_BUTTON_PRESSED_COLOR 0xFFAEAEAE ///< Menu button color when pressed in HEX value
#define MENU_BUTTON_IDLE_COLOR 0xFF595959    ///< Menu button color when not pressed in HEX value
#define MENU_BUTTON_TEXT_SIZE &Font16        ///< Possible font sizes: 8 12 16 20 24
#define MENU_BUTTON_TEXT_COLOR 0xFFFFFFFF    ///< Button text color in HEX value

#define BACK_BUTTON_COLOR_PRESSED 0xFF87CEEB ///< "BACK" button color when pressed in HEX value
#define BACK_BUTTON_COLOR_IDLE 0xFF00008B

#define DIST_COLOR_FAR                  0xFFFFFFFF
#define DIST_COLOR_MID                  0xFFFCCF49
#define DIST_COLOR_CLOSE                0xFFFC4949

#define SCREEN_WIDTH                    240
#define SCREEN_HEIGHT                   320

#define DIST_BLOCK_N                    4
#define DIST_BLOCK_PADDING_X            30
#define DIST_BLOCK_PADDING_Y            20
#define DIST_BLOCK_PADDING_INNER        15
#define DIST_BLOCK_TOP_PADDING_Y        60

#define DIST_BLOCK_WIDTH_X              (SCREEN_WIDTH/2-DIST_BLOCK_PADDING_X*2)
#define DIST_BLOCK_HEIGHT_Y             (SCREEN_HEIGHT-DIST_BLOCK_PADDING_Y*2-DIST_BLOCK_TOP_PADDING_Y)/DIST_BLOCK_N-DIST_BLOCK_PADDING_INNER

#define FMCW_GRAPH_X                    (SCREEN_WIDTH/2)
#define FMCW_GRAPH_WIDTH                DIST_BLOCK_WIDTH_X+DIST_BLOCK_PADDING_X

#define LOG_GRAPH_SCALING 10

/** Enumeration of possible text alignments */
typedef enum
{
    LEFT = 0,
    CENTER = 1,
    RIGHT = 2
} MENU_text_align_t;

/** Struct with fields of a menu entry */
typedef struct
{
    uint16_t pos[2];  // xy position
    uint16_t size[2]; // xy size
    uint32_t idle_color;
    uint32_t pressed_color;
    uint32_t text_color;
    sFONT *text_size;
    char text_line[12];
    bool state;
    bool pressed;
    bool rise_edge;
    bool fall_edge;
} MENU_entry_t;

/** Struct with fields of a text entry */
typedef struct
{
    uint16_t pos[2];
    uint16_t max_length;
    sFONT *text_size;
    uint32_t text_color;
    uint32_t background_color;
    char text_line[20];
} MENU_text_t;

extern MENU_entry_t MENU_entry[10];
extern MENU_text_t MENU_text[10];
extern TS_StateTypeDef TS_State;

void MENU_draw_header(char *header);
void MENU_draw_button(MENU_entry_t button);
void MENU_draw_text(MENU_text_t text, MENU_text_align_t align);

void MENU_draw_graph_grid(uint32_t pos_x, uint32_t pos_y, uint32_t size_x, uint32_t size_y, uint32_t div_x, uint32_t div_y);

void MENU_draw_graph_ptr(uint32_t pos_x, uint32_t pos_y, uint32_t size_x, uint32_t size_y, uint32_t *samples, uint32_t length, uint32_t color, bool clear);

void MENU_clear_screen(void);
void MENU_update_buttons(TS_StateTypeDef TS_State, uint16_t n);
void MENU_check_transition(void);

void ALERT_draw_blocks(int n_blocks_filled);
void ALERT_draw_distance(float distance);

#endif
