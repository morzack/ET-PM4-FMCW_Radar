/*
 * Functions for directly rendering *things* to the framebuffer
 *
 * Author:	John Kesler	<keslejoh@students.zhaw.ch>
 * 			Linus Leuch	<leuchlin@students.zhaw.ch>
 * 			Simon Meli	<melisim1@students.zhaw.ch>
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

#include "main.h"
#include "render.h"
#include "calculation.h"
#include "measuring.h"

MENU_entry_t MENU_entry[10]; ///< menu entries
MENU_text_t MENU_text[10];   ///< menu text
TS_StateTypeDef TS_State;    ///< State of the touch controller

int dist_block_colors[5] = {DIST_COLOR_CLOSE, DIST_COLOR_MID, DIST_COLOR_FAR, DIST_COLOR_FAR};

/*
 * draws a "header" to the display
 * this is rendered at the top of the screen in a corner
 */
void MENU_draw_header(char *header)
{
    uint16_t length = strlen(header);
    uint16_t text_pos[2];
    uint16_t text_width = (HEADER_TEXT_SIZE)->Width;
    uint16_t text_height = (HEADER_TEXT_SIZE)->Height;

    text_pos[0] = ((HEADER_WIDTH - length * text_width) / 2) + HEADER_HEIGHT; // x pos
    text_pos[1] = (HEADER_HEIGHT - text_height) / 2;                          // y pos

    BSP_LCD_SetTextColor(HEADER_BACKGROUND_COLOR);
    BSP_LCD_FillRect(0, 0, HEADER_WIDTH, HEADER_HEIGHT);
    BSP_LCD_SetBackColor(HEADER_BACKGROUND_COLOR);
    BSP_LCD_SetTextColor(HEADER_TEXT_COLOR);
    BSP_LCD_SetFont(HEADER_TEXT_SIZE);
    BSP_LCD_DisplayStringAt(text_pos[0], text_pos[1], header, LEFT_MODE);
}

/*
 * draw a clickable button to the screen
 * handles rendering button state (ex: pushed)
 * unused for this application
 */
void MENU_draw_button(MENU_entry_t button)
{
    uint32_t backcolor;
    uint16_t text_pos[2];
    uint16_t length = strlen(button.text_line);

    text_pos[0] = (button.pos[0] + button.size[0] / 2 - length * button.text_size->Width / 2);
    text_pos[1] = (button.pos[1] + button.size[1] / 2 - button.text_size->Height / 2);

    if (button.pressed)
    {
        backcolor = button.pressed_color;
    }
    else
    {
        backcolor = button.idle_color;
    }

    BSP_LCD_SetTextColor(backcolor);
    BSP_LCD_FillRect(button.pos[0], button.pos[1], button.size[0], button.size[1]);
    BSP_LCD_SetBackColor(backcolor);
    BSP_LCD_SetTextColor(button.text_color);
    BSP_LCD_SetFont(button.text_size);
    BSP_LCD_DisplayStringAt(text_pos[0], text_pos[1], (uint8_t *)button.text_line, LEFT_MODE);
}

/*
 * draw arbitrary text to the display, defined by a MENU_text_t and alignment
 */
void MENU_draw_text(MENU_text_t text, MENU_text_align_t align)
{
    uint16_t x_pos;

    BSP_LCD_SetFont(text.text_size);

    if (align == LEFT)
    {
        x_pos = text.pos[0];
    }
    else if (align == CENTER)
    {
        x_pos = text.pos[0] + ((text.max_length - strlen(text.text_line)) * text.text_size->Width) / 2;
    }
    else if (align == RIGHT)
    {
        x_pos = text.pos[0] + (text.max_length - strlen(text.text_line)) * text.text_size->Width;
    }
    else
    {
        return;
    }

    BSP_LCD_SetTextColor(text.background_color);
    BSP_LCD_FillRect(text.pos[0], text.pos[1], text.text_size->Width * text.max_length, text.text_size->Height);
    BSP_LCD_SetBackColor(text.background_color);
    BSP_LCD_SetTextColor(text.text_color);
    BSP_LCD_DisplayStringAt(x_pos, text.pos[1], (uint8_t *)text.text_line, LEFT_MODE);
}

/*
 * render a grid to the display (used for graphing)
 * given a top left point (pos_x, pos_y), number of vertical/horiontal lines (div_x, div_y), and pixel size (size_x, size_y)
 */
void MENU_draw_graph_grid(uint32_t pos_x, uint32_t pos_y, uint32_t size_x, uint32_t size_y, uint32_t div_x, uint32_t div_y)
{
    uint16_t line_x;
    uint16_t line_y;
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

    /* Draw horizontal lines */
    for (uint32_t i = 0; i < div_y - 1; i++)
    {
        line_y = pos_y + size_y / div_y * (i + 1);
        BSP_LCD_DrawLine(pos_x, line_y, pos_x + size_x, line_y);
    }

    /* Draw vertical lines */
    for (uint32_t i = 0; i < div_x - 1; i++)
    {
        line_x = pos_x + size_x / div_x * (i + 1);
        BSP_LCD_DrawLine(line_x, pos_y, line_x, pos_y + size_y);
    }
}

/******************************************************************************
 * @brief clears the screen
 *
 *****************************************************************************/

void MENU_clear_screen(void)
{
    BSP_LCD_Clear(MENU_BACKGROUND_COLOR);
}

/******************************************************************************
 * @brief update button state
 * @param State of the touch controller
 * @param number of buttons in menu
 *
 * @note TS_State is updated by calling MENU_check_transition().
 * Call once MENU_check_transition() in the main while loop for polling
 *****************************************************************************/
void MENU_update_buttons(TS_StateTypeDef TS_State, uint16_t n) // n=number of buttons in menu
{
    for (uint16_t i = 0; i < n; i++)
    {
        uint16_t Xmin = MENU_entry[i].pos[0];
        uint16_t Xmax = Xmin + MENU_entry[i].size[0];
        uint16_t Ymin = MENU_entry[i].pos[1];
        uint16_t Ymax = Ymin + MENU_entry[i].size[1];
        MENU_entry[i].state = MENU_entry[i].pressed;
        MENU_entry[i].pressed = 0;
        MENU_entry[i].rise_edge = 0;
        MENU_entry[i].fall_edge = 0;

        if (TS_State.TouchDetected)
        {
            if ((TS_State.X > Xmin && TS_State.X < Xmax) && (TS_State.Y > Ymin && TS_State.Y < Ymax))
            {
                MENU_entry[i].pressed = 1;
                MENU_draw_button(MENU_entry[i]);

                if (MENU_entry[i].state == 0)
                {
                    MENU_entry[i].rise_edge = 1;
                }
            }
        }
        else if (MENU_entry[i].state == 1)
        {
            MENU_draw_button(MENU_entry[i]);
            MENU_entry[i].fall_edge = 1;
        }
    }
}

/** ***************************************************************************
 * @brief Check for selection/transition
 *
 * @note  Evalboard revision E (blue PCB) has an inverted y-axis
 * in the touch controller compared to the display.
 * Uncomment or comment the <b>\#define EVAL_REV_E</b> in main.h accordingly.
 *****************************************************************************/
void MENU_check_transition(void)
{
    BSP_TS_GetState(&TS_State); // Get the state

// Evalboard revision E (blue) has an inverted y-axis in the touch controller
#ifdef EVAL_REV_E
    TS_State.Y = BSP_LCD_GetYSize() - TS_State.Y; // Invert the y-axis
#endif
    // Invert x- and y-axis if LCD is flipped
#ifdef FLIPPED_LCD
    TS_State.X = BSP_LCD_GetXSize() - TS_State.X; // Invert the x-axis
    TS_State.Y = BSP_LCD_GetYSize() - TS_State.Y; // Invert the y-axis
#endif
}

/*
 * draw a graph from a memory buffer to the screen
 * samples is pointer to buffer, length is length of points to grab
 */
void MENU_draw_graph_ptr(uint32_t pos_x, uint32_t pos_y, uint32_t size_x, uint32_t size_y, uint32_t *samples, uint32_t length, uint32_t color, bool clear)
{
    uint32_t data;
    uint32_t data_last;

    if (clear)
    {
        BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
        BSP_LCD_FillRect((pos_x - 1), pos_y, (size_x + 1), size_y);
        MENU_draw_graph_grid(pos_x, pos_y, (size_x - 1), (size_y - 1), 5, 4);
    }
    BSP_LCD_SetTextColor(color);

    size_x--;
    size_y--;

    data = samples[0];
    data = data * size_y / (1 << ADC_RESOLUTION);

    if (data >= size_y)
    {
        data = size_y;
    }

    for (uint32_t i = 1; i < length; i++)
    {
        data_last = data;
        data = samples[i] * size_y / (1 << ADC_RESOLUTION);

        if (data >= size_y)
        {
            data = size_y;
        }

        BSP_LCD_DrawLine((((i - 1) * size_x / (length - 1)) + pos_x), (size_y - data_last + pos_y), (i * size_x / (length - 1) + pos_x), (size_y - data + pos_y));
    }
}

void ALERT_draw_blocks(int n_blocks_filled) {
    for (int n_block=0; n_block < DIST_BLOCK_N; n_block++) {
        int block_x, block_y;
        block_x = DIST_BLOCK_PADDING_X;
        block_y = DIST_BLOCK_PADDING_Y + n_block * (DIST_BLOCK_HEIGHT_Y + DIST_BLOCK_PADDING_INNER) + DIST_BLOCK_TOP_PADDING_Y;

        if ((DIST_BLOCK_N-n_block-1) < n_blocks_filled) {
            BSP_LCD_SetTextColor(dist_block_colors[n_block]);
            BSP_LCD_FillRect(block_x, block_y, DIST_BLOCK_WIDTH_X, DIST_BLOCK_HEIGHT_Y);
        }
    }
}

void ALERT_draw_distance(float distance) {
    char text[10];
	snprintf(text, 9, "Dist: %f", distance);
    MENU_draw_header(text);
}
