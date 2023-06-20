#ifndef MAIN_H_
#define MAIN_H_

#include <stdbool.h>

extern bool buzzer;

/*****************************************************************************
 * Evalboard revision E (blue PCB)
 * has an inverted y-axis in the touch controller compared to the display.
 * @attention
 * Comment this \#define if you are NOT using evalboard revision E (blue PCB).
 *****************************************************************************/
#define EVAL_REV_E

#define FLIPPED_LCD  // when defined, LCD orientation is rotated by 180°

#define REFRESH_RATE 100 // ms per frame

#endif
