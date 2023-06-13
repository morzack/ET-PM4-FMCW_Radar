#ifndef PUSHBUTTON_H_
#define PUSHBUTTON_H_

#include <stdbool.h>

void PB_init(void);
void PB_enableIRQ(void);
bool PB_pressed(void);

#endif
