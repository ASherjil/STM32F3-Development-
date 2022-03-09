#ifndef THROTTLE_PEDAL_H
#define THROTTLE_PEDAL_H

#include "stm32f3xx.h"                  // Device header
#include <stdlib.h>

#define PRESCALER1 255
#define ARR1 121  // for 255 Hz frequency
#define PERIOD1 255
#define MAX_AMPLITUDE 127

static int count = 0;
void DAC_init(void); // initialise the DAC with a timer-based interrupt
void triangle_emulator(void); // write DAC values which output an analogue triangular signal 


#endif
