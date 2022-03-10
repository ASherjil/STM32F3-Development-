#include "Throttle_Pedal.h"


int main(void)
{
	DAC_init(); // generate triangle wave at 1Hz frequency, PA.4 
	encoder_generator(); // initialise the encoder emulator on PE.8 and PE.9
}

