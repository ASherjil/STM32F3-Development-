#include "Throttle_Pedal.h"


int main(void)
{
	ADC_init();// PF.2 as input 
	interrupt_init();// button interrupt, PA.0
	CountLEDs_init();// init ALL LEDS(PE.8-PE.15)
	
	ext_interrupt1_init(); // init interrupt on PA.1
	ext_interrupt2_init();// init interrupt on PC.3
	encoder_init(); // initialise the encoder emulator on PB.12 and PB.13
	DAC_init(); // generate triangle wave at 1Hz frequency, PA.4 

	
	while(1)
	{
		writeLEDs(); // run the main function 
	}

}
