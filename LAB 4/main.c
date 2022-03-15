#include "Throttle_Pedal.h"
#include "stm32f3xx.h"                  // Device header


int main(void)
{
	DAC_init(); // generate triangle wave at 1Hz frequency, PA.4 
	encoder_generator(); // initialise the encoder emulator on PE.8 and PE.9
	OpAmp_init(); // PA.5 as input, PA.2 as output 
	ADC_init();// PF.2 as input 
	interrupt_init();// button interrupt, PA.0
	CountLEDs_init();// init LED PE11-15 
	ext_interrupt1_init(); // init interrupt on PA.1
	ext_interrupt2_init();// init interrupt on PC.3

	
	while(1)
	{
		writeLEDs();
	}

}
