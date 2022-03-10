#include "Throttle_Pedal.h"
#include "stm32f3xx.h"                  // Device header

int main(void)
{
	DAC_init(); // generate triangle wave at 1Hz frequency, PA.4 
	encoder_generator(); // initialise the encoder emulator on PE.8 and PE.9
	OpAmp_init();
	ADC_init();
	interrupt_init();
	
	
	while (1)
	{
			ADC1->CR |= 0x4; // enable ADC
			while (!(ADC1->ISR & 0x4)) {}// wait for EOC flag to go high
			GPIOE->BSRRH = (0xF800); // turn OFF Leds
			writeLEDs();
	}
	
	
}
