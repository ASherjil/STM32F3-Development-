#include "stm32f3xx.h"                  // Device header


void delay(int a); // prototype for delay function

int main(void)
{
	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
	// GPIOE is a structure defined in stm32f303xc.h file
	// Define settings for each output pin using GPIOE structure
	GPIOE->MODER |= 0x55550000; // Set mode of each pin in port E
	GPIOE->OTYPER &= ~(0xFF00); // Set output type for each pin required in Port E
	GPIOE->PUPDR &= ~(0x55550000); // Set Pull up/Pull down resistor configuration for Port E
	
	int mask= 0x100;

	while (1)
	{
		while (mask <= 0xFF00)
		{
			GPIOE->BSRRL = mask;// turn on LED
			delay(555555*2);// wait for 1 second 
			GPIOE->BSRRH = mask; // turn off LED
			mask+=256;
		}
		
		GPIOE->BSRRH = 0xFF00; // turn off all LEDs
		mask = 0x100;// reset mask 
	}
}

// Delay function to occupy processor
void delay (int a)
{
    volatile int i;

    for (i=0 ; i < a ; i++)
    {}
}
