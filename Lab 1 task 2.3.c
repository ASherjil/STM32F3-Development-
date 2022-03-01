#include "stm32f3xx.h"                  // Device header

void delay(int a); // prototype for delay function

int main(void)
{
// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
// GPIOE is a structure defined in stm32f303xc.h file
	
//-----------------ALL LED PINS SET TO PUSH/PULL WITH NO No pull-up, pull-down
	GPIOE->MODER = (GPIOE->MODER & ~(0xFFFF0000))| 0x55550000;// output mode "01" for all pins
	GPIOE->OTYPER &= ~(0xFF00); // push/pull "00" for all pins
	GPIOE->PUPDR &= ~(0xFFFF0000); // no pullup, pull-down for all pins
//--------------------------------------------------------------------------------	
	
	int count= 1;// = 0b00000001

	while (1)
	{
		while (count < (255+1)) // run max value + 1 times, (2^8)-1 +1= 255+1  
		{
			GPIOE->BSRRL = (count<<8);// turn on LED by shifting bits to the left
			delay(555555*2);// wait for 1 second 
			GPIOE->BSRRH = (count<<8); // turn off LED
			++count; // increment 
		}
		GPIOE->BSRRH = 0xFF00; // turn off all LEDs
		count = 1;// reset mask 
	}
}

// Delay function to occupy processor
void delay (int a)
{
    volatile int i;

    for (i=0 ; i < a ; i++)
    {}
}
