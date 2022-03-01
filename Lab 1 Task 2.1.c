#include "stm32f3xx.h"                  // Device header

void delay(int a); // prototype for delay function

int main()
{
	
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;// Enable clock on GPIO port E
	
	GPIOE->MODER = (GPIOE->MODER & ~0x3030000)|(0x1010000); //PE.8 and PE.12 set to output mode "01"

	GPIOE->OTYPER |= 0x100; // PE.8 set to open drain 
	GPIOE->OTYPER &= ~0x1000; // PE.12 set to push/pull
	
//-------------Set PE.8 to Pull-Up and set PE.12 to No pull-up,pull-down
	GPIOE->PUPDR = (GPIOE->PUPDR& ~0x3030000)|0x10000;
//----------------------------------------------------------------------

	while (1)
	{
			GPIOE->BSRRL = 0x1100 ;// turn on LED on PE.8,PE.12
			delay(555555*2);// wait for 1 second 
			GPIOE->BSRRH = 0x1100 ; // turn off LED on PE.8,PE.12
			delay(555555*2);// wait for 1 second 
	}
}
// Delay function to occupy processor
void delay (int a)
{
    volatile int i;

    for (i=0 ; i < a ; i++)
    {}
}
