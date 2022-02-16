#include "stm32f3xx.h"                  // Device header


static int flag = 0;
static int mask = 1;

void counter();

void TIM3_IRQHandler()
 {
	 
 if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the 'Update' interrupt flag
 {
		counter();
 }
		TIM3->SR &= ~TIM_SR_UIF; // Reset 'update' interrupt flag in the SR register
}
 


int main(void)
{
	
// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
// GPIOE is a structure defined in stm32f303xc.h file
// Define settings for each output pin using GPIOE structure
/*
	LEDs going from PE8-PE15 this would mean 
	MODER 8-15
	
MODER  15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00 
		0b 01 01 01 01 01 01 01 01 00 00 00 00 00 00 00 00	
*/
	GPIOE->MODER |= 0x55550000; // Set mode of each pin in port E
	GPIOE->OTYPER &= ~(0xFF000000); // Set output type for each pin required in Port E
	GPIOE->PUPDR &= ~(0x55550000); // Set Pull up/Pull down resistor configuration for Port E
	
//Interrupt Setting before Task 3,all of these goes into main()
//Clock Speed is actually 72 MHz for APB1ENR
 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Define the clock pulse toTIM3
 TIM3->PSC = 899;
 TIM3->ARR = 8899;
 TIM3->CR1 |= TIM_CR1_CEN;//Set Timer Control Register to start timer
 TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
 NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer 3 interrupt request in NVIC


}

void counter()
{
		if (flag == 1)// if LED is on
		{
			GPIOE->BSRRH = mask<<8; // turn off LED
			
			if (mask > 255){mask = 1;}//reset mask if max value is reached
			
			mask+=1;
			flag =0;// change flag 
			
		}
		else if (flag==0)// if LED is off
		{
			GPIOE->BSRRL =mask<<8;// turn on LED
			flag =1; // change flag
		}
}
