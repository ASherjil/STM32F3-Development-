#include <stdbool.h>
#include "stm32f3xx.h"                  // Device header


static volatile bool direction = false; // true = clockwise, false = anti-clockwise 
static int counter = 0;
const int states[4] = {0b10,0b00,0b01,0b11}; // states of the encoder stored in an integer array 
static int state = 0; // state of the encoder

void encoder_signal();

void TIM3_IRQHandler()
 {
	 
 if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the 'Update' interrupt flag
 {
		encoder_signal();
 }
		TIM3->SR &= ~TIM_SR_UIF; // Reset 'update' interrupt flag in the SR register
}



int main(void)
{
	
// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
//-----------------LED Pins 8,9 set to output mode 
	GPIOE->MODER = (GPIOE->MODER & ~(0xF0000))| 0x50000;// output mode "01" for pins(8,9)
	GPIOE->OTYPER &= ~(0x300); // push/pull "00" for pins(8,9)
	GPIOE->PUPDR &= ~(0xF0000); // no pullup, pull-down for pins(8,9)
//--------------------------------------------------------------------------------	
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Define the clock pulse toTIM3
	TIM3->PSC = 65535;
	TIM3->ARR = (int)121.0703; // every 1 second the interrupt is called
	TIM3->CR1 |= TIM_CR1_CEN;//Set Timer Control Register to start timer
	
	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
	NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer 3 interrupt request in NVIC
}

void encoder_signal()
{
	/*
	if (counter > 3)// if a direction is done 
	{
		counter=0; // reset counter 
		if (direction){direction= false;} // toggle direction flag 
		else
		{
			direction = true;
		}
	}*/
	
	
	GPIOE -> BSRRH = 0x300; // 0b11<<8, this turns OFF leds on PE8,9 to visualize the encoder signal 	
	if (direction)// clockwise direction
	{
			switch(state)
			{
				case 0:
					GPIOE->BSRRL = (states[0]<<8); // 0b10 << 8  
					state = 1; // move on to the next state
				break;
				
				case 1:
					GPIOE->BSRRL = (states[1]<<8); // 0b00 << 8  
					state = 2; // move on to the next state
				break;
				
				case 2:
					GPIOE->BSRRL = (states[2]<<8); // 0b01 << 8  
					state = 3; // move on to the next state
				break;
				
				case 3:
					GPIOE->BSRRL = (states[3]<<8); // 0b11 << 8  
					state = 0; // move on to the next state
				break;	
			}
	}
	
	else if(!direction) // anti-clock wise direction
	{
		switch(state)
			{
				case 0:
					GPIOE->BSRRL = (states[3]<<8); // 0b11 << 8   
					state = 1; // move on to the next state
				break;
				
				case 1:
					GPIOE->BSRRL = (states[2]<<8); // 0b01 << 8   
					state = 2; // move on to the next state
				break;
				
				case 2:
					GPIOE->BSRRL = (states[1]<<8); // 0b00 << 8   
					state = 3; // move on to the next state
				break;
				
				case 3:
					GPIOE->BSRRL = (states[0]<<8); // 0b10 << 8   
					state = 0; // move on to the next state
				break;	
			}
	}
	
//	++counter;
}
