#include "stm32f3xx.h"                  // Device header
#include <stdbool.h>


void PWM_init(); // initialise the PWM 


int main()
{
		PWM_init();
	
		GPIOE->BSRRL = 0x6200;// turn on LEDs	
}

void PWM_init()
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;// enable clock connection for TIMER1
	GPIOE->MODER = (GPIOE->MODER & ~0x3C0C0000)|28080000; // PE.9,PE13,PE14 configured to alternate mode
	GPIOE->OTYPER &= ~(0x6200); // push/pull for PE9,PE13,PE14
	GPIOE->PUPDR &= ~(0x3C0C0000); // no pullup, pull down  
	
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN ;// enable clock on GPIOE
	
	GPIOE-> AFR[1] |= 0xED000900; // PE.9,PE.13 and PE.14 

	TIM1->PSC = 79999999;
	TIM1->ARR= 9; // around 100Hz or 0.01s
	
	TIM1->CCMR1 |= 0x00000060; // channel 1 
	TIM1->CCMR2 |= 0x6060; // channel 3 and 4(PE.13 and PE.14 )

	TIM1->CCR1 = 1; //determine the duty cycle, 10%
	TIM1->CCR3 = 5; //determine the duty cycle, 50%
	TIM1->CCR4 = 9; //determine the duty cycle, 90&
	
//Enable the Channel chosen to be output to the GPIO pin
	TIM1->BDTR |= TIM_BDTR_MOE;// 0x00008000
	TIM1->CCER |= TIM_CCER_CC1E;//0x00000001, channel 1
	TIM1->CCER |= 0x1101; // channel 3 and channel 4
//------------------------------------------------------

	TIM1->CR1 |= TIM_CR1_CEN;
	
};
