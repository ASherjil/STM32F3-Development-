#include "stm32f3xx.h"                  // Device header

static int flag = 0;
static int mask = 1;//0b01
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
	
//-------------------------------
// Configure ports for DAC
	GPIOA->MODER |= 0x300;
//	GPIOA->OTYPER &= ~(0x10); // open drain
//	GPIOE->PUPDR &= ~(0x100); // pull up resistor 
//--------------------------------
	
	GPIOE->MODER |= 0x55550000; // Set mode of each pin in port E
	GPIOE->OTYPER &= ~(0xFF000000); // Set output type for each pin required in Port E
	GPIOE->PUPDR &= ~(0x55550000); // Set Pull up/Pull down resistor configuration for Port E
	
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Define the clock pulse toTIM3
  TIM3->PSC = 89;
  TIM3->ARR = 8899;
  TIM3->CR1 |= TIM_CR1_CEN;//Set Timer Control Register to start timer
  TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
  NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer 3 interrupt request in NVIC

//-------------------------------------
//DAC configuration 
  RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
  DAC1->CR |= DAC_CR_BOFF1;
  DAC1->CR |= DAC_CR_EN1;
//-------------------------------------
}

void counter()
{
		if (flag == 1)// if LED is on
		{
			GPIOE->BSRRH = 0xFF00; // turn off
			
			if (mask >255){mask = 1;}//reset mask if max value is reached
			
			mask+=1;
			flag =0;// change flag
			
		}
		else if (flag==0)// if LED is off
		{
			DAC1->DHR12R1 = mask;
			GPIOE->BSRRL =	(DAC1->DHR12R1)<<8;// shift the DAC info to turn on LEDs 
			flag =1;// change flag
		}
}
