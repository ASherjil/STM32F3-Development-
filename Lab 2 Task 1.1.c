#include "stm32f3xx.h"                  // Device header

static int count = 1;//0b01
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

// Configure ports for DAC
	GPIOA->MODER |= 0x300; // PA.4 set to analogue mode 
//-----------------ALL LED PINS SET TO PUSH/PULL WITH NO No pull-up, pull-down
	GPIOE->MODER = (GPIOE->MODER & ~(0xFFFF0000))| 0x55550000;// output mode "01" for pins(8-15)
	GPIOE->OTYPER &= ~(0xFF00); // push/pull "00" for pins(8-15)
	GPIOE->PUPDR &= ~(0xFFFF0000); // no pullup, pull-down for pins(8-15)
//--------------------------------------------------------------------------------
	
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Define the clock pulse toTIM3
  TIM3->PSC = 65535;
  TIM3->ARR = (int)11.2070;
  TIM3->CR1 |= TIM_CR1_CEN;//Set Timer Control Register to start timer
  TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
  NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer 3 interrupt request in NVIC

//--------------------DAC configuration 
  RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
  DAC1->CR |= DAC_CR_BOFF1;
  DAC1->CR |= DAC_CR_EN1;
//-------------------------------------
}

void counter()
{
		if (count > 255){count=1;}
		DAC1->DHR12R1 = count ;// write counter into DAC
		GPIOE->BSRRH = 0xFF00; // turn OFF Leds 
		GPIOE->BSRRL = (DAC1->DHR12R1) <<8; // turn ON Leds by shifting bits
		++count;
}
