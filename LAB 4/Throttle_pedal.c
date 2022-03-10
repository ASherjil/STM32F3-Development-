#include <stdbool.h>
#include <stdlib.h>
#include "stm32f3xx.h"                  // Device header
#include "Throttle_pedal.h"

//-------------------------------------------------POTENTIOMETER INIT
static int count = 0;
//-------------------------------------------------------------------



//-------------------------------------------------ENCODER EMULATOR
static volatile bool direction = true; // true = clockwise, false = anti-clockwise 
const int states[4] = {2,0,1,3}; // {0b10,0b00,0b01,0b11},states of the encoder stored in an integer array, 
static int state = 0; // state of the encoder
static int encoderCount = 0; // counter for encoder pulses 
//--------------------------------------------------------------------



void DAC_init(void)
{

	GPIOA->MODER |= 0x300; // PA.4 set to analogue mode,Configure ports for DAC 
	
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Define the clock pulse to TIM3
  TIM3->PSC = PRESCALER1;
  TIM3->ARR =  ARR1;// generate the triangle wave at 1Hz frequency 
  TIM3->CR1 |= TIM_CR1_CEN;//Set Timer Control Register to start timer
  TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
  NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer 3 interrupt request in NVIC

//--------------------DAC configuration 
  RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
  DAC1->CR |= DAC_CR_BOFF1;
  DAC1->CR |= DAC_CR_EN1;
//-------------------------------------
	
}

void TIM3_IRQHandler() // Timer interrupt 1
 {
	 
 if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the 'Update' interrupt flag
 {
		triangle_emulator(); // generate the triangle wave 
 }
		TIM3->SR &= ~TIM_SR_UIF; // Reset 'update' interrupt flag in the SR register
}


void triangle_emulator(void)
{
	DAC1->DHR12R1 = abs((count++ % PERIOD1)-MAX_AMPLITUDE); // triangle wave writing 0-127,127-0 to the DAC
}

void encoder_generator(void) // initialise the interrupt for timer interrupt 2
{
	// Enable clock on GPIO port E
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable clock for TIM2
	
	GPIOE->MODER = (GPIOE->MODER & ~(0xF0000))| 0x50000;// output mode "01" for pins(8,9)
	GPIOE->OTYPER &= ~(0x300); // push/pull "00" for pins(8,9)
	GPIOE->PUPDR &= ~(0xF0000); // no pullup, pull-down for pins(8,9)
	
	
	TIM2->PSC = PRESCALER2;
  TIM2->ARR =  ARR2;// set to 1Hz frequency 
	TIM2->CR1 |= TIM_CR1_CEN;//  timer action is set in motion with the ‘enable’ command
	TIM2->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
	NVIC_EnableIRQ(TIM2_IRQn); // Enable Timer 3 interrupt request in NVIC
}

void TIM2_IRQHandler()
{
	if ((TIM2->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the ‘Update’ interrupt flag
	{
		encoder_signal();
	}
	TIM2->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
}

void encoder_signal() // emulates the encoder signal using state machine mechanism 
{
			
	GPIOE -> BSRRH = 0x300; // 0b11<<8, this turns OFF leds on PE8,9 to visualize the encoder signal 	
	if (!direction)// anti-clockwise direction
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
			--encoderCount;
	}
	
	else if(direction) // clockwise direction
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
			++encoderCount;// increment count everytime a rising/falling edge occurs
	}
	
}
