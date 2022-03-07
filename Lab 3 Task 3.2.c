#include <stdbool.h>
#include "stm32f3xx.h"                  

int abs(int a)
{
	if (a>0){return a;}
	else
		return (-1*a);		
}


//-------------------------------------------------ENCODER STATE VARIABLES & FUNCTIONS
static volatile bool direction = true; // true = clockwise, false = anti-clockwise 
static int counter = 0;
const int states[4] = {0b10,0b00,0b01,0b11}; // states of the encoder stored in an integer array 
static int state = 0; // state of the encoder
void encoder_signal(); // emulate the encoder signal 
//-------------------------------------------------------------------------




//------------------------------------------------------DISPLAY ENCODER PULSES ON LEDS
void ext_interrupt1(); // initialise external interrupts
void ext_interrupt2(); // initialise external interrupts
static int encoderCount = 0; // counter for encoder pulses 
void displayLED(int); // display encoder count on the LEDs
//------------------------------------------------------------------------------------

void EXTI0_IRQHandler() // external interrupt channel 0 
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR |= EXTI_PR_PR0; // clear flag*
		
		if ((encoderCount > 15)||(encoderCount < -15)){encoderCount=0;}// reset when max 4-bit value is reached
		displayLED(encoderCount);// display the count on the LED PE.11-14
	}
};

void EXTI1_IRQHandler() // external interrupt channel 1
{
	if (EXTI->PR & EXTI_PR_PR1) // check source
	{
			EXTI->PR |= EXTI_PR_PR1; // clear flag*
		
			if ((encoderCount > 15)||(encoderCount < -15)){encoderCount=0;}// reset when max 4-bit value is reached
			displayLED(encoderCount);// display the count on the LED PE.11-14		
	}
}


void TIM3_IRQHandler()// Timer based interrupt 
 {
	 
 if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the 'Update' interrupt flag
 {
		encoder_signal();// generate singal using the state machine mechanism 
 }
		TIM3->SR &= ~TIM_SR_UIF; // Reset 'update' interrupt flag in the SR register
}


void LED_init() // initialise LED on PE.11-14
{
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;// Enable clock on GPIO port E
	
	GPIOE->MODER = (GPIOE->MODER& ~(0x3FC00000))|(0x15400000); // PE.11-14 set to output mode 
	GPIOE->OTYPER &= ~(0x7800); // Push/pull mode set for PE.11-14
	GPIOE->PUPDR &= ~(0x3FC00000);// no pullup, pull down for PE.11-14
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
	
	LED_init(); // initialise LEDs on PE11-14
	ext_interrupt1(); // initialise external interrupt on PA.0
	ext_interrupt2();// initialise external interrupt on PA.1
}

void encoder_signal()
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

void ext_interrupt1()
{
//Enable the system configuration controller to be connected to a system cloc
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	EXTI->IMR |= EXTI_IMR_MR0;
	
	EXTI->RTSR |= EXTI_RTSR_TR0; // trigger on rising edge
	EXTI->FTSR |= EXTI_FTSR_TR0; // trigger on falling edge
	
//The USR push button (blue button on the STM32F3discovery board) is connected to pin PA.0.
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	NVIC_EnableIRQ(EXTI0_IRQn); // set the nvic
	NVIC_SetPriority(EXTI0_IRQn,0);// set priority to 0
}

void ext_interrupt2()
{
	//Enable the system configuration controller to be connected to a system cloc
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	EXTI->IMR |= EXTI_IMR_MR1;
	
	EXTI->RTSR |= EXTI_RTSR_TR1; // trigger on rising edge
	EXTI->FTSR |= EXTI_FTSR_TR1; // trigger on falling edge
	
// Configure the second channel, PinA.1
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI1_PA;
	
	NVIC_EnableIRQ(EXTI1_IRQn); // set the nvic
	NVIC_SetPriority(EXTI1_IRQn,0);// set priority to 0
}

void displayLED(int encoder_count)
{
	GPIOE->BSRRH = (0xF)<<11; // turn off all LEDs on PE11-14
	GPIOE->BSRRL = abs(encoder_count) << 11; // turn on LEDs by shifting bit to match PE11-14
}
