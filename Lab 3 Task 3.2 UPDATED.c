#include <stdbool.h>
#include <stdlib.h>
#include "stm32f3xx.h"                  


//-------------------------------------------------ENCODER STATE VARIABLES & FUNCTIONS
static volatile bool direction = true; // true = clockwise, false = anti-clockwise 
const int states[4] = {0,2,3,1}; // states of the encoder stored in an integer array,{0b00,0b10,0b11,0b01} 
static int state = 0; // state of the encoder
void timer_init(void);// initialise timer based interrupt 
void encoder_signal(void); // emulate the encoder signal 
void encoder_pos(void); // CHA = PE.9 connected to PA.0, CHB= PE.8 connected to PA.1
static int current_state[2] = {0,0}; // index 0 = CHA, index 1= CHB
static int last_state[2] = {0,0};// index 1 = CHB, index 1 = CHB 
//-------------------------------------------------------------------------



//------------------------------------------------------DISPLAY ENCODER PULSES ON LEDS
void ext_interrupt1(void); // initialise external interrupts
void ext_interrupt2(void); // initialise external interrupts
void interrupt_pins_init(void);// initialise the input external interrupt pins PA0,PA1
static int encoderCount = 0; // counter for encoder pulses
void LED_init(void); // initialise LEDs on PE8,9,11-14
void displayLED(int); // display encoder count on the LEDs
//------------------------------------------------------------------------------------

static int counter1=0;
static int counter2=0;


void EXTI0_IRQHandler() // external interrupt channel 0 
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR |= EXTI_PR_PR0; // clear flag*
		
		if (abs(encoderCount)>15){encoderCount=0;}// reset when max 4-bit value is reached
		encoder_pos();
		displayLED(encoderCount);// display the count on the LED PE.11-14
		++counter1;
	}
};

void EXTI1_IRQHandler() // external interrupt channel 1
{
	if (EXTI->PR & EXTI_PR_PR1) // check source
	{
			EXTI->PR |= EXTI_PR_PR1; // clear flag*
		
			if (abs(encoderCount)>15){encoderCount=0;}// reset when max 4-bit value is reached
			encoder_pos();
			displayLED(encoderCount);// display the count on the LED PE.11-14
			++counter2;
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

int main(void)
{
	LED_init(); // initialise LEDs on PE8,9,11-14
	interrupt_pins_init(); // initialise the input external interrupt pins PA0,PA1
	ext_interrupt1(); // initialise external interrupt on PA.0
	ext_interrupt2();// initialise external interrupt on PA.1
	timer_init(); // generate encoder signal on PE8,PE9 at 1Hz square wave

}

void encoder_signal()
{
			
	GPIOE -> BSRRH = 0x300; // 0b11<<8, this turns OFF leds on PE8,9 to visualize the encoder signal 	
	if (!direction)// anti-clockwise direction
	{
			switch(state)
			{
				case 0:
					GPIOE->BSRRL = (states[0]<<8); // 0b00 << 8  
					state = 1; // move on to the next state
				break;
				
				case 1:
					GPIOE->BSRRL = (states[3]<<8); // 0b01 << 8  
					state = 2; // move on to the next state
				break;
				
				case 2:
					GPIOE->BSRRL = (states[2]<<8); // 0b11 << 8  
					state = 3; // move on to the next state
				break;
				
				case 3:
					GPIOE->BSRRL = (states[1]<<8); // 0b10 << 8  
					state = 0; // move on to the next state
				break;	
			}
	}
	
	else if(direction) // clockwise direction
	{
		switch(state)
			{
				case 0:
					GPIOE->BSRRL = (states[0]<<8); // 0b00 << 8   
					state = 1; // move on to the next state
				break;
				
				case 1:
					GPIOE->BSRRL = (states[1]<<8); // 0b10 << 8   
					state = 2; // move on to the next state
				break;
				
				case 2:
					GPIOE->BSRRL = (states[2]<<8); // 0b11 << 8   
					state = 3; // move on to the next state
				break;
				
				case 3:
					GPIOE->BSRRL = (states[3]<<8); // 0b01 << 8   
					state = 0; // move on to the next state
				break;	
			}
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
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;
	
	NVIC_EnableIRQ(EXTI1_IRQn); // set the nvic
	NVIC_SetPriority(EXTI1_IRQn,0);// set priority to 1
}

void displayLED(int encoder_count)
{
	GPIOE->BSRRH = (0xF)<<11; // turn off all LEDs on PE11-14
	GPIOE->BSRRL = abs(encoder_count) << 11; // turn on LEDs by shifting bit to match PE11-14
}

void encoder_pos()
{
	   
    current_state[0] = (GPIOA -> IDR & 0x1); //Store Current State of CHA
    current_state[1] = (GPIOA -> IDR & 0x2)>>1; //Store Current State of CHB
	
	
	if (current_state[0]!= last_state[0]) // CHA triggered the interrupt 
	{
		if (current_state[0] ^ last_state[1]) // CHA_new XOR CHB_old
		{
			++encoderCount;
		}
		else 
		{
			--encoderCount;
		}
		
		last_state[0] = current_state[0]; // update CHA state 
	}
	else if (current_state[1] != last_state[1])// CHB triggered the interrupt 
	{
		if (last_state[0] ^ current_state[1]) // CHA_old XOR CHB_new
		{
			--encoderCount;
		}
		else
		{
			++encoderCount;
		}
		last_state[1] = current_state[1]; // update CHB state 
	}
    
}

void LED_init() // initialise LEDs on PE8,9,11-14
{
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN;// Enable clock on GPIO port E
	
	//-----------------LED Pins 8,9 set to output mode 
	GPIOE->MODER = (GPIOE->MODER & ~(0xF0000))| 0x50000;// output mode "01" for pins(8,9)
	GPIOE->OTYPER &= ~(0x300); // push/pull "00" for pins(8,9)
	GPIOE->PUPDR &= ~(0xF0000); // no pullup, pull-down for pins(8,9)
//--------------------------------------------------------------------------------	
	
	GPIOE->MODER = (GPIOE->MODER& ~(0x3FC00000))|(0x15400000); // PE.11-14 set to output mode 
	GPIOE->OTYPER &= ~(0x7800); // Push/pull mode set for PE.11-14
	GPIOE->PUPDR &= ~(0x3FC00000);// no pullup, pull down for PE.11-14
}

void interrupt_pins_init()
{
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// Enable clock on GPIO port A
	
		GPIOA->MODER &= ~(0xF); // pins A.0+A.1 both set to input mode
		GPIOA->PUPDR = (GPIOA->PUPDR & ~(0xF))|(0x5); // PA.0 and PA.1 with pullup enabled 
}

void timer_init()// initialise timer based interrupt 
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Define the clock pulse toTIM3
	TIM3->PSC = 65535;
	TIM3->ARR = (int)121.0703; // every 1 second the interrupt is called
	TIM3->CR1 |= TIM_CR1_CEN;//Set Timer Control Register to start timer
	
	TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
	NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer 3 interrupt request in NVIC
}
