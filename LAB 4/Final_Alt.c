/*
PIN CONNECTIONS 

ENCODER CONNECTIONS:
1- PB.13 -> PA.1
2- PB.12 -> PC.3

DAC Trianagle WAVE Connections:
DAC (PA.4) -> ADC(PF.2) 
*/
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f3xx.h"               

#define PRESCALER1 85 // Triangle wave simuation occurs at 226 Hz
#define ARR1 407

/* The counter increments by 32 each time, making the triangle wave go from
	3616 then back down to 0. The DAC value is then scaled to 5-bits.
	
*/
//---------------------------------------FUNCTION PROTOTYYPES---------------------------------
void CountLEDs_init(void); // Initialise ALL LEDs
//-------------------------------------------------POTENTIOMETER INIT
void DAC_init(void); // initialise the DAC with a timer-based interrupt, PA.4
void triangle_emulator(void); // write DAC values which output an analogue triangular signal 
//-------------------------------------------------------------------
//-------------------------------------------------ENCODER EMULATOR
void triangle_generator(void); // emulate the encoder signal on PB.12 and PB.13 
void encoder_init(void); // initialise the interrupt required to generate the encoder signal
void ext_interrupt1_init(void); // initialise interrupt pins, PA.1
void ext_interrupt2_init(void);// initialise interrupt pins , PC.3
void encoder_pos(void); // increment CW, decrement CCW
//--------------------------------------------------------------------
//-------------------------------------------------ADC INIT
void ADC_init(void);// PF.2 as input 
void wait(int); // delay function for ADC 
void store_ADC(void); // a function to store ADC values in an array 
//--------------------------------------------------------------------
//--------------------------------------Button Interrupt PA.0, EXTI0 and STATES 
void interrupt_init(void);// button interrupt, PA.0
void test_options(void);
enum tests{POTENTIOMETER,ENCODER,COMBINED_TEST};
//-----------------------------------------------------------------------------
void writeLEDs(void); // MAIN function to run inside the int main() while loop 





//-------------------------------------------------POTENTIOMETER INIT
static long count = 0; // counter variable for producing triangle wave 
const float scaler1 = (0xD40/31.0); // ADC_12-bit_value/31 = scaling factor for DAC triangle wave
static volatile bool count_up = false;
//-------------------------------------------------------------------



//-------------------------------------------------ENCODER EMULATOR
static volatile bool direction = true; // true = clockwise, false = anti-clockwise
const int states[4] = {0,2,3,1}; // states of the encoder stored in an integer array,{0b00,0b10,0b11,0b01} 
static int state = 0; // state of the encoder
static int current_state[2]={0,0}; // for position measurement
static int last_state[2]={0,0};// for position 
static volatile int encoderCount = 0; // counter for encoder pulses
static volatile int encoder_dir_counter = 0; // counter for inverting encoder direction to match triangular wave
const float scaler2 = (113/31.0); // encoderCounts converted to 5-bit, 113/31
//--------------------------------------------------------------------


//---------------------------------------------------------COMPUTING 5-POINT MOVING AVG
static int j=0;// index for cycling through array
float storage[5]={0};// store values from ADC
float sum=0,avg=0; // variables to find avg
//-------------------------------------------------------------------------------------




int main(void)
{
	ADC_init();// PF.2 as input 
	interrupt_init();// button interrupt, PA.0
	CountLEDs_init();// init ALL LEDS(PE.8-PE.15)
	
	ext_interrupt1_init(); // init interrupt on PA.1
	ext_interrupt2_init();// init interrupt on PC.3
	encoder_init(); // initialise the encoder emulator on PB.12 and PB.13
	DAC_init(); // generate triangle wave at 1Hz frequency, PA.4 

	
	while(1)
	{
		writeLEDs(); // run the main function 
	}

}








//----------------------------------------------------------------ALL LEDS init---------------------------------
void CountLEDs_init() 
{
	RCC->AHBENR |=  RCC_AHBENR_GPIOEEN;// Enable clock on GPIO port C
	GPIOE->MODER = (GPIOE->MODER & ~(0xFFFF0000))| 0x55550000;// output mode "01" for pins(8-15)
	GPIOE->OTYPER &= ~(0xFF00); // push/pull "00" for pins(8-15)
	GPIOE->PUPDR &= ~(0xFFFF0000); // no pullup, pull-down for pins(8-15)
}

//---------------------------------------------------------------POTENTIOMETER INIT--------------------------
void DAC_init(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
  GPIOA->MODER |= 0x300; // PA.4 set to analogue mode,Configure ports for DAC 
  DAC1->CR |= DAC_CR_BOFF1;
  DAC1->CR |= DAC_CR_EN1;
}
//-------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------ENCODER EMULATOR---------

void encoder_init(void) // initialise the interrupt for timer interrupt 2
{
	// Enable clock on GPIO port B
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable clock for TIM2
	
	GPIOB->MODER = (GPIOB->MODER & ~(0xF000000))| 0x5000000;// output mode "01" for PB(12,13)
	GPIOB->OTYPER &= ~(0x3000); // push/pull "00" for pins(12,13)
	GPIOB->PUPDR &= ~(0xF000000); // no pullup, pull-down for pins(12,13)
	
	TIM2->PSC = PRESCALER1;
	TIM2->ARR =  ARR1;
	TIM2->CR1 |= TIM_CR1_CEN;//  timer action is set in motion with the ‘enable’ command
	TIM2->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
	NVIC_EnableIRQ(TIM2_IRQn); // Enable Timer 2 interrupt request in NVIC
}

void ext_interrupt1_init(void) // initialise interrupt on PA.1
{
	GPIOA->MODER &= ~(0xC); // pins A.1 set to input mode
	GPIOA->PUPDR = (GPIOA->PUPDR & ~(0xC))|(0x4); // PA.1 with pullup enabled 
		
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;//Enable the system configuration controller to be connected to a system cloc
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// Enable clock on GPIO port A
	
//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	EXTI->IMR |= EXTI_IMR_MR1;
	

	EXTI->RTSR |= EXTI_RTSR_TR1;//Set interrupt trigger to be rising edge
	EXTI->FTSR |= EXTI_RTSR_TR1;//Set interrupt trigger to be rising edge
	
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;// PA.1
	
	NVIC_EnableIRQ(EXTI1_IRQn); // set the nvic
	NVIC_SetPriority(EXTI1_IRQn,0);// set priority to 1, second highest priority 
}	

void EXTI1_IRQHandler() // ext interrupt on PA.1
{
	if (EXTI->PR & EXTI_PR_PR1) // check source
	{
		EXTI->PR |= EXTI_PR_PR1; // clear flag*
		
		encoder_pos(); // function to determine direction of rotation  
	}
};

void ext_interrupt2_init(void)// initialise interrupt on PC.3
{
	
	GPIOC->MODER &= ~(0xC0); // PC.3 set to input mode
	GPIOC->PUPDR = (GPIOC->PUPDR & ~(0xC0))|(0x40); // PC.3 with pullup enabled 	

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;//Enable the system configuration controller to be connected to a system cloc
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;// Enable clock on GPIO port C
	

	EXTI->IMR |= EXTI_IMR_MR3;//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	
	EXTI->RTSR |= EXTI_RTSR_TR3;//Set interrupt trigger to be rising edge
	EXTI->FTSR |= EXTI_RTSR_TR3;//Set interrupt trigger to be rising edge
	
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
	
	NVIC_EnableIRQ(EXTI3_IRQn); // set the nvic
	NVIC_SetPriority(EXTI3_IRQn,0);// set priority to 2, third highest priority 
}

void EXTI3_IRQHandler() // ext interrupt on PC.3
{
	if (EXTI->PR & EXTI_PR_PR3) // check source
	{
		EXTI->PR |= EXTI_PR_PR3; // clear flag*
		
		encoder_pos();// function to determine direction of rotation  
	}
};

void TIM2_IRQHandler()
{
	if ((TIM2->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the ‘Update’ interrupt flag
	{
		triangle_generator(); // generate triangle wave for Pot and encoder 
	}
	TIM2->SR &= ~TIM_SR_UIF; // Reset ‘Update’ interrupt flag in the SR register
}

void triangle_generator() // emulates the encoder signal using state machine mechanism 
{
//----------------------------Generate Triangle wave----------------------------
	if (count_up == false) // generate triangle wave, increase
	{
		count += 32; // increment by 32
		DAC1->DHR12R1 = count;
		store_ADC(); // store DAC data in array 
	}
	else if (count_up== true)// generate triangle wave, decrease 
	{
		count -= 32; // decrement by 32
		DAC1->DHR12R1 = count;
		store_ADC(); // store DAC data in array 
	}
	
	if (encoder_dir_counter > 113) // reset when 113 encoder counts have occurred 
	{
		encoder_dir_counter =0 ;// reset
		direction = !direction; // toggle direction
		state = 0; // reset state
		count_up = (!count_up); // toggle flag for generating triangle wave 
	} 
//--------------------------------------Generate Encode signal-------------------------------------	
	GPIOB -> BSRRH = 0x3000; // 0b11<<12, making both of the signals low to emalate a square wave signal  	
	if (!direction)// anti-clockwise direction
	{
			switch(state)
			{
				case 0:
					GPIOB->BSRRL = (states[0]<<12); // 0b00 << 12  
					state = 1; // move on to the next state
				break;
				
				case 1:
					GPIOB->BSRRL = (states[3]<<12); // 0b01 << 12  
					state = 2; // move on to the next state
				break;
				
				case 2:
					GPIOB->BSRRL = (states[2]<<12); // 0b11 << 12  
					state = 3; // move on to the next state
				break;
				
				case 3:
					GPIOB->BSRRL = (states[1]<<12); // 0b10 << 12  
					state = 0; // move on to the next state
				break;	
			}
	}
	
	else if(direction) // clockwise direction
	{
		switch(state)
			{
				case 0:
					GPIOB->BSRRL = (states[0]<<12); // 0b00 << 12   
					state = 1; // move on to the next state
				break;
				
				case 1:
					GPIOB->BSRRL = (states[1]<<12); // 0b10 << 12   
					state = 2; // move on to the next state
				break;
				
				case 2:
					GPIOB->BSRRL = (states[2]<<12); // 0b11 << 12   
					state = 3; // move on to the next state
				break;
				
				case 3:
					GPIOB->BSRRL = (states[3]<<12); // 0b01 << 12   
					state = 0; // move on to the next state
				break;	
			}
	}
	
	++encoder_dir_counter; // increment variable for changing direction 
}
void encoder_pos() // function to determine direction of encoder from external interrupts 
{  
    current_state[0] = (GPIOA -> IDR & 0x2)>>1; //Store Current State of CHA
    current_state[1] = (GPIOC -> IDR & 0x8)>>3; //Store Current State of CHB
	
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
//---------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------ADC INIT-----
void wait(int c) // delay function for ADC 
{
	int ticks=0;
	
	while (ticks<c)// exit when ticks reach the count
	{
		++ticks;
	}
}

void ADC_init()
{
	ADC1->CR &= ~(0x30000000); // 0b00 for the ADVREGEN = reset
	ADC1->CR |= (0x10000000);// 0b01 for the ADVREGEN = enable 
	
	RCC->AHBENR |= RCC_AHBENR_GPIOFEN;// enable clock on GPIOF
	GPIOF->MODER |= 0x30; // analogue mode on PF.2, "11" 
		
	wait(100);// 10us delay function 
		
	ADC1->CR &= ~(0x40000000);// 0b0 0 single-ended calibration 
	ADC1->CR |= 0x80000000;// begin calibration 
	
	while (ADC1->CR & 0x80000000) {}  // wait for ADCAL to return 0
		
	//------------------Enabling clock peripherials
	RCC->CFGR2 = (RCC->CFGR2 & ~0x1F0)|RCC_CFGR2_ADCPRE12_DIV2;// clear registor first then 10001: PLL clock divided by 2 
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1_2_COMMON->CCR |= 0x00010000;
	//---------------------------------------------
		
	ADC1->CR &= ~(0xC);// set ADSTART and JADSTART to 0, "00 00"	
	
	ADC1->CFGR &= ~(0x18); // 12 bit resolution 
	ADC1->CFGR &= ~(0x20);// align right 
	ADC1->CFGR &= ~(0x2000);// no continous 
				
	ADC1->SQR1 = (ADC1->SQR1& ~0x3C0)|0x280;// channel 10 in SQR1, "01010" 
	ADC1->SQR1 &= ~(0xF);// only one conversion, L is "0000"
	ADC1->SMPR1 |= 0x38; // 601.5 ADC clock cycles 
	
	ADC1->CR |= 0x1; // ADEN set to high to enable ADC
	
	while (!(ADC1->ISR & 0x01)) {}// wait for ARDY flag to go high 
}

int ADC_read() // a function to continously read from the ADC, return the integer value of the ADC
{
	ADC1->CR |= 0x4; // enable ADC
	while (!(ADC1->ISR & 0x4)) {}// wait for EOC flag to go high
		
	return (ADC1->DR);
}

void store_ADC() // store triangle wave inside an array for 5-point moving average
{
	if (j>4){j=0;}// cycle through the storage array 
	storage[j++]= ADC_read();// store results in the array
}


//-------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------BUTTON INTERRUPT INIT----------------------------------------------------------
void interrupt_init(void)
{	
	//Enable the system configuration controller to be connected to a system cloc
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	EXTI->IMR |= EXTI_IMR_MR0;
	
//Set interrupt trigger to be rising edge
	EXTI->RTSR |= EXTI_RTSR_TR0;
	
//The USR push button (blue button on the STM32F3discovery board) is connected to pin PA.0.
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	NVIC_EnableIRQ(EXTI0_IRQn); // set the nvic
	NVIC_SetPriority(EXTI0_IRQn,0);// set priority to 0, highest priority
};

void EXTI0_IRQHandler() // button interrupt on PA.0
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR |= EXTI_PR_PR0; // clear flag*
		
		test_options(); // switch state
	}
};


enum tests testing = POTENTIOMETER; // start at potentiometer test

void test_options() // state machine mechanism to respond to blue push button 
{	
		switch (testing)
		{
			case POTENTIOMETER:
			testing = ENCODER; // move on to the next state 
			break;
			
			case ENCODER:
			testing = COMBINED_TEST;// move on to the next state 	
			break;
			
			case COMBINED_TEST:
			testing = POTENTIOMETER;// move on to the next state 	
			break; 
		}		
}
//--------------------------------------------------MAIN FUNCTION TO RUN INSIDE int main()-----
void writeLEDs()
{
	switch (testing)
		{
			case POTENTIOMETER:
				
				GPIOE->BSRRH = (0xFE00);  // turn OFF Leds PE.9-PE.15
			
				if(!(GPIOE->IDR & 0x100)) // turn ON Led PE.8 if OFF
				{
					GPIOE->BSRRL = 0x100; 
				}

				// the following scales the ADC value to 5-bit then converts that float into integer 
				GPIOE->BSRRL = ((int)((ADC_read()/scaler1))) << 11; // turn ON Leds by shifting bits, the ADC is scaled to 5-bits 
				
			break;
			
			case ENCODER:
				
				GPIOE->BSRRH = (0xFD00); // turn OFF Leds PE.8, PE.10-15
				
				if(!(GPIOE->IDR & 0x200)) // turn ON Led PE.9 if OFF
				{
					GPIOE->BSRRL = 0x200; 
				}
				
				// scale the encoder count to 5-bit then cast to integer 
				GPIOE->BSRRL = ((int)(abs(encoderCount)/scaler2)) << 11; // turn on LEDs by shifting bit to match PE11-15	
				
			break;
			
			case COMBINED_TEST:
				
				GPIOE->BSRRH = (0xFB00); // turn OFF Leds PE.8-9, PE.11-15
				
				if(!(GPIOE->IDR & 0x400)) // turn ON Led PE.10 if OFF
				{
					GPIOE->BSRRL = 0x400;  
				}
				__disable_irq(); // disable interrupts 
				for (size_t i=0;i<5;++i)
				{
					sum += storage[i]/scaler1;// scale the array values to fit 5-bit 
				}

				avg =((sum/5)+(abs(encoderCount)/scaler2))/2; // sum/5 = avg of POT , (avg of POT + encoder)/2
				__enable_irq();// enable interrupts 
					
				GPIOE->BSRRL = ((int)(avg)) << 11; // cast the decimal value to int after rounding 
				sum = 0;// reset sum to zero for next iteration 
				
			break; 
		}	
}
//-----------------------------------------------------------------------------------------------------------
