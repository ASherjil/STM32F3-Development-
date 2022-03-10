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


//-----------------------------------------------------------------POTENTIOMETER INIT--------------------------
void DAC_init(void)
{
	
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Define the clock pulse to TIM3
  TIM3->PSC = PRESCALER1;
  TIM3->ARR =  ARR1;// generate the triangle wave at 1Hz frequency 
  TIM3->CR1 |= TIM_CR1_CEN;//Set Timer Control Register to start timer
  TIM3->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
  NVIC_EnableIRQ(TIM3_IRQn); // Enable Timer 3 interrupt request in NVIC

//--------------------DAC configuration 
  RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
	GPIOA->MODER |= 0x300; // PA.4 set to analogue mode,Configure ports for DAC 
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
//-------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------ENCODER EMULATOR

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
//-------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------ADC and OP-Amp INIT-----
void wait(int c)
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
	
	ADC1->CFGR = (ADC1->CFGR & ~0x18)|(0x10);// 8 bit resolution 
	ADC1->CFGR &= ~(0x20);// align right 
	ADC1->CFGR &= ~(0x2000);// no continous 
				
	ADC1->SQR1 = (ADC1->SQR1& ~0x3C0)|0x280;// channel 10 in SQR1, "01010" 
	ADC1->SQR1 &= ~(0xF);// only one conversion, L is "0000"
	ADC1->SMPR1 |= 0x38; // 601.5 ADC clock cycles 
	
	ADC1->CR |= 0x1; // ADEN set to high to enable ADC
	
	while (!(ADC1->ISR & 0x01)) {}// wait for ARDY flag to go high 
}
void OpAmp_init()
{
//The OpAmp is connected to the system clock via the APB2 peripheral clock bus
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	GPIOA->MODER |= 0xC00;// input PA.5 set to analogue mode
	GPIOA->MODER |= 0x30; //output PA.2 set to analogue mode "11"

// Enable the OpAmp by setting the OPAMP1EN bit high in the CSR register	
	OPAMP1->CSR |= 0x00000001;
//Define the inputs to be used by specifying the correct VM_SEL and VP_SEL bits in the CSR register
	OPAMP1->CSR |= 0x00000004; // PA.5 as non-inv. Input
	OPAMP1->CSR |= 0x00000040;// Set VM_SEL to 0b10 to enable PGA mode
	
	OPAMP1->CSR = (OPAMP1->CSR & ~0x3C000)|0xC000;// gain set to 16 
}
//-------------------------------------------------------------------------------------------------------------------------------

//------------------------------------------------BUTTON INTERRUPT INIT
void interrupt_init(void)
{	
	//Enable the system configuration controller to be connected to a system cloc
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	EXTI->IMR |= EXTI_IMR_MR0;
	
//Set interrupt trigger to be rising edge, falling edge or both using the Rising Trigger Selection
	EXTI->RTSR |= EXTI_RTSR_TR0;
	
//The USR push button (blue button on the STM32F3discovery board) is connected to pin PA.0.
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	
	NVIC_EnableIRQ(EXTI0_IRQn); // set the nvic
	NVIC_SetPriority(EXTI0_IRQn,0);// set priority to 0
};

void EXTI0_IRQHandler() // button interrupt on PA.0
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR |= EXTI_PR_PR0; // clear flag*
		
		test_options(); // switch state 
	}
};

void CountLEDs_init()
{
	GPIOE->MODER = (GPIOE->MODER & ~(0xFFC00000))| 0x55400000;// output mode "01" for pins(11-15)
	GPIOE->OTYPER &= ~(0xF800); // push/pull "00" for pins(11-15)
	GPIOE->PUPDR &= ~(0xFFC00000); // no pullup, pull-down for pins(11-15)
}


enum tests testing = POTENTIOMETER;

void test_options()
{	
		switch (testing)
		{
			case POTENTIOMETER:
			testing = ENCODER;
			break;
			
			case ENCODER:
			testing = COMBINED_TEST;	
			break;
			
			case COMBINED_TEST:
			testing = POTENTIOMETER;	
			break; 
		}		
}

void writeLEDs()
{
	switch (testing)
		{
			case POTENTIOMETER:
			GPIOE->BSRRL = (ADC1->DR) <<11; // turn ON Leds by shifting bits
			break;
			
			case ENCODER:
			
			break;
			
			case COMBINED_TEST:
				
			break; 
		}	
}

//-----------------------------------------------------------------------------------------------------------
