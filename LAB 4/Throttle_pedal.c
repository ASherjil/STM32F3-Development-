#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f3xx.h"                  // Device header
#include "Throttle_pedal.h"

//-------------------------------------------------POTENTIOMETER INIT
static int count = 0;
//-------------------------------------------------------------------



//-------------------------------------------------ENCODER EMULATOR
static volatile bool direction = false; // true = clockwise, false = anti-clockwise 
const int states[4] = {0,2,3,1}; // states of the encoder stored in an integer array,{0b00,0b10,0b11,0b01} 
static int state = 0; // state of the encoder
static int current_state=0; // for position measurement
static int last_state=0;// for position 
static int encoderCount = 63; // counter for encoder pulses
static int encoder_dir_counter = 0; // counter for inverting encoder direction to match triangular wave
//--------------------------------------------------------------------


//---------------------------------------------------------COMPUTING 5-POINT MOVING AVG
static int j=0;// index for cycling through array
double storage[5]={0};// store values from ADC
double sum=0,avg=0; // variables to find avg 
//-------------------------------------------------------------------------------------



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
	if (j>4){j=0;}// cycle through the storage array 
	storage[j++]= (DAC1->DHR12R1);// store results in the array
}
//-------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------ENCODER EMULATOR---------

void encoder_generator(void) // initialise the interrupt for timer interrupt 2
{
	// Enable clock on GPIO port B
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable clock for TIM2
	
	GPIOB->MODER = (GPIOB->MODER & ~(0xF000000))| 0x5000000;// output mode "01" for PB(12,13)
	GPIOB->OTYPER &= ~(0x3000); // push/pull "00" for pins(12,13)
	GPIOB->PUPDR &= ~(0xF000000); // no pullup, pull-down for pins(12,13)
	
	TIM2->PSC = 389;
  TIM2->ARR =  39;// set to 512Hz 
	TIM2->CR1 |= TIM_CR1_CEN;//  timer action is set in motion with the ‘enable’ command
	TIM2->DIER |= TIM_DIER_UIE; // Set DIER register to watch out for an 'Update' Interrupt Enable (UIE) – or 0x00000001
	NVIC_EnableIRQ(TIM2_IRQn); // Enable Timer 3 interrupt request in NVIC
}

void ext_interrupt1_init(void) // initialise interrupt on PA.1
{
	GPIOA->MODER &= ~(0xC); // pins A.1 set to input mode
	GPIOA->PUPDR = (GPIOA->PUPDR & ~(0xC))|(0x4); // PA.1 with pullup enabled 
	
		//Enable the system configuration controller to be connected to a system cloc
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// Enable clock on GPIO port A
	
//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	EXTI->IMR |= EXTI_IMR_MR1;
	
//Set interrupt trigger to be rising edge
	EXTI->RTSR |= EXTI_RTSR_TR1;
	
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;// PA.1
	
	NVIC_EnableIRQ(EXTI1_IRQn); // set the nvic
	NVIC_SetPriority(EXTI1_IRQn,1);// set priority to 1, second highest priority 
}	

void EXTI1_IRQHandler() // ext interrupt on PA.1
{
	if (EXTI->PR & EXTI_PR_PR1) // check source
	{
		EXTI->PR |= EXTI_PR_PR1; // clear flag*
		
		encoder_pos();
	}
};

void ext_interrupt2_init(void)// initialise interrupt on PC.3
{
	
	GPIOC->MODER &= ~(0xC0); // PC.3 set to input mode
	GPIOC->PUPDR = (GPIOC->PUPDR & ~(0xC0))|(0x40); // PC.3 with pullup enabled 
	
//Enable the system configuration controller to be connected to a system cloc
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;// Enable clock on GPIO port C
	
//Remove the mask to enable an interrupt to be generated using the EXTI_IMR register
	EXTI->IMR |= EXTI_IMR_MR3;
	
//Set interrupt trigger to be rising edge
	EXTI->RTSR |= EXTI_RTSR_TR3;
	

	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PC;
	
	NVIC_EnableIRQ(EXTI3_IRQn); // set the nvic
	NVIC_SetPriority(EXTI3_IRQn,2);// set priority to 2, third highest priority 
}

void EXTI3_IRQHandler() // ext interrupt on PC.3
{
	if (EXTI->PR & EXTI_PR_PR3) // check source
	{
		EXTI->PR |= EXTI_PR_PR3; // clear flag*
		
		encoder_pos();
	}
};

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
		
	if (encoder_dir_counter > 255)// 128 means a total(CHA+CHB) encoder counts of 64 have occured
	{
		encoder_dir_counter =0 ;// reset
		if (direction){direction=false;}// TOGGLE DIRECTION TO EMULATE A TRIANGLE WAVE
		else if(!direction){direction=true;}
	} 
	
	GPIOB -> BSRRH = 0x3000; // 0b11<<12, this turns OFF leds on PB12,13 to visualize the encoder signal 	
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
	++encoder_dir_counter;
}
//---------------------------------------------------------------------------------------------------------------

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

void CountLEDs_init()
{
	RCC->AHBENR |=  RCC_AHBENR_GPIOEEN;// Enable clock on GPIO port C
	GPIOE->MODER = (GPIOE->MODER & ~(0xFFFF0000))| 0x55550000;// output mode "01" for pins(8-15)
	GPIOE->OTYPER &= ~(0xFF00); // push/pull "00" for pins(8-15)
	GPIOE->PUPDR &= ~(0xFFFF0000); // no pullup, pull-down for pins(8-15)
}

void encoder_pos()
{
	    //Store Current State of CHA
    current_state = ((GPIOA -> IDR & 0x2)>>1); // PA.1
    //Check if previous state of CHA is different from current state
    //Which indicates a pulse has elapsed
    if ((last_state != current_state) && (current_state == 1)) 
		{
        //Check if CHA state is different to CHB to determine 
        //direction of rotation
        //Increment Counter according to direction
        if (((GPIOC->IDR & 0x8)>>3)!= current_state)
				{
        encoderCount++;
				} 
        else {encoderCount--;}
    }
		
		 last_state = current_state;
		
		if ((encoderCount > 63)||(encoderCount < -63) )// reset is max value is reached
		{
			encoderCount=0;
		}
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
				
				GPIOE->BSRRH = (0xFE00);  // turn OFF Leds PE.9-PE.15
			
				if(!(GPIOE->IDR & 0x100)) // turn ON Led PE.8 if OFF
				{
					GPIOE->BSRRL = 0x100; 
				}
			
				ADC1->CR |= 0x4; // enable ADC
				while (!(ADC1->ISR & 0x4)) {}// wait for EOC flag to go high
				GPIOE->BSRRL = ((ADC1->DR)/4) << 11; // turn ON Leds by shifting bits, the ADC is scaled to 5-bits 
			
			break;
			
			case ENCODER:
				
				GPIOE->BSRRH = (0xFD00); // turn OFF Leds PE.8, PE.10-15
				
				if(!(GPIOE->IDR & 0x200)) // turn ON Led PE.8 if OFF
				{
					GPIOE->BSRRL = 0x200; 
				}
				
				
				GPIOE->BSRRL = abs(encoderCount/2) << 11; // turn on LEDs by shifting bit to match PE11-15	
			
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
						sum += storage[i]/4;// divide by 4 for scaling numbers  
					}					
					__enable_irq(); // enable interrupts 
					
					avg = ((sum/5)+abs(encoderCount/2))/2; // sum/5 = avg of POT , (avg of POT + encoder)/2
					
					GPIOE->BSRRL = ((int)round(avg)) << 11; // cast the decimal value to int 
					sum = 0;
			break; 
		}	
}

//-----------------------------------------------------------------------------------------------------------
