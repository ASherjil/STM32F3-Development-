#include "stm32f3xx.h"                  // Device header

static int flag = 0;
static int mask = 1;//0b01

void counter();
void wait(int a);
void ADC_init();

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

	ADC_init();// initialise the ADC	
	int flag2=0;	
	
	ADC1->CR |= 0x4; // enable ADC
	while (!(ADC1->ISR & 0x4)) {}// wait for ARDY flag to go high 
	
	while (1)
	{
			if (!flag2)// if LEDs are off
			{
				GPIOE->BSRRL =	(ADC1->DR)<<8;// voltage read from the ADC goes to the LED
				flag2=1;//toggle flag2
			}
			else if (flag2==1)
			{
				GPIOE->BSRRH=0xFF00; // turn off LEDs
				flag2 =0;//change flag to go back to the first if statement 
			}
	}
	

}

void wait(int count)
{
	int ticks=0;
	
	while (ticks<count)// exit when ticks reach the count
	{
		++ticks;
	}
}


void counter()
{
		if (flag == 1)// if LED is on
		{			
			if (mask >255){mask = 1;}//reset mask if max value is reached
			
			mask+=1;
			flag =0;// change flag
			
		}
		else if (flag==0)// if LED is off
		{
			DAC1->DHR12R1 = mask;
			flag =1;// change flag
		}
}


void ADC_init()
{
	ADC1->CR &= ~(0x30000000); // 0b00 for the ADVREGEN = reset
	ADC1->CR |= (0x10000000);// 0b01 for the ADVREGEN = enable 
		
	wait(100);// 10us delay function 
		
	ADC1->CR &= ~(0x40000000);// 0b0 0 single-ended calibration 
	ADC1->CR |= 0x80000000;// begin calibration 
	
	while (ADC1->CR & 0x80000000) {}  // wait for ADCAL to return 0
		
	//------------------Enabling clock peripherials
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1_2_COMMON->CCR |= 0x00010000;
	//---------------------------------------------
		
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// enable clock on GPIOA
	GPIOA->MODER |= 0x200; // analogue mode on PA.2 
		
	ADC1->CFGR |= (0x10);// 8 bit resolution 
	ADC1->CFGR &= ~(0x20);// align right 
	ADC1->CFGR &= ~(0x2000);// no continous 
		
	ADC1->SQR1 |= 0x40;// channel 1 in SQR1 
	ADC1->SQR1 &= ~(0x1);// only one conversion so length is 0
	
	ADC1->SMPR1 |= (0x18); // 7.5 ADC clock cycles 
		
	ADC1->CR |= 0x1; // ADEN set to high to enable ADC
	
	while (!(ADC1->ISR & 0x01)) {}// wait for ARDY flag to go high 
	
}
