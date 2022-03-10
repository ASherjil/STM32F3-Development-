#include "stm32f3xx.h"                  // Device header

static int count = 0;

void wait(int a);
void ADC_init();
void OpAmp_init();

void TIM3_IRQHandler()
 {
	 
 if ((TIM3->SR & TIM_SR_UIF) !=0) // Check interrupt source is from the 'Update' interrupt flag
 {
		++count;
		if (count > 255){count = 0;}// reset count to 1 if max value reached
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

//-------------------------------------
//DAC configuration 
  RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;
  DAC1->CR |= DAC_CR_BOFF1;
  DAC1->CR |= DAC_CR_EN1;
//-------------------------------------

	ADC_init();// initialise the ADC
	OpAmp_init(); // initialise the OP-AMP
		
	
	while (1)
	{
			DAC1->DHR12R1 = count; // write the value into the DAC
			ADC1->CR |= 0x4; // enable ADC
			while (!(ADC1->ISR & 0x4)) {}// wait for EOC flag to go high 
			
			GPIOE->BSRRH = 0xFF00; // turn OFF Leds 
			GPIOE->BSRRL = (ADC1->DR) <<8; // turn ON Leds by shifting bits	
						
	}
	

}

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
		
	wait(100);// 10us delay function 
		
	ADC1->CR &= ~(0x40000000);// 0b0 0 single-ended calibration 
	ADC1->CR |= 0x80000000;// begin calibration 
	
	while (ADC1->CR & 0x80000000) {}  // wait for ADCAL to return 0
		
	//------------------Enabling clock peripherials
	RCC->CFGR2 = (RCC->CFGR2 & ~0x1F0)|RCC_CFGR2_ADCPRE12_DIV2;// clear registor first then 10001: PLL clock divided by 2 
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	ADC1_2_COMMON->CCR |= 0x00010000;
	//---------------------------------------------
		
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// enable clock on GPIOA
	GPIOA->MODER |= 0x3; // analogue mode on PA.0, "11" 
		
	ADC1->CFGR |= (0x10);// 8 bit resolution 
	ADC1->CFGR &= ~(0x20);// align right 
	ADC1->CFGR &= ~(0x2000);// no continous 
		
	//ADC1->SQR1 |= 0x40;// channel 1 in SQR1 
	//ADC1->SQR1 &= ~(0x1);// only one conversion so length is 0
	//ADC1->SMPR1 |= (0x18); // 7.5 ADC clock cycles
		
	ADC1->SQR1 = (ADC1->SQR1& ~0x7C0)|0x40;// channel 1 in SQR1, "000001" 
	ADC1->SQR1 = (ADC1->SQR1& ~0xF)|0x1;// length of 1, "000001" 
	ADC1->SMPR1 = (ADC1->SMPR1 & ~0x38)|0x18;	// 7.5 ADC clock cycles 
		
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
