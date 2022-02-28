#include "stm32f3xx.h"                  // Device header
#include <stdbool.h>

static int index=0;
int arr[5]={0,5,10,15,20}; // calculated values for CCR1

void PWM_init(int);
void interrupt_init();


void EXTI0_IRQHandler()
{
	if (EXTI->PR & EXTI_PR_PR0) // check source
	{
		EXTI->PR |= EXTI_PR_PR0; // clear flag*
		if (index > 4){index = 0;}// reset index if max value reached
		PWM_init(arr[index++]);
	}
};

int main()
{
	interrupt_init();
	PWM_init(20);
}

void interrupt_init()
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


void PWM_init(int dutyCycle)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;// enable clock connection for TIMER1
	RCC->AHBENR |= RCC_AHBENR_GPIOEEN ;// enable clock on GPIOE
	
	GPIOE->MODER = (GPIOE->MODER & (~0x3C0C0000))|0x28080000; // PE.9,PE13,PE14 configured to alternate mode
	
	GPIOE->OTYPER = (GPIOE->OTYPER& (~0x6200)) | 0x6200; // open drain for PE9,PE13,PE14
	GPIOE->PUPDR = (GPIOE->PUPDR & (~0x3C0C0000))|0x14040000; // pull up registor for PE9,PE13,PE14 
	
	GPIOE-> AFR[1] = (GPIOE->AFR[1] & ~(0xFF000F0))|0x2200020; // PE.9,PE.13 and PE.14 

	TIM1->PSC = 3999;
	TIM1->ARR= 19; // around 100Hz or 0.01s
	
	TIM1->CCMR1 |= 0x00000060; // channel 1 for PE.9
	TIM1->CCMR2 = (TIM1->CCMR2 & ~0xF0F0)|0x6060;// channel 3 and 4(PE.13 and PE.14 )
	
	TIM1->CCR1 = dutyCycle; //determine the duty cycle, (CCR1/(ARR+1))*100 == Duty_cycle%
	TIM1->CCR3 = dutyCycle; //determine the duty cycle
	TIM1->CCR4 = dutyCycle; //determine the duty cycle
	
//Enable the Channel chosen to be output to the GPIO pin
	TIM1->BDTR |= TIM_BDTR_MOE;// 0x00008000
	TIM1->CCER = (TIM1->CCER & ~0x1101) | 0x1101; // channel 1,3 and channel 4
//------------------------------------------------------

	TIM1->CR1 |= TIM_CR1_CEN;
	
}
