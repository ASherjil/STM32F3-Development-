#ifndef THROTTLE_PEDAL_H
#define THROTTLE_PEDAL_H

/*
LAB 4 

Student: B820928
PIN CONNECTIONS 

ENCODER CONNECTIONS:
1- PB.13 -> PA.1
2- PB.12 -> PC.3

DAC Trianagle WAVE Connections:
DAC (PA.4) -> ADC(PF.2) 
*/
#include <stdbool.h>// for boolean values 
#include <stdlib.h> // for abs() function
#include "stm32f3xx.h"               

#define PRESCALER1 85 // Triangle wave simuation occurs at 226 Hz
#define ARR1 407

/* The counter increments by 32 each time, making the triangle wave go from
	32*113 = 3616 then back down to 0. The DAC value is then scaled to 5-bits.
	
*/
//---------------------------------------FUNCTION PROTOTYYPES---------------------------------
void CountLEDs_init(void); // Initialise ALL LEDs
//-------------------------------------------------POTENTIOMETER INIT
void DAC_init(void); // initialise the DAC with a timer-based interrupt, PA.4
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



#endif
