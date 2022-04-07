#ifndef THROTTLE_PEDAL_H
#define THROTTLE_PEDAL_H

#define PRESCALER1 80 // for 210 Hz frequency, DAC/triangle wave 
#define ARR1 269

#define PRESCALER2 85 // for 1 hz encoder, triangluar wave simuation occurs at 512 Hz
#define ARR2 407

#define MAX_AMPLITUDE 3640 // triangle wave goes up to 3640 -> 0 -> 3640


void CountLEDs_init(void); // Initialise ALL LEDs

//-------------------------------------------------POTENTIOMETER INIT
void DAC_init(void); // initialise the DAC with a timer-based interrupt, PA.4
void triangle_emulator(void); // write DAC values which output an analogue triangular signal 
//-------------------------------------------------------------------


//-------------------------------------------------ENCODER EMULATOR
void encoder_signal(void); // emulate the encoder signal on PB.12 and PB.13 
void encoder_init(void); // initialise the interrupt required to generate the encoder signal
void ext_interrupt1_init(void); // initialise interrupt pins, PA.1(HIGHER priority)
void ext_interrupt2_init(void);// initialise interrupt pins , PC.3
void encoder_pos(void); // increment CW, decrement CCW
//--------------------------------------------------------------------

//-------------------------------------------------ADC INIT
void ADC_init(void);// PF.2 as input 
void wait(int);
void store_ADC(void); // a function to store ADC values in an array 
//--------------------------------------------------------------------


//--------------------------------------Button Interrupt PA.0, EXTI0 and STATES 
void interrupt_init(void);// button interrupt, PA.0
void test_options(void);
enum tests{POTENTIOMETER,ENCODER,COMBINED_TEST};
//-----------------------------------------------------------------------------


void writeLEDs(void); // MAIN function to run inside the int main() while loop 


#endif
