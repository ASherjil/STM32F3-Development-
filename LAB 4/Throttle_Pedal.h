#ifndef THROTTLE_PEDAL_H
#define THROTTLE_PEDAL_H

#define PRESCALER1 255 // for 256 Hz frequency, DAC/triangle wave 
#define ARR1 121
#define PRESCALER2 39 // for 1 hz encoder, triangluar wave simuation occurs at 512 Hz
#define ARR2 389
#define PERIOD1 255
#define MAX_AMPLITUDE 127

//-------------------------------------------------POTENTIOMETER INIT
void DAC_init(void); // initialise the DAC with a timer-based interrupt
void triangle_emulator(void); // write DAC values which output an analogue triangular signal 
//-------------------------------------------------------------------


//-------------------------------------------------ENCODER EMULATOR
void encoder_signal(void); // emulate the encoder signal on PB.12 and PB.13 
void encoder_generator(void); // initialise the interrupt required to generate the encoder signal
void ext_interrupt1_init(void); // initialise interrupt pins, PA.1(HIGHER priority)
void ext_interrupt2_init(void);// initialise interrupt pins , PC.3
void encoder_pos(void); // increment CW, decrement 
//--------------------------------------------------------------------

//-------------------------------------------------Op-Amp and ADC INIT
void OpAmp_init(void); // PA.5 as input, PA.2 as output 
void ADC_init(void);// PF.2 as input 
void wait(int);
//--------------------------------------------------------------------

//--------------------------------------Button Interrupt PA.0, EXTI0 and STATES 
void interrupt_init(void);// button interrupt, PA.0
void test_options(void);
enum tests{POTENTIOMETER,ENCODER,COMBINED_TEST};
void CountLEDs_init(void); // Initialise ALL LEDs
void writeLEDs(void);
//-----------------------------------------------------------------------------


#endif
