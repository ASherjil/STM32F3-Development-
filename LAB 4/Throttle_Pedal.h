#ifndef THROTTLE_PEDAL_H
#define THROTTLE_PEDAL_H

#define PRESCALER1 255 // for 255 Hz frequency, DAC/triangle wave 
#define PRESCALER2 255 // for 1 hz encoder, triangluar wave simuation 
#define ARR1 121  
#define ARR2 60
#define PERIOD1 255
#define MAX_AMPLITUDE 127

//-------------------------------------------------POTENTIOMETER INIT
void DAC_init(void); // initialise the DAC with a timer-based interrupt
void triangle_emulator(void); // write DAC values which output an analogue triangular signal 
//-------------------------------------------------------------------


//-------------------------------------------------ENCODER EMULATOR
void encoder_signal(void); // emulate the encoder signal on PE.8 and PE.9 
void encoder_generator(void); // initialise the interrupt required to generate the encoder signal
void ext_interrupt1_init(void); // initialise interrupt pins, PA.1(HIGHER priority)
void ext_interrupt2_init(void);// initialise interrupt pins , PC.3
void encoder_pos(void); // increment CW, decrement 
static int current_state=0;
static int last_state=0;
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
void CountLEDs_init(void); // Initialise LED PE.11-15
void writeLEDs(void);
//-----------------------------------------------------------------------------


#endif
