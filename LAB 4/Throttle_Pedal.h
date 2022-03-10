#ifndef THROTTLE_PEDAL_H
#define THROTTLE_PEDAL_H

#define PRESCALER1 255 // for 255 Hz frequency, DAC/triangle wave 
#define PRESCALER2 65535 // for 1 hz encoder 
#define ARR1 121  
#define ARR2 121
#define PERIOD1 255
#define MAX_AMPLITUDE 127

//-------------------------------------------------POTENTIOMETER INIT
void DAC_init(void); // initialise the DAC with a timer-based interrupt
void triangle_emulator(void); // write DAC values which output an analogue triangular signal 
//-------------------------------------------------------------------


//-------------------------------------------------ENCODER EMULATOR
void encoder_signal(void); // emulate the encoder signal 
void encoder_generator(void); // initialise the interrupt required to generate the encoder signal 
//--------------------------------------------------------------------

//-------------------------------------------------Op-Amp and ADC INIT
void OpAmp_init(void);
void ADC_init(void);
void wait(int);
//--------------------------------------------------------------------

//--------------------------------------Button Interrupt PA.0, EXTI0 and STATES 
void interrupt_init(void);
void test_options(void);
enum tests{POTENTIOMETER,ENCODER,COMBINED_TEST};
void CountLEDs_init(void);
void writeLEDs(void);
//-----------------------------------------------------------------------------




#endif
