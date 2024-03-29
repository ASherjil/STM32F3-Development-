# Comparing Encoder and Potentiometer readings for angular displacement measurements

The microcontroller used in this project was the STM32F303 Discovery Board. The software was developed in Keil uVision without
the use of HAL library. 

The STM32 board is used to generate signals to emulate a potentiometer and a quadrature encoder. The encoder signals are then connected to 2 pins with external interrupts enabled. The potentiometer signal is produced by a DAC which is then connected to the ADC. 
These two measurement devices are supposed to represent a throttle pedal of a vehicle where the pedal starts at 80 degrees and is floored at 0 degrees. The encoder and the potentiometer keep track of the angular displacement and present the position information on the on board LEDs.

Encoder Resolution = 512 CPR

DAC                = 12-bit

ADC                = 12-bit

This is shown in the simple diagrams below:

<img width="800" alt="image" src="https://user-images.githubusercontent.com/92602684/177421359-6b43d3e1-0d10-4cf3-9ce3-c3ad2931c50b.png">

<img width="750" alt="image" src="https://user-images.githubusercontent.com/92602684/177421099-91c3bff2-8fcb-4e6a-a426-357ff1df0992.png">


The STM32F3 board produced 3 types of output:

1- Potentiometer test: displays the position only of the potentiometer.

2- Encoder test: displays the position only of the Encoder.

3- Combined output : displays the position by computing the 5-point moving average.

The diagram below shows how they are the position is displayed on the onboard LEDs. 

<img width="600" alt="image" src="https://user-images.githubusercontent.com/92602684/177422101-4f8dfbe3-ad6f-4b4a-9bfc-756f02d8011c.png">

![GitHub code size in bytes](https://img.shields.io/github/languages/code-size/areebTP/STM32F3-Development-)

# Installation and Usage

1- Find files "WSC355 Keil Download and Setup.pdf" and follow the steps until "Testing the IDE" section.

2- Create a new project and add files found in folder "Potentiometer and Encoder" :
 
 "Throttle_Pedal.h" + "Throttle_Pedal.c" + "mainFile.c"
 
3- Wire up the STM32 board as pointed out in the header file("Throttle_Pedal.h"). Pressing the blue push button will switch between the different testing modes. 
 
# Other Related Projects

Another interesting project I worked on was creating a GUI polynomial long division calculator using C++17 with the Qt framework. Check it out:

https://github.com/ASherjil/Polynomial_Long_Division_GUI
