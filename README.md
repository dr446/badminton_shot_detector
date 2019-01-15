# The Badminton Shot Detector

Diya Rajan, Selwyn College, dr446

## Summary

This project is fun way to help badminton players improve their game. The hardware of the system consists of two modules. One is placed on the wrist, like a watch, and contains an accelerometer, a microphone and an OLED screen. The other contains the microcontroller and power supply and is worn on the upper arm. 

The system detects a shot using the microphone and aims to classify it into one of four types; a lift, a drive, a clear or a drop. It does this by comparing the acceleration values of the sensor as the shot was played and comparing it to a "template" waveform for each shot.


## Repository Layout

The project uses the Warp-firmware framework. The source code is located in Warp-firmware/src/boot/ksdk1.1.0

The "main" function and the "badminton_shot_detector_routine", which does all the signal processing of the sensor inputs, are located in the file warp-kl03-ksdk1.1-boot.c. 

There are three sets of files for the three components I have used.

### The MPU-6050 accelerometer

The driver files are: 
devMPU6050.c and devMPU6050.h

- The "update_circular_buffer" function ensures that the 10 latest acceleration readings are stored in the circular buffer "acceleration_circular_buffer" at all times. 
- The "update_shot_buffer()" function is called right after a shot has been detected and transfers the contents of the circular buffer into a "waveform_buffer" that will be used to classify the detected shot. 

### The INMP401 MEMs microphone

The driver files are: 
devINMP401.c and devINMP401.h

The microphone is used to hear when a shot has been detected. It is an analog device and is connected to ADC_0 of the KL03 microcontroller. The hardware comparator is used to generate an interrupt service routine when the ADC outputs are greater than a set threshold. 

- The Interrupt Service Routine, "Microphone_ISR" simply raises a flag that a shot has been detected, which signals to the system to begin attempting to classify it.

### The SSD1331 OLED display. 

My driver files are: 
devSSD1331.c and devSSD1331.h

I have also used and adapted the mbed driver for the ssd1331 device, because it had functions for printing out text. My main adaptions were deleting unecessary functions, changing the driver to use my SPI communication function rather than mbed's and optimising the text drawing function to use lines rather than pixels to reduced the required number of SPI commands.
 
- The "draw_result" function takes in the shot name and the confidence level for that shot and uses mbed's text writing function to display it on the screen.

