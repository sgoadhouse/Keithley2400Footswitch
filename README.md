# Keithley2400Footswitch
Arduino program to control a ProMicro that interfaces a Keithley over serial and outputs numbers when footswitch is pressed

## DESCRIPTION
Reads data from Keithley 2400 SourceMeter instrument and sends 
to keyboard when footswitch is pressed. So can focus on probing and
actual measurement value goes into a spreadsheet or some other document.

IMPORTANT: Be sure to have the correct Board (Sparkfun Pro Micro) AND
Processor (5V, 16 MHz) selected in the Arduino IDE. I once had 3.3V, 8 MHz
processor selected by mistake and keyboard access did not work.

Board: Sparkfun Pro Micro</br>
Processor: ATmega32U4 (5V, 16 MHz) </br>

## Customize
There are several notes and defines in the code for different desired modes like sweeping voltage and current. Modify as desired before building.

## Build
Use the Arduino IDE to compile and download to the Sparkfun Pro Micro.

