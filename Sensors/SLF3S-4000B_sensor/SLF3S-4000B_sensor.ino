/* 
Sensirion SLF3S-4000B liquid flow sensor 

Sensor       Arduino
------------------------- 
1.IRQn (Brown)  ->   NC
2.SDA (Red)     ->   SDA
3.VDD (Orange)  ->   3.3V
4.GND (Yellow)  ->   GND 
5.SCL (Green)   ->   SCL 
6.NC (Empty)    ->   NC

 

*/ 

#include <SensirionI2cSf06Lf.h>
#include <Wire.h>

SensirionI2cSf06Lf sensor;


void setup() {
  analogReadResolution(14);  // Increase dafault 10-bit ADC resolution to 14-bit
  Serial.begin(9600);        // Initialize serial communication
}

void loop() {
  // put your main code here, to run repeatedly:

}
