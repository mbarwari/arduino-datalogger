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

Pull-up resistors: 
  1.8K pull-up resistor from SDA to VDD
  1.8K pull-up resistor from VDD to SCL

This code works on the Arduino Uno R4 WiFi, Arduino GIGA R1 WiFi, and ESP32-S2-DevKitM-1-N4R2.

ESP32-S2-DevKitM-1-N4R2
  SDA - Pin 8 
  SCL - Pin 9  

*/ 

#include <SensirionI2cSf06Lf.h>
#include <Wire.h>

SensirionI2cSf06Lf sensor;

void setup() {
  
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }
  
  Wire.begin();
  sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);

  delay(100);
  sensor.startH2oContinuousMeasurement();
}

void loop() {

  float aFlow = 0.0;
  float aTemperature = 0.0;    
  uint16_t aSignalingFlags = 0u;
  delay(20);
  sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F, aFlow, aTemperature, aSignalingFlags);
   
  Serial.print("aFlow: ");
  Serial.print(aFlow);
  Serial.print("\t");
  Serial.print("aTemperature: ");
  Serial.print(aTemperature);
  Serial.print("\t");
  Serial.print("aSignalingFlags: ");
  Serial.print(aSignalingFlags);
  Serial.println();
}
