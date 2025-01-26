/*
Adafruit INA260 High or Low Side Voltage, Current, Power Sensor

Bus voltage = 0-36 VDC
Max current = 15A 
VCC/Logic = 3.3V or 5V

Sensor       Microcontroller Development Board
-------------------------------------------------- 
1.Vcc        ->   3.3V or 5V
2.GND        ->   GND
3.SCL        ->   SCL
4.SDA        ->   SDA
5.Alert      ->   NC
6.VBus       ->   NC
7.Vin+       ->   NC
8.Vin-       ->   NC

This code works on the Arduino Uno R4 WiFi, Arduino GIGA R1 WiFi, and ESP32-S2-DevKitM-1-N4R2.

ESP32-S2-DevKitM-1-N4R2
  SDA - Pin 8 
  SCL - Pin 9  
*/

#include <Adafruit_INA260.h>

Adafruit_INA260 currentSensor1 = Adafruit_INA260();
Adafruit_INA260 currentSensor2 = Adafruit_INA260();

uint8_t address1 = 0x40;
uint8_t address2 = 0x41;


void setup() {
  Serial.begin(9600);
  // Wait until serial port is opened
  while (!Serial) { 
    delay(10); 
  }

  Serial.println("Adafruit INA260 Test");

  if (!currentSensor1.begin(address1, &Wire)) {
    Serial.println("Couldn't find current sensor 1");
    while (1);
  }
  Serial.println("Found current sensor 1");

  if (!currentSensor2.begin(address2, &Wire)) {
    Serial.println("Couldn't find current sensor 2");
    while (1);
  }
  Serial.println("Found current sensor 2");
  Serial.println();
}

void loop() {

  //sensor 1
  Serial.print("Sensor 1 Current: ");
  Serial.print(currentSensor1.readCurrent());
  Serial.println(" mA");

  Serial.print("Sensor 1 Bus Voltage: ");
  Serial.print(currentSensor1.readBusVoltage()/1000.0);
  Serial.println(" V");

  Serial.print("Sensor 1 Power: ");
  Serial.print(currentSensor1.readPower());
  Serial.println(" mW");

  Serial.println();

  //sensor 2
  Serial.print("Sensor 2 Current: ");
  Serial.print(currentSensor2.readCurrent());
  Serial.println(" mA");

  Serial.print("Sensor 2 Bus Voltage: ");
  Serial.print(currentSensor2.readBusVoltage()/1000.0);
  Serial.println(" V");

  Serial.print("Sensor 2 Power: ");
  Serial.print(currentSensor2.readPower());
  Serial.println(" mW");
  
  Serial.println();


  delay(5000);
}
