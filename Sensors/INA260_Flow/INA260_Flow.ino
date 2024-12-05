/*
This code works on the Arduino Uno R4 WiFi, Arduino GIGA R1 WiFi, and ESP32-S2-DevKitM-1-N4R2.

ESP32-S2-DevKitM-1-N4R2
  SDA - Pin 8 
  SCL - Pin 9  
*/

#include <Adafruit_INA260.h>
#include <SensirionI2cSf06Lf.h>
#include <Wire.h>

Adafruit_INA260 currentSensor1 = Adafruit_INA260();
Adafruit_INA260 currentSensor2 = Adafruit_INA260();

SensirionI2cSf06Lf sensor;

uint8_t address1 = 0x40;
uint8_t address2 = 0x41;


void setup() {
  Serial.begin(9600);
  // Wait until serial port is opened
  while (!Serial) { delay(10); }

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

  Wire.begin();
  sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);

  delay(100);
  sensor.startH2oContinuousMeasurement();

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

  delay(5000);
}
