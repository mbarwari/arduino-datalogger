/*
MUX datalogger  
Date: 9/1/2024

Temperature in Celsius 
Flow rate in ml/min 
Pressure in PSI 
Currenet in mA
Voltage in V
*/

// include necessary libraries
#include <Wire.h>
#include "WiFiS3.h"
#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
#include <Adafruit_INA260.h>

// Digital pins for the IRQn pins of the flow sensors to connect to
#define IRQN_PIN_SENSOR_A 13
#define IRQN_PIN_SENSOR_B 12
#define IRQN_PIN_SENSOR_C 11

// I2C addresses for the flow sensors
#define I2C_ADDR_SENSOR_A 0x0A
#define I2C_ADDR_SENSOR_B 0x0B
#define I2C_ADDR_SENSOR_C 0x0C

// Define the flow sensor objects 
SensirionI2cSf06Lf sensorA;
SensirionI2cSf06Lf sensorB;
SensirionI2cSf06Lf sensorC;

// Error message for flow sensor
static char errorMessage[64];
static int16_t error;

// Define current sensor objects and I2C addresses 
Adafruit_INA260 pumpINA260 = Adafruit_INA260();
Adafruit_INA260 peltierINA260 = Adafruit_INA260();
uint8_t pumpI2CAddress = 0x40;
uint8_t peltierI2CAddress = 0x41;

// Pressure sensor specifications
const float Vsupply = 5.0;  // Supply voltage
const float Pmin = 0.0;     // Minimum pressure in PSI
const float Pmax = 15.0;    // Maximum pressure in PSI

const int sampleSize = 10;   // Sample size
const int decimalPlaces = 3;  // Decimal places for Serial.print()

// Thermistor related global variables and macros
#define NUMSAMPLES 10
#define SERIESRESISTOR 10000
#define BCOEFFICIENT 3895
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25

// General global variables
unsigned long waqt;
int tempCount, pressureCount, flowCount, currentCount, ncCount;

// WIFI related global variables, macros, and object
#define SECRET_SSID "pigTrial"
#define SECRET_PASS "123456789"
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

// MUX1 control pins, S0-S3 (digital pins)
int mux1_s0 = 9;
int mux1_s1 = 8;
int mux1_s2 = 7;
int mux1_s3 = 6;

// MUX2 control pins, S0-S3 (digital pins)
int mux2_s0 = 5;
int mux2_s1 = 4;
int mux2_s2 = 3;
int mux2_s3 = 2;

// MUX1 and MUX2 signal pins, SIG (analog pins)
int mux1_sig = 0;
int mux2_sig = 1;


// setup() function is called once when microcontroller starts
void setup() {

  // set the control pins of MUX1 (S0-S3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from
  pinMode(mux1_s0, OUTPUT);
  pinMode(mux1_s1, OUTPUT);
  pinMode(mux1_s2, OUTPUT);
  pinMode(mux1_s3, OUTPUT);

  // set the initial state of MUX1 (S0-S3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.
  digitalWrite(mux1_s0, LOW);
  digitalWrite(mux1_s1, LOW);
  digitalWrite(mux1_s2, LOW);
  digitalWrite(mux1_s3, LOW);

  // set the control pins of MUX2 (S0-S3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from
  pinMode(mux2_s0, OUTPUT);
  pinMode(mux2_s1, OUTPUT);
  pinMode(mux2_s2, OUTPUT);
  pinMode(mux2_s3, OUTPUT);

  // set the initial state of MUX2 (S0-S3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.  digitalWrite(mux2_s0, LOW);
  digitalWrite(mux2_s0, LOW);
  digitalWrite(mux2_s1, LOW);
  digitalWrite(mux2_s2, LOW);
  digitalWrite(mux2_s3, LOW);

  // starts the serial communication at a baud rate of 9600
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }

  Serial.println("I2C setup: "); 

  // change the flow sensor I2C addresses and ......
  flowSensorSetUp();

  // set up the current sensors 
  currentSensorSetUp();

  // set the reference voltage for analog-to-digital conversion to an external source for accuracy
  analogReference(AR_EXTERNAL);

  Serial.println("WIFI setup: "); 

  // set up the WiFi Access Point
  setupWiFiAP();

  // start the WiFi server
  // the server will listen for incoming connections on port 80 (the default port for HTTP)
  server.begin();
}


// loop() function runs continuously after the setup() function completes
void loop() {

  // compare previous WIFI status to current WIFI status
  if (status != WiFi.status()) {
    // WIFI status has changed so update status variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  // listen for incoming clients
  WiFiClient client = server.available();

  if (client) {
    // if you get a client

    tempCount = 0;
    pressureCount = 0;
    flowCount = 0;
    currentCount = 0;
    ncCount = 0;
    // Loop through and read, convert, and display all 16 channels from MUX 1
    for (int i = 0; i < 16; i++) {
      tempCount += 1;
      int sig = setMux(1, i);
      float temp = readThermistor(sig);

      client.print(waqt);
      client.print(" ");
      client.print("T");
      client.print(tempCount);
      client.print(" ");
      client.println(temp);
    }

    // Loop through and read, convert, and display all 16 channels from MUX 2
    for (int i = 0; i < 16; i++) {
      if (i < 12) {
        // MUX channels 0-11 have thermistors connected so
        // read, convert, and display the temperature (in Celsius) from the thermistors
        tempCount += 1;
        int sig = setMux(2, i);
        float temp = readThermistor(sig);

        client.print(waqt);
        client.print(" ");
        client.print("T");
        client.print(tempCount);
        client.print(" ");
        client.println(temp);
      } else if (i < 15) {
        // MUX channels 12-14 have pressure sensors connected so
        // read, convert, and display the pressure (in PSI) from the pressure sensors
        pressureCount += 1;
        int sig = setMux(2, i);
        float pressure = readPressure(sig);

        client.print(waqt);
        client.print(" ");
        client.print("P");
        client.print(pressureCount);
        client.print(" ");
        client.println(pressure);
      } else {
        // last MUX channel (C15) is not connected
        ncCount += 1;

        client.print(waqt);
        client.print(" ");
        client.print("NC");
        client.println(ncCount);
      }
    }

    // read and display the flow rate (in ml/min) from the flow sensors
    printFlowSensorOutput(sensorA, client);
    printFlowSensorOutput(sensorB, client);
    printFlowSensorOutput(sensorC, client);

    // read and display the voltage and current of the pump and peltier from the current sensors
    printCurrentSensorOutput(pumpINA260, client); 
    printCurrentSensorOutput(peltierINA260, client); 


    // delay for 5 seconds
    delay(5000);
  }
}


/*
printCurrentSensorOutput() -  
Parameters - none
Return - none (function return type: void) 
*/
void printCurrentSensorOutput(Adafruit_INA260& sensor, WiFiClient& client) {

  currentCount += 1;
  client.print(waqt);
  client.print(" ");
  client.print("C");
  client.print(currentCount);
  client.print(" ");
  client.print(sensor.readBusVoltage()/1000.0);
  client.println(" V");
}


/*
currentSensorSetUp() -  
Parameters - none
Return - none (function return type: void) 
*/
void currentSensorSetUp() {

  if (!pumpINA260.begin(pumpI2CAddress)) {
    Serial.println("Couldn't find pump current sensor");
    while (1);
  }
  Serial.println("Found pump current sensor");


  if (!peltierINA260.begin(peltierI2CAddress)) {
    Serial.println("Couldn't find peltier current sensor");
    while (1);
  }
  Serial.println("Found peltier current sensor");
  Serial.println();
}


/*
readPressure(int sig_pin) -  
Parameters - 
  int sig_pin
Return - pressureApplied (function return type: float) 
*/
float readPressure(int sig_pin) {
  delay(50);
  int total = 0;

  for (int i = 0; i < sampleSize; i++) {
    // Read the voltage from the pressure sensor
    // analogRead() returns an integer between 0-1023 (or 0-16383 if resolution changed to 14-bit)
    int sensorValue = analogRead(sig_pin);
    total += sensorValue;
    delay(0);
  }

  // Find the average of the sensor value readings 
  int averageSensorValue = total / sampleSize;

  // Convert the analog reading to voltage (0-5V)
  //float outputVoltage = averageSensorValue * (5.0 / 16383.0);
  //float outputVoltage = averageSensorValue * (5.0 / 1023.0);
  float outputVoltage = averageSensorValue * (3.0 / 1023.0);

  // pressureApplied = 15/(0.8*5)*(Vout-0.5) + 0
  float pressureApplied = 15 / (0.8 * 5) * (outputVoltage - 0.5) + 0;

  return pressureApplied;
}


/*
printFlowSensorOutput(SensirionI2cSf06Lf& sensor, WiFiClient& client) - 
Parameters - 
  SensirionI2cSf06Lf& sensor
  WiFiClient& client
Return - none (function return type: void) 
*/
void printFlowSensorOutput(SensirionI2cSf06Lf& sensor, WiFiClient& client) {
  float aFlow = 0.0;
  float aTemperature = 0.0;
  uint16_t aSignalingFlags = 0u;
  delay(20);
  sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, aFlow, aTemperature, aSignalingFlags);

  tempCount += 1;
  client.print(waqt);
  client.print(" ");
  client.print("T");
  client.print(tempCount);
  client.print(" ");
  client.println(aTemperature);

  flowCount += 1;
  client.print(waqt);
  client.print(" ");
  client.print("F");
  client.print(flowCount);
  client.print(" ");
  client.println(aFlow);
}


/*
IMPORTANT - this function was copied from the program "exampleI2cAddressChange.ino" under the examples from the "Sensirion I2C SF06-LF" library   
flowSensorSetUp()
Parameters - none
Return - none (function return type: void) 
*/
void flowSensorSetUp() {
  error = NO_ERROR;

  Wire.begin();

  // Make sure that sensors are in proper state to perform a address change by
  // doing a soft reset and not sending any other commands prior to the
  // address change procedure
  i2c_soft_reset();
  // SLF3x sensors need 25ms to start up after the reset
  delay(25);

  // Change address of the first sensor
  // Set IRQN_PIN_SENSOR_A to the GPIO pin number where you connected Pin 1
  // of your first sensor.
  error = changeSensorAddress(Wire, I2C_ADDR_SENSOR_A, IRQN_PIN_SENSOR_A);
  if (error != NO_ERROR) {
    Serial.print("Error changing sensor address: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  // Change address of the first sensor
  // Set IRQN_PIN_SENSOR_B to the GPIO pin number where you connected Pin 1
  // of your second sensor.
  error = changeSensorAddress(Wire, I2C_ADDR_SENSOR_B, IRQN_PIN_SENSOR_B);
  if (error != NO_ERROR) {
    Serial.print("Error changing sensor address: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  // Change address of the first sensor
  // Set IRQN_PIN_SENSOR_B to the GPIO pin number where you connected Pin 1
  // of your third sensor.
  error = changeSensorAddress(Wire, I2C_ADDR_SENSOR_C, IRQN_PIN_SENSOR_C);
  if (error != NO_ERROR) {
    Serial.print("Error changing sensor address: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  // Initialize first sensor
  Serial.println("Initialising flow sensor A");
  sensorA.begin(Wire, 0x0A);
  //readAndPrintSerial(sensorA);
  error = sensorA.startH2oContinuousMeasurement();
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startH2oContinuousMeasurement() for sensor A: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  // Initialize second sensor
  Serial.println("Initialising flow sensor B");
  sensorB.begin(Wire, 0x0B);
  //readAndPrintSerial(sensorB);
  error = sensorB.startH2oContinuousMeasurement();
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startH2oContinuousMeasurement() for sensor B: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  // Initialize third sensor
  Serial.println("Initialising flow sensor C");
  sensorC.begin(Wire, 0x0C);
  //readAndPrintSerial(sensorC);
  error = sensorC.startH2oContinuousMeasurement();
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute startH2oContinuousMeasurement() for sensor C: ");
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }
}


/*
IMPORTANT - this function was copied from the program "exampleI2cAddressChange.ino" under the examples from the "Sensirion I2C SF06-LF" library   
i2c_soft_reset()   
Parameters - none 
Return - none (function return type: void) 
*/
void i2c_soft_reset() {
  Wire.beginTransmission(0x00);
  size_t writtenBytes = Wire.write(0x06);
  uint8_t i2c_error = Wire.endTransmission();
}


/*
IMPORTANT - this function was copied from the program "exampleI2cAddressChange.ino" under the examples from the "Sensirion I2C SF06-LF" library   
changeSensorAddress(TwoWire& wire, uint16_t newI2cAddress, uint8_t sensorIrqPin)
Parameters - 
  TwoWire& wire
  uint16_t newI2cAddress 
  uint8_t sensorIrqPin
Return - NO_ERROR (function return type: int16_t) 
*/
int16_t changeSensorAddress(TwoWire& wire, uint16_t newI2cAddress, uint8_t sensorIrqPin) {
  uint8_t communication_buffer[5] = { 0 };
  int16_t localError = NO_ERROR;
  uint8_t* buffer_ptr = communication_buffer;

  // Send I2C address change command 0x3661 with the new I2C address as a
  // parameter (including CRC for the parameter)
  SensirionI2CTxFrame txFrame = SensirionI2CTxFrame::createWithUInt16Command(0x3661, buffer_ptr, 5);
  txFrame.addUInt16(newI2cAddress);
  // Note that the command is sent to the default address 0x08 of the sensor
  localError = SensirionI2CCommunication::sendFrame(SLF3C_1300F_I2C_ADDR_08, txFrame, wire);
  if (localError != NO_ERROR) {
    Serial.println("error sending address change command");
    errorToString(localError, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    Serial.println("As there are multiple sensors attached initially listening on the same I2C address \
        the acknowledge might overlap and cause an error which you can ignore if the subsequent communication is successful.");
  }

  // set IRQN pin of one sensor to high for at least 150μs to confirm address
  // change only after this pulse has been sent the sensor actually accepts
  // the new I2C address sent before
  pinMode(sensorIrqPin, OUTPUT);
  digitalWrite(sensorIrqPin, HIGH);
  delayMicroseconds(500);
  // reset IRQn pin back to low state
  digitalWrite(sensorIrqPin, LOW);

  // switch mode to input and listen to the pulse the sensor
  // sends 1500μs after the address change command to confirm the new I2C
  // address
  pinMode(sensorIrqPin, INPUT_PULLDOWN);
  delayMicroseconds(500);
  uint8_t success = 0;
  uint16_t cnt = 0;
  while (success == 0 && cnt < 100) {
    cnt++;
    success = digitalRead(sensorIrqPin);
    delayMicroseconds(10);
  }
  if (success == 0) {
    // return error as sensor did not acknowledge address change
    return -1;
  }

  Serial.print("Flow sensor address changed to: 0x");
  if (newI2cAddress < 16) {
    Serial.print("0");
  }
  Serial.println(newI2cAddress, HEX);
  return NO_ERROR;
}


/*
readThermistor(int sig_pin) - converts thermistor value to temperature using the Steinhart-Hart equation
Parameters - 
  int sig_pin -   
Return - (function return type: int)
  float steinhart - temperature in celsius of the thermistor  
*/
float readThermistor(int sig_pin) {

  // read the value at the SIG pin
  waqt = millis() / 1000;
  uint16_t val[NUMSAMPLES] = { 0 };
  delay(50);
  for (uint8_t j = 0; j < NUMSAMPLES; j++) {  // take N samples in a row, with a slight delay
    val[j] = analogRead(sig_pin);
    delay(0);
  }

  float avgval = 0;
  for (int k = 0; k < NUMSAMPLES; k++) {
    avgval += val[k];
  }
  avgval = SERIESRESISTOR / (1023 / (avgval / NUMSAMPLES) - 1);
  float steinhart = 1 / ((log(avgval / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.5;  // (R/Ro)

  return steinhart;
}


/*
setMux(int mux, int channel) - sets the MUX channel based off the parameters  
Parameters - 
  int mux - represents which MUX(1-2) to read from. int must be [1,2] for function to work properly.  
  int channel - represents which channel(0-15) to read from. int must be [0,15] for function to work properly.
Return - (function return type: int)
  int sig_pin -  
*/
int setMux(int mux, int channel) {

  // declare local array for control pins (S0-S3) and variable for SIG pin
  int controlPin[4];
  int sig_pin;

  // set the correct values for each MUX's control pins and SIG pins
  if (mux == 1) {
    controlPin[0] = mux1_s0;
    controlPin[1] = mux1_s1;
    controlPin[2] = mux1_s2;
    controlPin[3] = mux1_s3;
    sig_pin = mux1_sig;
  } else if (mux == 2) {
    controlPin[0] = mux2_s0;
    controlPin[1] = mux2_s1;
    controlPin[2] = mux2_s2;
    controlPin[3] = mux2_s3;
    sig_pin = mux2_sig;
  }

  // 2D integer array for MUX channels
  // arrayName[row][column]
  int muxChannel[16][4] = {
    { 0, 0, 0, 0 },  //channel 0
    { 1, 0, 0, 0 },  //channel 1
    { 0, 1, 0, 0 },  //channel 2
    { 1, 1, 0, 0 },  //channel 3
    { 0, 0, 1, 0 },  //channel 4
    { 1, 0, 1, 0 },  //channel 5
    { 0, 1, 1, 0 },  //channel 6
    { 1, 1, 1, 0 },  //channel 7
    { 0, 0, 0, 1 },  //channel 8
    { 1, 0, 0, 1 },  //channel 9
    { 0, 1, 0, 1 },  //channel 10
    { 1, 1, 0, 1 },  //channel 11
    { 0, 0, 1, 1 },  //channel 12
    { 1, 0, 1, 1 },  //channel 13
    { 0, 1, 1, 1 },  //channel 14
    { 1, 1, 1, 1 }   //channel 15
  };

  // loop through the 4 SIG
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  return sig_pin;
}


/*
setupWiFiAP() - sets up the WiFi Access Point and includes error handling 
Parameters -  none (uses global variable status)
Return - none (function return type: void)
*/
void setupWiFiAP() {

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  // check firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  WiFi.config(IPAddress(192, 48, 56, 2));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
      ;
  }

  // wait 10 seconds for connection:
  delay(10000);
}
