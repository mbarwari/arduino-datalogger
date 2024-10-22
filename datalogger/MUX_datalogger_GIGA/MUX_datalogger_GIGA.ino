/*
MUX datalogger  

Temperature in Celsius 
Flow rate in ml/min 
Pressure in PSI 
Voltage in V
Current in mA
*/

// include necessary libraries
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "WiFi.h"
#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
#include <Adafruit_INA260.h>
#include <mbed.h>


// PWM code SFE 
PinName pin = digitalPinToPinName(D10);
mbed::PwmOut* pwm = new mbed::PwmOut(pin);

// Define the flow sensor objects 
SensirionI2cSf06Lf flowSensorA;
SensirionI2cSf06Lf flowSensorB;

// Define current sensor objects and I2C addresses 
Adafruit_INA260 pumpINA260 = Adafruit_INA260();
Adafruit_INA260 peltierINA260 = Adafruit_INA260();
uint8_t pumpI2CAddress = 0x40;
uint8_t peltierI2CAddress = 0x41;

// Pressure sensor specifications
const float vSupply = 3.3;  // Supply voltage
const float pMin = 0.0;     // Minimum pressure in PSI
const float pMax = 15.0;    // Maximum pressure in PSI

const int sampleSize = 10;   // Sample size
const int decimalPlaces = 3;  // Decimal places for Serial.print()

// Thermistor related macros
#define NUM_SAMPLES 10
#define SERIES_RESISTOR 10000
#define B_COEFFICIENT 3895
#define THERMISTOR_NOMINAL 10000
#define TEMPERATURE_NOMINAL 25

// General global variables
unsigned long waqt;
int tempCount, pressureCount, flowCount, currentCount, noneCount;

// WIFI related global variables, macros, and object
#define SECRET_SSID "pigTrial"
#define SECRET_PASS "123456789"
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

// MUX1 control pins, S0-S3 (digital pins)
int mux1S0 = 9;
int mux1S1 = 8;
int mux1S2 = 7;
int mux1S3 = 6;

// MUX2 control pins, S0-S3 (digital pins)
int mux2S0 = 5;
int mux2S1 = 4;
int mux2S2 = 3;
int mux2S3 = 2;

// MUX1 and MUX2 signal pins, SIG (analog pins)
int mux1Sig = 0;
int mux2Sig = 1;

// Micro SD card module chip select 
const int chipSelect = 50;
File myFile;
String dataString = "";  


// setup() function is called once when microcontroller starts
void setup() {

  // set the control pins of MUX1 (S0-S3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from
  pinMode(mux1S0, OUTPUT);
  pinMode(mux1S1, OUTPUT);
  pinMode(mux1S2, OUTPUT);
  pinMode(mux1S3, OUTPUT);

  // set the initial state of MUX1 (S0-S3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.
  digitalWrite(mux1S0, LOW);
  digitalWrite(mux1S1, LOW);
  digitalWrite(mux1S2, LOW);
  digitalWrite(mux1S3, LOW);

  // set the control pins of MUX2 (S0-S3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from
  pinMode(mux2S0, OUTPUT);
  pinMode(mux2S1, OUTPUT);
  pinMode(mux2S2, OUTPUT);
  pinMode(mux2S3, OUTPUT);

  // set the initial state of MUX2 (S0-S3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.  digitalWrite(mux2_s0, LOW);
  digitalWrite(mux2S0, LOW);
  digitalWrite(mux2S1, LOW);
  digitalWrite(mux2S2, LOW);
  digitalWrite(mux2S3, LOW);

  // starts the serial communication at a baud rate of 9600
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }

  pwm->period(1.0 / 60.0);  // Initial period for 60 Hz frequency
  pwm->write(0.5f);         // 50% duty cycle

  Serial.println("I2C setup: "); 

  // change the flow sensor I2C addresses and ......
  flowSensorSetUp();

  // set up the current sensors 
  //currentSensorSetUp();

  Serial.println("WIFI setup: "); 

  // set up the WiFi Access Point
  //setupWiFiAP();

  // start the WiFi server
  // the server will listen for incoming connections on port 80 (the default port for HTTP)
  //server.begin();

  //setupSDCard();
}


// loop() function runs continuously after the setup() function completes
void loop() {

  // compare previous WIFI status to current WIFI status
  /*
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
  */

  // listen for incoming clients
  //WiFiClient client = server.available();

  //if (client) {
    // if you get a client

    tempCount = 0;
    pressureCount = 0;
    flowCount = 0;
    currentCount = 0;
    noneCount = 0;
    // Loop through and read, convert, and display all 16 channels from MUX 1
    for (int i = 0; i < 16; i++) {
      tempCount += 1;
      int sig = setMux(1, i);
      float temp = readThermistor(sig);

      dataString += String(waqt);
      dataString += " ";
      dataString += "T";
      dataString += String(tempCount);
      dataString += " ";
      dataString += String(temp);
      dataString += "\n";


    //client.print(dataString);
    Serial.print(dataString);
      /*
      myFile = SD.open("datalogger.txt", FILE_WRITE);
      if (myFile) {
        myFile.println(dataString);
        myFile.close();
        }
      */
      dataString = ""; 
    }

    // Loop through and read, convert, and display all 16 channels from MUX 2
    for (int i = 0; i < 16; i++) {
      if (i < 12) {
        // MUX channels 0-11 have thermistors connected so
        // read, convert, and display the temperature (in Celsius) from the thermistors
        tempCount += 1;
        int sig = setMux(2, i);
        float temp = readThermistor(sig);

        dataString += String(waqt);
        dataString += " ";
        dataString += "T";
        dataString += String(tempCount);
        dataString += " ";
        dataString += String(temp);
        dataString += "\n";


        //client.print(dataString);
        Serial.print(dataString);

        /*
        myFile = SD.open("datalogger.txt", FILE_WRITE);
        if (myFile) {
          myFile.println(dataString);
          myFile.close();
          }
        */
        dataString = ""; 

      } else {
        // MUX channels 12-15 have pressure sensors connected so
        // read, convert, and display the pressure (in PSI) from the pressure sensors
        pressureCount += 1;
        int sig = setMux(2, i);
        float pressure = readPressure(sig);

        dataString += String(waqt);
        dataString += " ";
        dataString += "P";
        dataString += String(pressureCount);
        dataString += " ";
        dataString += String(pressure);
        dataString += "\n";


        //client.print(dataString);
        Serial.print(dataString);
        /*
        myFile = SD.open("datalogger.txt", FILE_WRITE);
        if (myFile) {
          myFile.println(dataString);
          myFile.close();
          }
        */
        dataString = ""; 
      } 
    }

    // read and display the flow rate (in ml/min) from the flow sensors
    //printFlowSensorOutput(flowSensorA, client);
    //printFlowSensorOutput(flowSensorB, client);

    printFlowSensorOutput(flowSensorA);
    printFlowSensorOutput(flowSensorB);

    // read and display the voltage and current of the pump and peltier from the current sensors
    //printCurrentSensorOutput(pumpINA260, client); 
    //printCurrentSensorOutput(peltierINA260, client); 


    // delay for 5 seconds
    delay(5000);
  //}
}


/*
printCurrentSensorOutput(Adafruit_INA260& sensor, WiFiClient& client) -  
Parameters - none
Return - none (function return type: void) 
*/
//void printCurrentSensorOutput(Adafruit_INA260& sensor, WiFiClient& client) {
void printCurrentSensorOutput(Adafruit_INA260& sensor) {

  currentCount += 1;
  float v = sensor.readBusVoltage()/1000.0;
  float c = sensor.readCurrent();

  dataString += String(waqt);
  dataString += " ";
  dataString += "V";
  dataString += String(currentCount);
  dataString += " ";
  dataString += String(v);
  dataString += " V"; 
  dataString += "\n";


  dataString += String(waqt);
  dataString += " ";
  dataString += "C";
  dataString += String(currentCount);
  dataString += " ";
  dataString += String(c);
  dataString += " mA"; 
  dataString += "\n";


  //client.print(dataString);
  Serial.print(dataString);

  /*
  myFile = SD.open("datalogger.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(dataString);
    myFile.close();
  }
  */
  dataString = "";   
}


/*
printFlowSensorOutput(SensirionI2cSf06Lf& sensor, WiFiClient& client) - 
Parameters - 
  SensirionI2cSf06Lf& sensor
  WiFiClient& client
Return - none (function return type: void) 
*/
//void printFlowSensorOutput(SensirionI2cSf06Lf& sensor, WiFiClient& client) {
void printFlowSensorOutput(SensirionI2cSf06Lf& sensor) {
  float aFlow = 0.0;
  float aTemperature = 0.0;
  uint16_t aSignalingFlags = 0u;
  delay(20);
  sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, aFlow, aTemperature, aSignalingFlags);

  tempCount += 1;
  dataString += String(waqt);
  dataString += " ";
  dataString += "T";
  dataString += String(tempCount);
  dataString += " ";
  dataString += String(aTemperature);
  dataString += "\n";

  flowCount += 1;
  dataString += String(waqt);
  dataString += " ";
  dataString += "F";
  dataString += String(flowCount);
  dataString += " ";
  dataString += String(aFlow);
  dataString += "\n";


  //client.print(dataString);
  Serial.print(dataString);
  /*
  myFile = SD.open("datalogger.txt", FILE_WRITE);
  if (myFile) {
    myFile.println(dataString);
    myFile.close();
  }
  */
  dataString = "";
}


/*
readPressure(int sigPin) -  
Parameters - 
  int sigPin
Return - pressureApplied (function return type: float) 
*/
float readPressure(int sigPin) {
  delay(50);
  int total = 0;

  for (int i = 0; i < sampleSize; i++) {
    // Read the voltage from the pressure sensor
    // analogRead() returns an integer between 0-1023 (or 0-16383 if resolution changed to 14-bit)
    int sensorValue = analogRead(sigPin);
    total += sensorValue;
    delay(0);
  }

  // Find the average of the sensor value readings 
  int averageSensorValue = total / sampleSize;

  // Convert the analog reading to voltage (0-3.3V or 0-5V)
  //float outputVoltage = averageSensorValue * (Vsupply / 16383.0);
  float outputVoltage = averageSensorValue * (vSupply / 1023.0);

  //pressureApplied = ((Pmax - Pmin) / (0.8 * Vsupply)) * (Output - 0.10 * Vsupply) + Pmin 
  //pressureApplied = ((15) / (0.8 * Vsupply)) * (Output - 0.10 * Vsupply)
  float pressureApplied = (15 / (0.8 * vSupply)) * (outputVoltage - 0.10 * vSupply);

  return pressureApplied;
}


/*
readThermistor(int sigPin) - converts thermistor value to temperature using the Steinhart-Hart equation
Parameters - 
  int sigPin -   
Return - (function return type: int)
  float steinhart - temperature in celsius of the thermistor  
*/
float readThermistor(int sigPin) {

  // read the value at the SIG pin
  waqt = millis() / 1000;
  uint16_t val[NUM_SAMPLES] = { 0 };
  delay(50);
  for (uint8_t j = 0; j < NUM_SAMPLES; j++) {  // take N samples in a row, with a slight delay
    val[j] = analogRead(sigPin);
    delay(0);
  }

  float avgval = 0;
  for (int k = 0; k < NUM_SAMPLES; k++) {
    avgval += val[k];
  }
  avgval = SERIES_RESISTOR / (1023 / (avgval / NUM_SAMPLES) - 1);
  float steinhart = 1 / ((log(avgval / THERMISTOR_NOMINAL)) / B_COEFFICIENT + 1.0 / (TEMPERATURE_NOMINAL + 273.15)) - 273.5;  // (R/Ro)

  return steinhart;
}


/*
setupSDCard() -  
Parameters - none
Return - none (function return type: void) 
*/
void setupSDCard(){
  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed.");
  }
  else{
    Serial.println("initialization done.");
  }
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
Parameters - none
Return - none (function return type: void) 
*/
void flowSensorSetUp() {

  Wire.begin(); //SDA & SDL
  Wire1.begin(); //SDA1 & SDL1

  // Initialize first sensor
  Serial.println("Initialising flow sensor A");
  flowSensorA.begin(Wire, SLF3C_1300F_I2C_ADDR_08);
  
  // Initialize second sensor
  Serial.println("Initialising flow sensor B");
  flowSensorB.begin(Wire1, SLF3C_1300F_I2C_ADDR_08);

  delay(100);
  flowSensorA.startH2oContinuousMeasurement();
  flowSensorB.startH2oContinuousMeasurement();
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
  int sigPin;

  // set the correct values for each MUX's control pins and SIG pins
  if (mux == 1) {
    controlPin[0] = mux1S0;
    controlPin[1] = mux1S1;
    controlPin[2] = mux1S2;
    controlPin[3] = mux1S3;
    sigPin = mux1Sig;
  } else if (mux == 2) {
    controlPin[0] = mux2S0;
    controlPin[1] = mux2S1;
    controlPin[2] = mux2S2;
    controlPin[3] = mux2S3;
    sigPin = mux2Sig;
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

  return sigPin;
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