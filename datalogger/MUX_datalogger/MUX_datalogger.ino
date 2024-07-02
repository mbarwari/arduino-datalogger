/*
MUX datalogger  
Date: 7/2/2024

Temperature in Celsius 
Flow rate in ml/min 
Pressure in PSI 

*/

// include necessary libraries
#include "WiFiS3.h"
#include <SensirionI2cSf06Lf.h>

SensirionI2cSf06Lf sensor;

// Sensor specifications
const float Vsupply = 5.0;   // Supply voltage
const float Pmin = 0;        // Minimum pressure in PSI
const float Pmax = 15;       // Maximum pressure in PSI

const int decimalPlaces = 3; // Decimal places for Serial.print() 

// thermistor related global variables and macros
#define NUMSAMPLES 10
#define SERIESRESISTOR 10000
#define BCOEFFICIENT 3895
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
unsigned long waqt;

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
  Wire.begin();
  sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);

  delay(100);
  sensor.startH2oContinuousMeasurement();

  // sets the reference voltage for analog-to-digital conversion to an external source for accuracy 
  analogReference(AR_EXTERNAL);

  // call setupWiFiAP() function which sets up the WiFi Access Point 
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

    int tempCount = 0; 
    int pressureCount = 0; 
    int flowCount = 0; 
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
      if(i < 12){
        tempCount += 1;
        int sig = setMux(2, i);
        float temp = readThermistor(sig);

        client.print(waqt);
        client.print(" ");
        client.print("T");
        client.print(tempCount);
        client.print(" ");
        client.println(temp);
      }
      else{
        pressureCount += 1;
        int sig = setMux(2, i);
        float pressure = readPressure(sig);
        
        client.print(waqt);
        client.print(" ");
        client.print("P");
        client.print(pressureCount);
        client.print(" ");
        client.println(pressure);
      }
    }

    float aFlow = 0.0;
    float aTemperature = 0.0;    
    uint16_t aSignalingFlags = 0u;
    delay(20);
    sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F, aFlow, aTemperature, aSignalingFlags);

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

    // delay for 5 seconds
    delay(5000);
  }
}


//todo 
float readPressure(int sig_pin) {
  
  delay(50);
  
  int sensorValue = analogRead(sig_pin);
  Serial.print(sensorValue, decimalPlaces);

  // Convert the analog reading to voltage (0-5V)
  //float outputVoltage = averageSensorValue * (5.0 / 16383.0);
  float outputVoltage = sensorValue * (5.0 / 1023.0);

  // Print the voltage to the serial monitor
  Serial.print(" ");
  Serial.print(outputVoltage, decimalPlaces);

  // pressureApplied = 15/(0.8*5)*(Vout-0.5) + 0
  float pressureApplied = 15 / (0.8 * 5) * (outputVoltage - 0.5) + 0;

  // Print the pressure to the serial monitor
  Serial.print(" ");
  Serial.println(pressureApplied, decimalPlaces);

  return pressureApplied; 
}
//todo 


/*
readThermistor(int sig_pin) - converts thermistor value to temperature using the Steinhart-Hart equation
Parameters - 
  int sig_pin -   
Return - (function return type: int)
  float steinhart - temperature in celsius of the thermistor  
*/
float readThermistor(int sig_pin){
  
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
    while (true);
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
    while (true);
  }

  // wait 10 seconds for connection:
  delay(10000);
}

