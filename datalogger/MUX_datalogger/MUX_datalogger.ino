/*
MUX datalogger  
Date: 6/19/2024

Temperature in Celsius 
Pressure in PSI
Flow rate in ml/min 

*/

// include necessary libraries
#include "WiFiS3.h"

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

// MUX1 control pins (s0-s3)
int mux1_s0 = 9;
int mux1_s1 = 8;
int mux1_s2 = 7;
int mux1_s3 = 6;

// MUX2 control pins (s0-s3)
int mux2_s0 = 5;
int mux2_s1 = 4;
int mux2_s2 = 3;
int mux2_s3 = 2;

// MUX1 and MUX2 signal pins (sig)
int mux1_sig = 0;
int mux2_sig = 1;


// setup() function is called once when microcontroller starts 
void setup() {

  // set the control pins of MUX1 (s0-s3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from 
  pinMode(mux1_s0, OUTPUT);
  pinMode(mux1_s1, OUTPUT);
  pinMode(mux1_s2, OUTPUT);
  pinMode(mux1_s3, OUTPUT);

  // set the initial state of MUX1 (s0-s3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.
  digitalWrite(mux1_s0, LOW);
  digitalWrite(mux1_s1, LOW);
  digitalWrite(mux1_s2, LOW);
  digitalWrite(mux1_s3, LOW);

  // set the control pins of MUX2 (s0-s3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from 
  pinMode(mux2_s0, OUTPUT);
  pinMode(mux2_s1, OUTPUT);
  pinMode(mux2_s2, OUTPUT);
  pinMode(mux2_s3, OUTPUT);

  // set the initial state of MUX2 (s0-s3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.  digitalWrite(mux2_s0, LOW);
  digitalWrite(mux2_s0, LOW);
  digitalWrite(mux2_s1, LOW);
  digitalWrite(mux2_s2, LOW);
  digitalWrite(mux2_s3, LOW);

  // starts the serial communication at a baud rate of 9600 
  Serial.begin(9600);

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

    // Loop through and read, convert, and display all 16 channels from MUX 1
    for (int i = 0; i < 16; i++) {
      float temp = readMux(i, 1);

      client.print(waqt);
      client.print(" ");
      client.print("T");
      client.print(i);
      client.print(" ");
      client.println(temp);
    }

    // Loop through and read, convert, and display all 16 channels from MUX 2
    for (int i = 0; i < 16; i++) {
      float temp = readMux(i, 2);

      client.print(waqt);
      client.print(" ");
      client.print("T");
      client.print(i + 16);
      client.print(" ");
      client.println(temp);
    }

    // delay for 5 seconds
    delay(5000);

  }

}


float readMux(int channel, int mux) {

  // declare local array and variable for control pins (s0-s3) and sig pin
  int controlPin[4];
  int sig_pin;


  // set the correct values for the control pins and sig pins for each MUX
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

  // loop through the 4 sig
  for (int i = 0; i < 4; i++) {
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  // read the value at the SIG pin
  uint8_t j = 0;
  waqt = millis() / 1000;
  uint16_t val[NUMSAMPLES] = { 0 };
  delay(50);
  for (j = 0; j < NUMSAMPLES; j++) {  // take N samples in a row, with a slight delay
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
