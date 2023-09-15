/*
Datalogger program 

DATE: 8/12/2023
NOTE: comment out specified multiline comment blocks for 2 mux 
*/

#include <SD.h>
#include "WiFiS3.h"

#define NUMSAMPLES 10
#define SERIESRESISTOR 10000
#define BCOEFFICIENT 3895
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
unsigned long waqt;

#define SECRET_SSID "pigTrial"
#define SECRET_PASS "123456789"
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;           // your network key index number (needed only for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);

// Mux1 control pins
int mux1_s0 = 9;
int mux1_s1 = 8;
int mux1_s2 = 7;
int mux1_s3 = 6;

// Mux2 control pins
int mux2_s0 = 5;
int mux2_s1 = 4;
int mux2_s2 = 3;
int mux2_s3 = 2;

// Mux in "SIG" pin
int mux1_sig = 0;
int mux2_sig = 1;


void setup() {
  pinMode(mux1_s0, OUTPUT);
  pinMode(mux1_s1, OUTPUT);
  pinMode(mux1_s2, OUTPUT);
  pinMode(mux1_s3, OUTPUT);

  digitalWrite(mux1_s0, LOW);
  digitalWrite(mux1_s1, LOW);
  digitalWrite(mux1_s2, LOW);
  digitalWrite(mux1_s3, LOW);

  pinMode(mux2_s0, OUTPUT); 
  pinMode(mux2_s1, OUTPUT); 
  pinMode(mux2_s2, OUTPUT); 
  pinMode(mux2_s3, OUTPUT); 

  digitalWrite(mux2_s0, LOW);
  digitalWrite(mux2_s1, LOW);
  digitalWrite(mux2_s2, LOW);
  digitalWrite(mux2_s3, LOW);

  Serial.begin(9600);
  
  analogReference(AR_EXTERNAL);

  setWiFi();

  server.begin();
}

void loop() {

  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  
  WiFiClient client = server.available();   // listen for incoming clients
  
  if (client){      
    // if you get a client

    // Loop through and read all 16 values from mux 1
    for (int i = 0; i < 16; i++) {
        float temp = readMux(i, 1);

        /*
        Serial.print(waqt);
        Serial.print(" C");
        Serial.print(i);
        Serial.print(" ");
        Serial.println(temp);
        */

        client.print(waqt);
        client.print(" C");
        client.print(i);
        client.print(" ");
        client.println(temp);
    }

    // Loop through and read all 16 values from mux 2
    for(int i = 0; i < 16; i ++){
        float temp = readMux(i, 2);

        /*
        Serial.print(waqt);
        Serial.print(" C");
        Serial.print(i+16);
        Serial.print(" ");
        Serial.println(temp);
        */
        
        client.print(waqt);
        client.print(" C");
        client.print(i+16);
        client.print(" ");
        client.println(temp);
    }

  }

  delay(1000);
}


float readMux(int channel, int mux) {

  int controlPin[4];
  int sig_pin;

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

  int muxChannel[16][4] = {
    { 0, 0, 0, 0 },  //channel 0
    { 0, 0, 0, 0 },  //channel 1
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
    //Serial.println(val[j]);

    val[j] = analogRead(sig_pin);
    //Serial.println(val[j]);
    delay(0);
  }
  float avgval = 0;
  for (int k = 0; k < NUMSAMPLES; k++) {
    avgval += val[k];
    //Serial.println(avgval);
    //Serial.println(avgval/NUMSAMPLES);
  }
  avgval = SERIESRESISTOR / (1023 / (avgval / NUMSAMPLES) - 1);
  //Serial.println(avgval);
  float steinhart = 1 / ((log(avgval / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.5;  // (R/Ro)
  return steinhart;
}

void setWiFi() {

  Serial.println("Access Point Web Server");

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

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

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}
