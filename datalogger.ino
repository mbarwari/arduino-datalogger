#include <SD.h>

#define NUMSAMPLES 10
#define SERIESRESISTOR 10000 
#define BCOEFFICIENT 3895
#define THERMISTORNOMINAL 10000 
#define TEMPERATURENOMINAL 25 
unsigned long waqt;


//Mux1 control pins
int mux1_s0 = 9;
int mux1_s1 = 8;
int mux1_s2 = 7;
int mux1_s3 = 6;

//Mux2 control pins
int mux2_s0 = 5;
int mux2_s1 = 4;
int mux2_s2 = 3;
int mux2_s3 = 2;

//Mux in "SIG" pin
int mux1_sig = 0;
int mux2_sig = 1;

//create file object (global variable)
File myFile;
String str_time;


void setup(){
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

  setMicroSDMod();
}


void loop(){

  //Loop through and read all 16 values from mux 1
  for(int i = 0; i < 16; i ++){
    float temp = readMux(i, 1);

    Serial.print(waqt);
    Serial.print(" ");
    Serial.print("C");
    Serial.print(i);
    Serial.print(" ");
    Serial.println(temp);

    myFile = SD.open("report.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(waqt);
      myFile.print(" ");
      myFile.print("C");
      myFile.print(i);
      myFile.print(" ");
      myFile.println(temp);
    }
    myFile.close();
  }

  //Loop through and read all 16 values from mux 2
  for(int i = 0; i < 16; i ++){
    float temp = readMux(i, 2);

    Serial.print(waqt);
    Serial.print(" ");
    Serial.print("C");
    Serial.print(i+16);
    Serial.print(" ");
    Serial.println(temp);

    myFile = SD.open("report.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(waqt);
      myFile.print(" ");
      myFile.print("C");
      myFile.print(i+16);
      myFile.print(" ");
      myFile.println(temp);
    }
    myFile.close();
  }

  delay(1000);
}


float readMux(int channel, int mux){

  int controlPin[4];
  int sig_pin; 
  
  if(mux == 1){
    controlPin[0] = mux1_s0;
    controlPin[1] = mux1_s1;
    controlPin[2] = mux1_s2;
    controlPin[3] = mux1_s3;
    sig_pin = mux1_sig; 
  }
  else if(mux == 2){
    controlPin[0] = mux2_s0;
    controlPin[1] = mux2_s1;
    controlPin[2] = mux2_s2;
    controlPin[3] = mux2_s3;
    sig_pin = mux2_sig; 
  }

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {0,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  //loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  uint8_t j = 0;
  waqt = millis()/1000;
  uint16_t val[NUMSAMPLES] = {0};
  delay(50);
  for (j=0; j< NUMSAMPLES; j++) {          // take N samples in a row, with a slight delay
    //Serial.println(val[j]);

    val[j] = analogRead(sig_pin);
    //Serial.println(val[j]);
    delay(0);
  }
  float avgval = 0;
  for (int k=0; k< NUMSAMPLES; k++) {
     avgval += val[k];
     //Serial.println(avgval);
     //Serial.println(avgval/NUMSAMPLES);
  }
  avgval = SERIESRESISTOR / (1023 /(avgval/NUMSAMPLES) - 1);
  //Serial.println(avgval);
  float steinhart = 1/((log(avgval / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.5;     // (R/Ro)
  return steinhart;
 
}

void setMicroSDMod() {

  if (SD.begin()) {
    Serial.print("SD Card initialization successful\n");
  } else {
    Serial.print("SD Card initialization failed\n");
    while (1);
  }
  
  myFile = SD.open("report.txt", FILE_WRITE);
  if (myFile) {
    Serial.print("Creating file...\n");
    myFile.println("______________________________________________________________________");
    myFile.println("Seconds, Channel, Temperature(degrees celsius)");
  } else {
    Serial.print("Error opeing file\n");
  }
  myFile.close();

}
