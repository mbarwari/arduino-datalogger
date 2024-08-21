#define DACPIN A0
#define THERMISTORPIN1 A1
#define THERMISTORPIN2 A2
#define THERMISTORPIN3 A3
#define THERMISTORNOMINAL 10000  
#define TEMPERATURENOMINAL 25 
#define NUMSAMPLES 10
#define BCOEFFICIENTL 3895
#define BCOEFFICIENTA 3895
#define SERIESRESISTOR 10000 


//This portion is to test the peltier buck converter with serial monitor

//String stringa = String(1);
int CompSetPeltier;
//#define PWMPeltier 3 //Choosing pin randomly for output for Peltier
//#define PWMPump 5 //Choosing pin randomly for output for pump

#include <Adafruit_INA260.h>
#include <Wire.h>
#include <pwm.h>

//Needed for the flow sensor
#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
SensirionI2cSf06Lf flow_sensor;

PwmOut pwm(D5);

Adafruit_INA260 ina260_peltier = Adafruit_INA260();

//Creating the output frequency using the DAC pin
//#include "analogWave.h" // Include the library for analog waveform generation
//analogWave wave(DAC);   // Create an instance of the analogWave class, using the DAC pin
int freq = 10;  // in hertz, change accordingly, randomly set at 0 here, can increase below

#include <PID_v1.h> //Initializing the PID loop

double SetpointPeltier, InputPeltierPID, OutputPeltierPID; //Generating the input variables for the peltier plate cooling PID -- Setpoint is goal; input is what is detected; output is PID output 0-255
double Kp_peltier=17, Ki_peltier=3, Kd_peltier=2; //Specify the initial tuning parameters. Ignoring Kd since not used often. -- watch youtube video: https://www.youtube.com/watch?v=IB1Ir4oCP5k
PID myPID_peltier(&InputPeltierPID, &OutputPeltierPID, &SetpointPeltier, Kp_peltier, Ki_peltier, Kd_peltier, DIRECT, REVERSE); //By default, PID "warms" up, so we have to reverse it -- unknown what DIRECT is for

//This PID is to make sure that the water block is adequtely cooling down. WB stands for water block
double SetpointWB, InputWB, OutputWB; // setpoint is desired temp, input is current temp, output is 0-255
double Kp_WB=15, Ki_WB=10, Kd_WB=0;
PID myPID_WB(&InputWB, &OutputWB, &SetpointWB, Kp_WB, Ki_WB, Kd_WB, DIRECT, REVERSE);


//These are parameters that can be adjusted for temperature cutoffs -- TempIdeals are equivalent to Setpoint
int BrainTempIdeal = 10; //Ideal brain temperature, currently 12 for testing purposes (should be 25)
int WBTempMax = 26; //The maximum peltier plate temperature, currently 25 for testing purposes (should be 40)
int WBTempIdeal = 20; //The ideal temperature of the WB, currently 22 for testing purposes (should be 35)

unsigned long waqt;
uint16_t samplesa1[NUMSAMPLES];
uint16_t samplesa2[NUMSAMPLES];
uint16_t samplesa3[NUMSAMPLES];



void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
 while (!Serial) { delay(10); }
  Wire.begin();
 pwm.begin(100.0f, 50.0f);
  Serial.println("Adafruit INA260 Test");

  if (!ina260_peltier.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
  } 

  Serial.println("Found INA260 chip");
  //analogReference(EXTERNAL);
  analogReference(AR_EXTERNAL); //This needs to be wired to AREF at 3.3V, and this is the powersupply to the thermestimors
  pinMode(DACPIN, OUTPUT);
  //wave.square(freq);
  

  SetpointPeltier = BrainTempIdeal;
  //turn the PID on
  myPID_peltier.SetMode(AUTOMATIC);
  //myPID_pump.SetMode(AUTOMATIC);

  SetpointWB = WBTempIdeal;
  myPID_WB.SetMode(AUTOMATIC);

//Initializing flow sensor
  flow_sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);

  delay(100);
  flow_sensor.startH2oContinuousMeasurement();

}

void loop() {
  // put your main code here, to run repeatedly:

uint8_t i;
  waqt = millis()/1000;
   for (i=0; i< NUMSAMPLES; i++) {          // take N samples in a row, with a slight delay
   samplesa1[i] = analogRead(THERMISTORPIN1);
   samplesa2[i] = analogRead(THERMISTORPIN2);
   //samplesa3[i] = analogRead(THERMISTORPIN3);
   delay(100);
  }
  
  float avga1;
  float avga2;
  float avga3;

 // average all the samples out
  avga1 = 0;
  avga2 = 0;
  avga3 = 0;

  for (i=0; i< NUMSAMPLES; i++) {
     avga1 += samplesa1[i];
     avga2 += samplesa2[i];
     avga3 += samplesa3[i];
  }
  avga1 = SERIESRESISTOR / (1023 /(avga1/NUMSAMPLES) - 1);
  avga2 = SERIESRESISTOR / (1023 /(avga2/NUMSAMPLES) - 1);
  avga3 = avga3/NUMSAMPLES;

  //float flow3 = 100 * (avga3 - 0.045);

  float steinharta1;
  float steinharta2;
  //float steinharta3;


   // (R/Ro)
  steinharta1 = 1/((log(avga1 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta2 = 1/((log(avga2 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
 // steinharta3 = 1/((log(avga3 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;


  //Serial.print(waqt);
  Serial.print("P_T:");
  Serial.print(steinharta1);
  Serial.print("\t");
 // Serial.print(waqt);
  //Serial.println();
 //Serial.print(waqt);
  Serial.print("WB_T:");
  Serial.print(steinharta2);
  Serial.print("\t");
  //Serial.println();
  //Serial.print(avga3);
  //Serial.print(",");
  //Serial.print(flow3);
  //Serial.print(",");

  /*
  //Serial.print(waqt);
  Serial.print(" Block Inlet Temp ");
  Serial.print(steinharta3);
  Serial.print(",");
*/



//analogWrite(PWMPeltier, 250);

//Creating PID to always keep the peltier running

  float braintemp_atm;
  braintemp_atm = steinharta1; //This is the current brain temperature
  InputPeltierPID = braintemp_atm;
  myPID_peltier.Compute();
  //OutputPeltierPID = 0;

if (Serial.available()>0) {

//stringa = Serial.readString();
//CompSetPeltier = stringa.toInt();

}

  analogWrite(DACPIN, 255-OutputPeltierPID);
 //analogWrite(DACPIN, 255-CompSetPeltier);
  Serial.print("P_OUT(div10):");
  Serial.print(OutputPeltierPID/10);
  //Serial.print(CompSetPeltier/10);
  Serial.print("\t");
  


float WBTemp_atm;
WBTemp_atm = steinharta2;


if (WBTemp_atm >= WBTempIdeal) {
 InputWB = WBTemp_atm;
 myPID_WB.Compute();
 freq = (OutputWB * (60)) / 255;
 Serial.print("WB_OUT(div10):");
 Serial.print(OutputWB/10);
 pwm.period_raw(50000000/freq); ///50000000 might need some adjustmnet, it is rough ballpark
 pwm.pulse_perc(50.0f);
 Serial.print("\t");
 Serial.print("FREQ:");
 Serial.print(freq);


}
/*else {freq = 0;

Serial.print("WB_OUT(div10):");
 Serial.print(10/10);
 pwm.period_raw(50000000/freq); ///50000000 might need some adjustmnet, it is rough ballpark
 pwm.pulse_perc(50.0f);
 Serial.print("\t");
 Serial.print("FREQ:");
 Serial.print(freq); */



float peltier_current;
peltier_current = ina260_peltier.readCurrent()/1000;


if (peltier_current >= 2) {

OutputPeltierPID = 0;

}


 Serial.print("\t");
  Serial.print("P_Curr:");
  Serial.print(peltier_current);
 
 
  Serial.print("\t");
  Serial.print("P_Volt");
  Serial.print(ina260_peltier.readBusVoltage()/1000);

 //Getting info from flow sensor
 float aFlow = 0.0;
  float aTemperature = 0.0;    
  uint16_t aSignalingFlags = 0u;
 flow_sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, aFlow, aTemperature, aSignalingFlags);
  Serial.print("\t");
  Serial.print("aFlow: ");
  Serial.println(aFlow);
   Serial.print("aTemperature: ");
  Serial.print(aTemperature);
  Serial.print("\t");
  Serial.print("aSignalingFlags: ");
  Serial.println(aSignalingFlags);



//delay (400);
}

/*float readCurrent() {
  Adafruit_I2CRegister current =
      Adafruit_I2CRegister(i2c_dev, INA260_REG_CURRENT, 2, MSBFIRST);
  return (int16_t)current.read() * 1.25;
} */
