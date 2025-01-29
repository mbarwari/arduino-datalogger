/*
Mux 1
  Sig = pin 18
  S3 = pin 20
  S2 = pin 21
  S1 = pin 26
  S0 = pin 33

Mux 2
  Sig = pin 19
  S3 = pin 39
  S2 = pin 40
  S1 = pin 41
  S0 = pin 42

ESP32-S2-DevKitM-1-N4R2
*/

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
int tempCount, pressureCount, noneCount;

// MUX1 control pins, S0-S3 (digital pins)
int mux1S0 = 33;
int mux1S1 = 26;
int mux1S2 = 21;
int mux1S3 = 20;

// MUX2 control pins, S0-S3 (digital pins)
int mux2S0 = 42;
int mux2S1 = 41;
int mux2S2 = 40;
int mux2S3 = 39;

// MUX1 and MUX2 signal pins, SIG (analog pins)
int mux1Sig = 18;
int mux2Sig = 19;


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

}


// loop() function runs continuously after the setup() function completes
void loop() {

  tempCount = 0;
  pressureCount = 0;
  noneCount = 0;
  // Loop through and read, convert, and display all 16 channels from MUX 1
  for (int i = 0; i < 16; i++) {
    tempCount += 1;
    int sig = setMux(1, i);
    float temp = readThermistor(sig);

    Serial.print(waqt);
    Serial.print(" ");
    Serial.print("T");
    Serial.print(tempCount);
    Serial.print(" ");
    Serial.println(temp);
  }

  // Loop through and read, convert, and display all 16 channels from MUX 2
  for (int i = 0; i < 16; i++) {
    if (i < 12) {
      // MUX channels 0-11 have thermistors connected so
      // read, convert, and display the temperature (in Celsius) from the thermistors
      tempCount += 1;
      int sig = setMux(2, i);
      float temp = readThermistor(sig);

      Serial.print(waqt);
      Serial.print(" ");
      Serial.print("T");
      Serial.print(tempCount);
      Serial.print(" ");
      Serial.println(temp);

    } else if (i < 15) {
      // MUX channels 12-15 have pressure sensors connected so
      // read, convert, and display the pressure (in PSI) from the pressure sensors
      pressureCount += 1;
      int sig = setMux(2, i);
      float pressure = readPressure(sig);

      Serial.print(waqt);
      Serial.print(" ");
      Serial.print("P");
      Serial.print(pressureCount);
      Serial.print(" ");
      Serial.println(pressure);
    }
        
  }

  // delay for 5 seconds
  delay(5000);
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

