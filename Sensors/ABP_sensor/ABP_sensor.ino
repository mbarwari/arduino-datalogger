/* 
Honeywell ABPDANT015PGAA5 pressure sensor 

Minimum pressure = 0 PSI
Maximum pressure = 15 PSI 

Sensor       Arduino
------------------------- 
1.GND        ->   GND
2.NC         ->   NC
3.Vout       ->   A0 (or any analog input pin, but make sure to change pressureSensorPin value)
4.NC         ->   NC
5.NC         ->   NC
6.Vsupply    ->   5V

Reduce noise as per the datasheet: 
  Add 0.001 uf capacitor between Vout and GND
  Add 0.1 uf capacitor between Vsupply and GND 

*/

// Define the analog input pin for the sensor
const int pressureSensorPin = A2;

// Sensor specifications
const float Vsupply = 5.0;   // Supply voltage
const float Pmin = 0;        // Minimum pressure in PSI
const float Pmax = 15;       // Maximum pressure in PSI

const int sampleSize = 10;   // Sample size  
const int decimalPlaces = 3; // Decimal places for Serial.print() 


void setup() {
  //analogReadResolution(14);  // Increase dafault 10-bit ADC resolution to 14-bit
  Serial.begin(9600);        // Initialize serial communication
}

void loop() {

  int total = 0;

  for (int i = 0; i < sampleSize; i++) {
    // Read the voltage from the pressure sensor
    // analogRead() returns an integer between 0-16383
    int sensorValue = analogRead(pressureSensorPin);
    total += sensorValue;
    delay(100);
  }

  // Find the average of the sensor value readings 
  int averageSensorValue = total / sampleSize;

  // Convert the analog reading to voltage (0-5V)
  //float outputVoltage = averageSensorValue * (5.0 / 16383.0);
  float outputVoltage = averageSensorValue * (5.0 / 1023.0);

  // Print the voltage to the serial monitor
  Serial.print(outputVoltage, decimalPlaces);

  // pressureApplied = 15/(0.8*5)*(Vout-0.5) + 0
  float pressureApplied = 15 / (0.8 * 5) * (outputVoltage - 0.5) + 0;

  // Print the pressure to the serial monitor
  Serial.print(" ");
  Serial.println(pressureApplied, decimalPlaces);

  // Delay for 100 miliseconds before the next set of readings 
  delay(100);
}
