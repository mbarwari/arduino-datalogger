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

Add 0.001 uf capacitor between Vout and GND
Add 0.1 uf capacitor between Vsupply and GND 

*/

// Define the analog input pin for the sensor
const int pressureSensorPin = A0;

// Sensor specifications
const float Vsupply = 5.0; // Supply voltage
const float Pmin = 0; // Minimum pressure in PSI
const float Pmax = 15; // Maximum pressure in PSI
const float Vmin = 0.10 * Vsupply; // Minimum output voltage
const float Vmax = 0.90 * Vsupply; // Maximum output voltage

void setup() {
  analogReadResolution(14); // Increase dafault 10-bit ADC resolution to 14-bit 
  Serial.begin(9600); // Initialize serial communication

}

void loop() {
  // Read the voltage from the pressure sensor
  // analogRead() returns an integer between 0-16383
  int sensorValue = analogRead(pressureSensorPin); 
  
  // Print the analogRead value to the serial monitor
  Serial.print("analogRead value: ");
  Serial.println(sensorValue);

  // Convert the analog reading to voltage (0-5V)
  float outputVoltage = sensorValue * (5.0  / 16383.0);

  // Print the voltage to the serial monitor
  Serial.print("Output Voltage: ");
  Serial.print(outputVoltage);
  Serial.println(" V");
  
  /*
  // pressureApplied = ((Pmax - Pmin) / (Vmax - Vmin)) * (outputVoltage - Vmin) + Pmin 
  float pressureApplied = ((Pmax - Pmin) / (Vmax - Vmin)) * (outputVoltage - Vmin) + Pmin;
  */ 

  // pressureApplied = 15/(0.8*5)*(Vout-0.05) + 0 
  float pressureApplied = 15/(0.8*5)*(outputVoltage-0.05) + 0;

  // Print the pressure to the serial monitor
  Serial.print("Pressure Applied: ");
  Serial.print(pressureApplied);
  Serial.println(" PSI");

  // Delay for a short period before the next reading
  delay(1000);

}




