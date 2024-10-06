/* 
Honeywell ABPDANT015PGAA5 pressure sensor 

Minimum pressure = 0 PSI
Maximum pressure = 15 PSI 

Sensor       Arduino
------------------------- 
1.GND        ->   GND
2.NC         ->   NC
3.Vout       ->   A2 (or any analog input pin, but make sure to change pressureSensorPin value)
4.NC         ->   NC
5.NC         ->   NC
6.Vsupply    ->   3.3V or 5V
*/

// Define the analog input pin for the sensor
const int pressureSensorPin = A2;

// Sensor specifications
const float Vsupply = 3.3;    // Supply voltage (3.3V or 5V)
const float Pmin = 0.0;       // Minimum pressure in PSI
const float Pmax = 15.0;      // Maximum pressure in PSI

const int sampleSize = 10;   // Sample size
const int decimalPlaces = 3;  // Decimal places for Serial.print()


void setup() {
  //analogReadResolution(14);  // Increase dafault 10-bit ADC resolution to 14-bit
  Serial.begin(9600);  // Initialize serial communication

  // sets the reference voltage for analog-to-digital conversion to an external source for accuracy
  //analogReference(AR_EXTERNAL);
}

void loop() {
  delay(50);
  int total = 0;

  for (int i = 0; i < sampleSize; i++) {
    // Read the voltage from the pressure sensor
    // analogRead() returns an integer between 0-1023 (or 0-16383 if resolution changed to 14-bit)
    int sensorValue = analogRead(pressureSensorPin);
    total += sensorValue;
    delay(0);
  }

  // Find the average of the sensor value readings 
  int averageSensorValue = total / sampleSize;

  // Print the analogread val to the serial monitor
  Serial.print(averageSensorValue);
  Serial.print(" ");

  // Convert the analog reading to voltage (0-3.3V or 0-5V)
  //float outputVoltage = averageSensorValue * (Vsupply / 16383.0);
  float outputVoltage = averageSensorValue * (Vsupply / 1023.0);

  // Print the voltage to the serial monitor
  Serial.print(outputVoltage, decimalPlaces);
  Serial.print(" ");

  //pressureApplied = ((Pmax - Pmin) / (0.8 * Vsupply)) * (Output - 0.10 * Vsupply) + Pmin 
  //pressureApplied = ((15) / (0.8 * Vsupply)) * (Output - 0.10 * Vsupply)
  float pressureApplied = (15 / (0.8 * Vsupply)) * (outputVoltage - 0.10 * Vsupply);

  // Print the pressure to the serial monitor
  Serial.println(pressureApplied, decimalPlaces);


  // Delay for 100 miliseconds before the next set of readings
  delay(1000);
}
