/*
This code works on the ESP32-S2-DevKitM-1-N4R2.

software resets the board 
*/

void setup() {
  // Start serial communication and give time for the connection to be established
  Serial.begin(9600);

  Serial.println("Reseting the board in 10 seconds");
}

void loop() {

  for (int i = 1; i < 11; i++) {
    delay(1000);  
    Serial.println(i);
  }

  // Trigger the software reset
  ESP.restart();
}