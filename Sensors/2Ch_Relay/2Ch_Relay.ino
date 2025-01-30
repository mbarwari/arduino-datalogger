/*
This code works on the ESP32-S2-DevKitM-1-N4R2.

Terminal blocks - NO1, COMM1, NC1, NO2, COMM2, NC2

NO - normally open 
COMM - common 
NC - normally close 

GND - GND
IN1 - GPIO pin
IN2 - GPIO pin
VCC - 5V

The normally open circuit(NO - COMM) on this relay uses active-low signal, so LOW = ON and HIGH = OFF. When the relay is powered, NO and COMM are connected. Red light turns on. 

NC and COMM are connected when the relay is not powered. By default this circuit is connected. Red light turns off. 
*/


// Pins IN1 and IN2 on the relay are connected to GPIO pins 45 and 42 
const int IN1 = 45;
const int IN2 = 42;

void setup() {
  
  // Start serial communication 
  Serial.begin(9600);

  // Set both relays as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Turn OFF relay 1 and relay 2 (HIGH = OFF)
  digitalWrite(IN1, HIGH);  
  digitalWrite(IN2, HIGH);

}

void loop() {

  // Turn ON relay 1 and relay 2 for 15 seconds (LOW = ON)
  Serial.println("Both relays are ON"); 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(15000); 

  // Turn OFF relay 1 and relay 2 for 15 seconds (HIGH = OFF)
  Serial.println("Both relays are OFF"); 
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  delay(15000); 
}