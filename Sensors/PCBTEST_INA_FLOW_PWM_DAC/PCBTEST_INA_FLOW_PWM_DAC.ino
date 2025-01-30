/*
This sketch:
 1) Reads two Adafruit INA260 current sensors (addresses 0x40, 0x41).
 2) Reads a Sensirion SF06 flow sensor (SLF3C_1300F).
 3) Outputs a 60 Hz PWM on GPIO16 (adjustable frequency via serial).
 4) Outputs a "step-like" triangle wave on the DAC (GPIO17) with a 1s delay in loop,
    completing one up/down cycle in ~10s (or user-adjustable).

Board: ESP32-S2-DevKitM-1
*/

#include <Adafruit_INA260.h>
#include <SensirionI2cSf06Lf.h>
#include <Wire.h>

// ---------- LEDC (PWM) Includes ----------
#include "driver/ledc.h"  // For low-level ESP-IDF LEDC config

// ---------- DAC Includes ----------
#include "driver/dac.h"   // For DAC output on ESP32-S2 (GPIO17=Channel1, GPIO18=Channel2)

// ========== [ Global Objects for Sensors ] ==========
Adafruit_INA260 currentSensor1 = Adafruit_INA260();
Adafruit_INA260 currentSensor2 = Adafruit_INA260();
SensirionI2cSf06Lf sensor;

// ========== [ Sensor Addresses ] ==========
uint8_t address1 = 0x40; // INA260 #1
uint8_t address2 = 0x41; // INA260 #2

// ========== [ LEDC PWM Configuration ] ==========
#define PWM_PIN         16                // GPIO16 for PWM
#define PWM_FREQ        60                // Default freq 60Hz
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT  // 8-bit resolution
#define PWM_CHANNEL     LEDC_CHANNEL_0
#define SPEED_MODE      LEDC_LOW_SPEED_MODE
#define TIMER_NUM       LEDC_TIMER_0
#define DUTY_50_PERCENT 128               // 50% duty

// Function to reconfigure PWM frequency
uint32_t setFrequency(uint32_t freq) {
  ledc_timer_config_t ledc_timer = {
    .speed_mode      = SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num       = TIMER_NUM,
    .freq_hz         = freq,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  // Return the actual freq (may differ slightly due to clock divisors)
  return ledc_get_freq(SPEED_MODE, TIMER_NUM);
}

// ========== [ DAC Triangle Wave Setup ] ==========

// Desired total cycle time in seconds (from 0V → max → 0V).
// Change this to any number of seconds you want:
float totalCycleTimeSec = 20.0f;

// Because we have a 1s delay each loop, we do 1 DAC step per second.
// So total steps per full cycle = totalCycleTimeSec.
static int  totalStepsInCycle;    // Will be computed from totalCycleTimeSec
static int  stepsUp, stepsDown;   // half the cycle up, half down
static int  stepSize;             // how many DAC units to move per step

// Current DAC state
static int  dacValue   = 0;       // 0..255
static bool goingUp    = true;    // direction

void initDacTriangle() {
  // If totalCycleTimeSec is 10 and loop delay is 1s, we do 10 steps total.
  // 5 steps up, 5 steps down => 10 steps total for a full cycle.
  // We must map 0..255 into 5 increments for half-cycle => step size ~51.
  
  totalStepsInCycle = (int)totalCycleTimeSec;
  if (totalStepsInCycle < 2) {
    totalStepsInCycle = 2;  // minimum to avoid division by zero
  }

  // half the steps up, half down
  stepsUp   = totalStepsInCycle / 2;
  stepsDown = totalStepsInCycle - stepsUp;  // remainder

  // Move from 0..255 in 'stepsUp' increments => stepSize
  if (stepsUp < 1) stepsUp = 1;  
  stepSize = 255 / stepsUp;  // integer division

  // Start at 0
  dacValue = 0;
  goingUp  = true;
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, dacValue);
}

// Move the DAC one step each loop iteration
void updateDacTriangleOneStep() {
  if (goingUp) {
    // Move up by stepSize
    dacValue += stepSize;
    if (dacValue >= 255) {
      dacValue = 255;
      goingUp  = false; 
    }
  } else {
    // Move down by stepSize
    dacValue -= stepSize;
    if (dacValue <= 0) {
      dacValue = 0;
      goingUp  = true;
    }
  }
  dac_output_voltage(DAC_CHANNEL_1, dacValue);
}

// ========== [ SETUP ] ==========
void setup() {
  // ---------- [ Serial Setup ] ----------
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println();
  Serial.println("=== Starting Sensors, PWM, and DAC ===");

  // ---------- [ INA260 Sensor #1 ] ----------
  if (!currentSensor1.begin(address1, &Wire)) {
    Serial.println("Couldn't find current sensor 1");
    while (1);
  }
  Serial.println("Found current sensor 1");

  // ---------- [ INA260 Sensor #2 ] ----------
  if (!currentSensor2.begin(address2, &Wire)) {
    Serial.println("Couldn't find current sensor 2");
    while (1);
  }
  Serial.println("Found current sensor 2");

  // ---------- [ Sensirion Flow Sensor ] ----------
  Wire.begin(); // On ESP32-S2, SDA=Pin8, SCL=Pin9
  sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);
  delay(100);
  sensor.startH2oContinuousMeasurement();
  Serial.println();

  // ---------- [ LEDC PWM Setup on GPIO16 ] ----------
  {
    ledc_timer_config_t ledc_timer = {
      .speed_mode      = SPEED_MODE,
      .duty_resolution = PWM_RESOLUTION,
      .timer_num       = TIMER_NUM,
      .freq_hz         = PWM_FREQ,
      .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
      .gpio_num       = PWM_PIN,
      .speed_mode     = SPEED_MODE,
      .channel        = PWM_CHANNEL,
      .intr_type      = LEDC_INTR_DISABLE,
      .timer_sel      = TIMER_NUM,
      .duty           = DUTY_50_PERCENT,
      .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);

    uint32_t actualFreq = ledc_get_freq(SPEED_MODE, TIMER_NUM);
    Serial.printf("PWM on GPIO%d at ~%d Hz (50%% duty)\n", PWM_PIN, actualFreq);
    Serial.println("Type a positive integer (e.g., 120) to set new PWM freq.\n");
  }

  // ---------- [ DAC Triangle Setup on GPIO17 ] ----------
  // On ESP32-S3, DAC_CHANNEL_1 -> GPIO17
  // (ESP32-S2 does NOT have a hardware DAC, so this only works if your board
  //  actually includes a DAC on pin 17.)
  initDacTriangle();

  Serial.printf("DAC on GPIO17: full wave in ~%.1f seconds (1 step/second)\n",
                totalCycleTimeSec);
}

// ========== [ LOOP ] ==========
void loop() {
  // 1) Each iteration, move DAC one step
  updateDacTriangleOneStep();

  // 2) Read INA260 #1
  Serial.print("Sensor 1 Current: ");
  Serial.print(currentSensor1.readCurrent());
  Serial.println(" mA");

  Serial.print("Sensor 1 Bus Voltage: ");
  Serial.print(currentSensor1.readBusVoltage() / 1000.0);
  Serial.println(" V");

  Serial.print("Sensor 1 Power: ");
  Serial.print(currentSensor1.readPower());
  Serial.println(" mW\n");

  // 3) Read INA260 #2
  Serial.print("Sensor 2 Current: ");
  Serial.print(currentSensor2.readCurrent());
  Serial.println(" mA");

  Serial.print("Sensor 2 Bus Voltage: ");
  Serial.print(currentSensor2.readBusVoltage() / 1000.0);
  Serial.println(" V");

  Serial.print("Sensor 2 Power: ");
  Serial.print(currentSensor2.readPower());
  Serial.println(" mW\n");

  // 4) Read Sensirion Flow
  float aFlow = 0.0;
  float aTemperature = 0.0;
  uint16_t aSignalingFlags = 0u;
  sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F,
                             aFlow, aTemperature, aSignalingFlags);

  Serial.print("Flow: ");
  Serial.print(aFlow);
  Serial.print(" ml/min,\tTemperature: ");
  Serial.print(aTemperature);
  Serial.print(" C,\tSignalingFlags: ");
  Serial.println(aSignalingFlags);
  Serial.println("-----");

  // 5) Check Serial for new PWM frequency
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int newFreq = input.toInt();

    if (newFreq > 0) {
      uint32_t realFreq = setFrequency(newFreq);
      Serial.printf("New freq requested: %d Hz, actual: %d Hz\n", newFreq, realFreq);
    } else {
      Serial.println("Invalid input! Enter a positive integer for PWM freq.\n");
    }
  }

  // 6) Wait 1 second
  delay(1000);
}
