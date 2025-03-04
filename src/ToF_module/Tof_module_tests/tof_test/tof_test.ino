/*
  Simple ToF Sensor Test Script for Arduino Mega 2560
  - Checks if all three VL53L0X sensors initialize correctly.
  - Continuously prints their distance readings.
  - Uses an LED on pin 13 for status indication:
    - Slow blinking: Sensors working
    - Fast blinking: Initialization failure
  
  Wiring:
  - All sensors share SDA (pin 20) and SCL (pin 21)
  - XSHUT pins: Left (22), Center (23), Right (24)
*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// XSHUT Pins
#define PIN_XSHUT_LEFT    22  
#define PIN_XSHUT_CENTER  23  
#define PIN_XSHUT_RIGHT   24  
#define STATUS_LED        13  // LED indicator

// Sensor Objects
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

void setup() {
  Serial.begin(9600);  // Use a faster baud rate for debugging

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  Wire.begin();  // Initialize I²C bus
  Serial.println(F("[PROCESS STAGE] Starting ToF Sensor Test..."));

  // Initialize Sensors
  if (!initializeSensors()) {
    Serial.println(F("[ERROR] One or more sensors failed to initialize! Entering idle mode..."));
    idleMode();
  } else {
    Serial.println(F("[SUCCESS] All sensors initialized successfully!"));
  }
}

void loop() {
  Serial.println(F("[PROCESS STAGE] Reading distances..."));

  uint16_t leftDistance = loxLeft.readRange();
  uint16_t centerDistance = loxCenter.readRange();
  uint16_t rightDistance = loxRight.readRange();

  Serial.print(F("[SENSOR DATA] Left: "));
  Serial.print(leftDistance);
  Serial.print(F(" mm | Center: "));
  Serial.print(centerDistance);
  Serial.print(F(" mm | Right: "));
  Serial.print(rightDistance);
  Serial.println(F(" mm"));

  // Blink LED slowly to indicate successful operation
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  digitalWrite(STATUS_LED, LOW);
  delay(500);

  delay(250); // Short pause before next reading
}

bool initializeSensors() {
  Serial.println(F("[PROCESS STAGE] Disabling all sensors (setting XSHUT pins LOW)."));

  pinMode(PIN_XSHUT_LEFT, OUTPUT);
  pinMode(PIN_XSHUT_CENTER, OUTPUT);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT);

  digitalWrite(PIN_XSHUT_LEFT, LOW);
  digitalWrite(PIN_XSHUT_CENTER, LOW);
  digitalWrite(PIN_XSHUT_RIGHT, LOW);
  delay(10);  // Ensure sensors are fully off

  bool leftOK = false, centerOK = false, rightOK = false;

  // LEFT SENSOR
  Serial.println(F("[PROCESS STAGE] Enabling Left Sensor..."));
  digitalWrite(PIN_XSHUT_LEFT, HIGH);
  delay(10);
  leftOK = loxLeft.begin(0x30);
  Serial.println(leftOK ? F("[SUCCESS] Left sensor initialized at I²C address 0x30.") : F("[ERROR] Left sensor failed to initialize!"));

  // CENTER SENSOR
  Serial.println(F("[PROCESS STAGE] Enabling Center Sensor..."));
  digitalWrite(PIN_XSHUT_CENTER, HIGH);
  delay(10);
  centerOK = loxCenter.begin(0x31);
  Serial.println(centerOK ? F("[SUCCESS] Center sensor initialized at I²C address 0x31.") : F("[ERROR] Center sensor failed to initialize!"));

  // RIGHT SENSOR
  Serial.println(F("[PROCESS STAGE] Enabling Right Sensor..."));
  digitalWrite(PIN_XSHUT_RIGHT, HIGH);
  delay(10);
  rightOK = loxRight.begin(0x32);
  Serial.println(rightOK ? F("[SUCCESS] Right sensor initialized at I²C address 0x32.") : F("[ERROR] Right sensor failed to initialize!"));

  return (leftOK && centerOK && rightOK);
}

void idleMode() {
  while (true) {
    Serial.println(F("[IDLE MODE] Sensors failed. Restarting initialization..."));
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);

    if (initializeSensors()) {
      Serial.println(F("[RECOVERY] Sensors re-initialized successfully!"));
      break;
    }
  }
}
