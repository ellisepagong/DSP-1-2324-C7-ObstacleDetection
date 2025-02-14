/*
   ToF Sensor Module Sketch for Arduino Mega 2560 with additional logging

   - Reads three VL53L0X sensors.
   - Implements a handshake over Serial1 with the Output Module.
   - Sends sensor data as a space-delimited string prefixed with "SENS:".
   - Sends log messages prefixed with "LOG:" to aid in debugging.
*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Pin definitions for Mega 2560
#define PIN_XSHUT_LEFT    22
#define PIN_XSHUT_CENTER  23
#define PIN_XSHUT_RIGHT   24
#define LED_IDLE          13

// Global sensor objects
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxCenter = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

bool handshakeComplete = false;

void handshakeProcedure() {
  Serial.println(F("[HANDSHAKE] Entering continuous stream mode..."));
  // Flush any stray data from Serial1
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(F("[HANDSHAKE] Flushed from Serial1: "));
    Serial.println(c);
  }
  handshakeComplete = true;
  Serial.println(F("[HANDSHAKE] Continuous stream mode enabled."));
  // Also notify the Output Module
  Serial1.println("LOG:Handshake complete. Sensor data streaming starting.");
}

bool initializeSensor(int xshutPin, Adafruit_VL53L0X &sensor, uint8_t address, const char *name) {
  Serial.print(F("[INIT] Enabling "));
  Serial.print(name);
  Serial.println(F(" sensor..."));
  
  digitalWrite(xshutPin, HIGH);
  delay(10);
  
  if (!sensor.begin(address)) {
    Serial.print(F("[ERROR] "));
    Serial.print(name);
    Serial.println(F(" sensor failed to initialize!"));
    Serial1.println(String("LOG:") + name + " sensor failed to initialize!");
    return false;
  }
  Serial.print(F("[SUCCESS] "));
  Serial.print(name);
  Serial.println(F(" sensor initialized."));
  Serial1.println(String("LOG:") + name + " sensor initialized.");
  return true;
}

void enterIdleMode() {
  Serial.println(F("[IDLE MODE] Entering idle mode..."));
  Serial1.println("LOG:Entering idle mode due to sensor failure.");
  while (true) {
    digitalWrite(LED_IDLE, HIGH);
    delay(150);
    digitalWrite(LED_IDLE, LOW);
    delay(150);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  
  Serial.println(F("[PROCESS] Starting ToF Sensor Module Initialization on Mega 2560..."));
  Wire.begin();
  
  pinMode(PIN_XSHUT_LEFT, OUTPUT);
  pinMode(PIN_XSHUT_CENTER, OUTPUT);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT);
  pinMode(LED_IDLE, OUTPUT);
  
  digitalWrite(PIN_XSHUT_LEFT, LOW);
  digitalWrite(PIN_XSHUT_CENTER, LOW);
  digitalWrite(PIN_XSHUT_RIGHT, LOW);
  digitalWrite(LED_IDLE, LOW);
  delay(10);
  
  bool leftStatus = initializeSensor(PIN_XSHUT_LEFT, loxLeft, 0x30, "Left");
  bool centerStatus = initializeSensor(PIN_XSHUT_CENTER, loxCenter, 0x31, "Center");
  bool rightStatus = initializeSensor(PIN_XSHUT_RIGHT, loxRight, 0x32, "Right");
  
  if (!leftStatus || !centerStatus || !rightStatus) {
    Serial.println(F("[ERROR] Sensor initialization failed!"));
    enterIdleMode();
  }
  
  Serial.println(F("[PROCESS] Sensor initialization complete."));
  Serial1.println("LOG:Sensor initialization complete.");
  
  Serial1.begin(9600);  // TX1 on D18, RX1 on D19
  delay(100);
  
  while (!handshakeComplete) {
    handshakeProcedure();
  }
}

void loop() {
  Serial.println(F("[PROCESS] Beginning sensor reading cycle..."));
  
  uint16_t left_mm = loxLeft.readRange();
  uint16_t center_mm = loxCenter.readRange();
  uint16_t right_mm = loxRight.readRange();
  
  if (loxLeft.timeoutOccurred() || loxCenter.timeoutOccurred() || loxRight.timeoutOccurred()) {
    Serial.println(F("[ERROR] Sensor timeout! Entering idle mode..."));
    Serial1.println("LOG:Sensor timeout! Entering idle mode.");
    enterIdleMode();
  }
  
  uint16_t left_cm = left_mm / 10;
  uint16_t center_cm = center_mm / 10;
  static uint16_t prevRight_cm = 0;
  uint16_t right_cm;
  
  if (right_mm >= 8190) {
    right_cm = prevRight_cm;
    Serial.println(F("[PROCESS] Right sensor returned 'no object'. Using previous reading."));
    Serial1.println("LOG:Right sensor 'no object' value; using previous reading.");
  } else {
    right_cm = right_mm / 10;
    prevRight_cm = right_cm;
  }
  
  Serial.print(F("[SENSOR DATA] Left: "));
  Serial.print(left_cm);
  Serial.print(F(" cm, Center: "));
  Serial.print(center_cm);
  Serial.print(F(" cm, Right: "));
  Serial.print(right_cm);
  Serial.println(F(" cm"));
  
  String sensorData = String(left_cm) + " " + String(center_cm) + " " + String(right_cm);
  // Send sensor data with prefix "SENS:" so the Output Module can parse it
  Serial1.println("SENS:" + sensorData);
  Serial1.println("LOG:Sent sensor data to Output Module: " + sensorData);
  Serial.print(F("[PROCESS] Sent data to Output Module: "));
  Serial.println(sensorData);
  
  delay(100);
}
