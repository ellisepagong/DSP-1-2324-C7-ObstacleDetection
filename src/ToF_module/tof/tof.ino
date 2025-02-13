/*
   ToF Sensor Module Sketch for Arduino Mega 2560

   - Reads three VL53L0X sensors.
   - Implements a handshake over Serial1 (TX1 on D18, RX1 on D19) with the Output Module.
   - Sends sensor data as a space-delimited string: "left_cm center_cm right_cm".
   - Logs status messages via USB Serial.
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

// --- Handshake Procedure ---
void handshakeProcedure() {
  Serial.println(F("[HANDSHAKE] Starting handshake with Output Module..."));
  
  // Flush any stray data on Serial1.
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(F("[HANDSHAKE] Flushed from Serial1: "));
    Serial.println(c);
  }
  
  unsigned long startTime = millis();
  bool ackReceived = false;
  
  while ((millis() - startTime < 5000) && !ackReceived) {
    Serial1.println("HELLO_TOF");
    Serial.println(F("[HANDSHAKE] Sent: HELLO_TOF"));
    delay(500);
    
    if (Serial1.available()) {
      String response = Serial1.readStringUntil('\n');
      response.trim();
      Serial.print(F("[HANDSHAKE] Received: "));
      Serial.println(response);
      if (response.equals("HELLO_OUTPUT")) {
         ackReceived = true;
         handshakeComplete = true;
         Serial.println(F("[HANDSHAKE] Handshake complete!"));
      }
    }
  }
  
  if (!ackReceived) {
    Serial.println(F("[HANDSHAKE] Handshake failed, retrying..."));
  }
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
    return false;
  }
  Serial.print(F("[SUCCESS] "));
  Serial.print(name);
  Serial.println(F(" sensor initialized."));
  return true;
}

void enterIdleMode() {
  Serial.println(F("[IDLE MODE] Entering idle mode..."));
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
    enterIdleMode();
  }
  
  uint16_t left_cm = left_mm / 10;
  uint16_t center_cm = center_mm / 10;
  uint16_t right_cm = right_mm / 10;
  
  Serial.print(F("[SENSOR DATA] Left: "));
  Serial.print(left_cm);
  Serial.print(F(" cm, Center: "));
  Serial.print(center_cm);
  Serial.print(F(" cm, Right: "));
  Serial.print(right_cm);
  Serial.println(F(" cm"));
  
  String sensorData = String(left_cm) + " " + String(center_cm) + " " + String(right_cm);
  Serial1.println(sensorData);
  Serial.print(F("[PROCESS] Sent data to Output Module: "));
  Serial.println(sensorData);
  
  delay(100);
}
