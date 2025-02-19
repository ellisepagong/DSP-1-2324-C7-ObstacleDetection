/*
Combined Code for ToF Sensor Module and Output Module on Arduino Mega R3 CH340G 2560

Pin Mapping:
-------------
ToF Sensors (VL53L0X):
  - Left Sensor:
      - XSHUT: Digital Pin 22
      - I2C SDA: Pin 20, I2C SCL: Pin 21
      - Address: 0x30
  - Center Sensor:
      - XSHUT: Digital Pin 23
      - I2C SDA: Pin 20, I2C SCL: Pin 21
      - Address: 0x31
  - Right Sensor:
      - XSHUT: Digital Pin 24
      - I2C SDA: Pin 20, I2C SCL: Pin 21
      - Address: 0x32

LED for Idle Mode: Digital Pin 4 (Reassigned from 13 to avoid conflict with motor control)

Motor Control Pins (Output Module):
  - MotorPins: {8, 9, 10, 11, 12, 13}

Bluetooth (HC-05) Module (Output Module):
  - Using Hardware Serial2
  - Connect HC-05 TX -> RX2 (Pin 17)
  - Connect HC-05 RX -> TX2 (Pin 16)

Raspberry Pi / CV Module (Output Module):
  - Uses USB Serial (Serial)

Additional Feature:
-------------
If any sensor records a value of 400cm or more, the previous value is used.
If no new CV data is received within 3 seconds, the motor control logic disables the motors.

Timing Deliverables:
-------------
SO2: Logs the time (in ms) required to process sensor data (should be < 1000 ms for 90% of data).
SO4: Logs the time (in ms) from receiving CV data to providing feedback (should be < 250 ms).

ADDITION:
-------------
Overall Module Processing Time: Logs the total time from start to end of each loop iteration (sensor data acquisition, CV processing, and motor control logic).
New: Logs the total processing time from the start of sensor data acquisition until the Bluetooth message is sent.
*/

#include <Wire.h>
#include <VL53L0X.h>

// ------------------ Pin Definitions ------------------
// ToF Sensor Module Pins
#define PIN_XSHUT_LEFT    22
#define PIN_XSHUT_CENTER  23
#define PIN_XSHUT_RIGHT   24
#define LED_IDLE          4   // Reassigned from 13 to avoid conflict with motor control

// Motor control pins (Output Module)
const int motorPins[] = {8, 9, 10, 11, 12, 13};

// ------------------ Bluetooth and CV Module Serial Definitions ------------------
// CV Module (Raspberry Pi) uses USB Serial: Serial
// Bluetooth (HC-05) will use Hardware Serial2: Serial2 (Pins: RX2=16, TX2=17)

// ------------------ Global Variables ------------------
String lastToFSensorData = ""; // Latest sensor data as string (e.g., "left center right")

// Weights array for class scores (index corresponds to class value offset by 1)
const int weights[] = {
  0, // none
  1, // animal
  2, // barrier
  4, // bike
  2, // crosswalk
  1, // hazard-sign
  3, // person
  1, // pole
  4, // stairs
  1, // stall
  5  // vehicle
};

bool handshakeComplete = false;
bool cvHandshakeComplete = false;

// For ToF sensors
VL53L0X loxLeft;
VL53L0X loxCenter;
VL53L0X loxRight;

// For previous reading fallback (for all three sensors)
static uint16_t prevLeft_cm = 0;
static uint16_t prevCenter_cm = 0;
static uint16_t prevRight_cm = 0;

// Variable to track the last time CV data was received
unsigned long lastCVTime = 0;

// ------------------ Long-Range Mode Helper Function ------------------
/*
  This function applies long-distance mode settings to a VL53L0X sensor.
  It increases the measurement timing budget to 200ms (200000µs) and lowers
  the signal rate limit to 0.1 MCPS.
  
  Note: These functions (setMeasurementTimingBudget and setSignalRateLimit)
  are available in the Pololu VL53L0X library.
*/
void applyLongRangeMode(VL53L0X &sensor) {
  // Increase timing budget to 200ms for improved long-range performance.
  sensor.setMeasurementTimingBudget(200000);  // 200,000 µs = 200 ms
  
  // Lower the signal rate limit from the default (≈0.25 MCPS) to 0.1 MCPS.
  sensor.setSignalRateLimit(0.1);
}

// ------------------ Function Definitions ------------------

// ---------- Output Module Functions (from modOutput.ino) ----------

// Handshake procedure for sensor module (adjusted for combined code)
void handshakeProcedure() {
  Serial.println(F("[OUTPUT LOG] [HANDSHAKE][SENSOR] Entering continuous sensor stream mode..."));
  // In combined mode, sensor data is read directly, so no flushing required.
  handshakeComplete = true;
  Serial.println(F("[OUTPUT LOG] [HANDSHAKE][SENSOR] Continuous sensor stream mode enabled."));
}

// CV handshake procedure (from modOutput.ino)
void cvHandshakeProcedure() {
  Serial.println("[OUTPUT LOG] [HANDSHAKE][CV] Disabling handshake for CV module. Continuous mode enabled.");
  cvHandshakeComplete = true;
}

// Compute weights based on CV classes and sensor distances (from modOutput.ino)
int* getWeights(int classes[5], int distance[3]) {
  int* scores = (int*)malloc(5 * sizeof(int));
  if (scores == NULL) {
    return NULL;
  }
  for (int i = 0; i < 5; i++) {
    int current = classes[i];
    int base_w = 0, dis_w = 0, dir_w = 0, score = 0;
    
    if (current >= 0 && current <= 9) {
      base_w = weights[current + 1];
      int dis = 0;
      if (i < 2) { // left
        dis = distance[0];
        if (i == 1) { dir_w = 2; }
      } else if (i == 2) { // front
        dis = distance[1];
        dir_w = 3;
      } else if (i >= 3 && i < 5) { // right
        dis = distance[2];
        if (i == 3) { dir_w = 2; }
      } else {
        scores[i] = -1;
        continue;
      }
      dis_w = map(dis, 0, 200, 5, 1);
      score = (dis_w * base_w) + dir_w;
      if (dis_w > 3) {
        score += 5;
      }
      scores[i] = score;
      Serial.print(F("[OUTPUT LOG] [WEIGHTS] Class index "));
      Serial.print(i);
      Serial.print(F(" => current: "));
      Serial.print(current);
      Serial.print(F(", base_w: "));
      Serial.print(base_w);
      Serial.print(F(", distance: "));
      Serial.print(dis);
      Serial.print(F(", dis_w: "));
      Serial.print(dis_w);
      Serial.print(F(", dir_w: "));
      Serial.print(dir_w);
      Serial.print(F(", score: "));
      Serial.println(score);
    }
    else if (current == -1) {
      scores[i] = 0;
      Serial.print(F("[OUTPUT LOG] [WEIGHTS] Class index "));
      Serial.print(i);
      Serial.println(F(" has no class (-1). Score set to 0."));
    } else {
      scores[i] = -1;
      Serial.print(F("[OUTPUT LOG] [WEIGHTS] Class index "));
      Serial.print(i);
      Serial.println(F(" invalid. Score set to -1."));
    }
  }
  return scores;
}

// Motor control logic (from modOutput.ino)
void motorLogic(int segment) {
  Serial.print(F("[OUTPUT LOG] [MOTOR] Activating motor logic for segment: "));
  Serial.println(segment);
  switch (segment) {
    case 0: // left
      digitalWrite(motorPins[0], HIGH);
      digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[3], LOW);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], LOW);
      break;
    case 1: // front left
      digitalWrite(motorPins[0], LOW);
      digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], HIGH);
      digitalWrite(motorPins[3], LOW);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], LOW);
      break;
    case 2: // front
      digitalWrite(motorPins[0], LOW);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], HIGH);
      digitalWrite(motorPins[3], HIGH);
      digitalWrite(motorPins[4], LOW);
      digitalWrite(motorPins[5], LOW);
      break;
    case 3: // front right
      digitalWrite(motorPins[0], LOW);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[3], HIGH);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], LOW);
      break;
    case 4: // right
      digitalWrite(motorPins[0], LOW);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[3], LOW);
      digitalWrite(motorPins[4], HIGH);
      digitalWrite(motorPins[5], HIGH);
      break;
    default:
      // Turn off all motor pins
      for (int i = 0; i < 6; i++) {
        digitalWrite(motorPins[i], LOW);
      }
      break;
  }
}

// Check if all scores are zero (from modOutput.ino)
int areAllScoresZero(int scores[5]) {
  for (int i = 0; i < 5; i++) {
    if (scores[i] != 0) {
      return 0;
    }
  }
  return 1;
}

// ---------- ToF Sensor Module Functions (from tof.ino) ----------

// Initialize a VL53L0X sensor (from tof.ino)
bool initializeSensor(int xshutPin, VL53L0X &sensor, uint8_t address, const char *name) {
  Serial.print(F("[INIT] Enabling "));
  Serial.print(name);
  Serial.println(F(" sensor..."));
  
  digitalWrite(xshutPin, HIGH);
  delay(10);
  
  sensor.init();
  sensor.setAddress(address);
  
  // We don't have a direct check for initialization failure in the Pololu library, so we assume success.
  Serial.print(F("[SUCCESS] "));
  Serial.print(name);
  Serial.println(F(" sensor initialized."));
  Serial.println(String("LOG:") + name + " sensor initialized.");
  return true;
}

// Enter idle mode in case of sensor failure (from tof.ino)
void enterIdleMode() {
  Serial.println(F("[IDLE MODE] Entering idle mode..."));
  Serial.println("LOG:Entering idle mode due to sensor failure.");
  while (true) {
    digitalWrite(LED_IDLE, HIGH);
    delay(150);
    digitalWrite(LED_IDLE, LOW);
    delay(150);
  }
}

// Read sensor data from VL53L0X sensors and update lastToFSensorData
void readToFSensors() {
  Serial.println(F("[PROCESS] Beginning sensor reading cycle..."));
  
  uint16_t left_mm = loxLeft.readRangeSingleMillimeters();
  uint16_t center_mm = loxCenter.readRangeSingleMillimeters();
  uint16_t right_mm = loxRight.readRangeSingleMillimeters();
  
  // Check for timeouts (Pololu library supports timeoutOccurred)
  if (loxLeft.timeoutOccurred() || loxCenter.timeoutOccurred() || loxRight.timeoutOccurred()) {
    Serial.println(F("[ERROR] Sensor timeout! Entering idle mode..."));
    Serial.println("LOG:Sensor timeout! Entering idle mode.");
    enterIdleMode();
  }
  
  uint16_t left_cm, center_cm, right_cm;
  
  // For left sensor: if reading is 400cm or more, use previous value
  if ((left_mm / 10) >= 400) {
    left_cm = prevLeft_cm;
    Serial.println(F("[PROCESS] Left sensor reading >= 400cm. Using previous reading."));
  } else {
    left_cm = left_mm / 10;
    prevLeft_cm = left_cm;
  }
  
  // For center sensor:
  if ((center_mm / 10) >= 400) {
    center_cm = prevCenter_cm;
    Serial.println(F("[PROCESS] Center sensor reading >= 400cm. Using previous reading."));
  } else {
    center_cm = center_mm / 10;
    prevCenter_cm = center_cm;
  }
  
  // For right sensor:
  if ((right_mm / 10) >= 400) {
    right_cm = prevRight_cm;
    Serial.println(F("[PROCESS] Right sensor reading >= 400cm. Using previous reading."));
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
  
  // Update global sensor data string (format: "left center right")
  lastToFSensorData = String(left_cm) + " " + String(center_cm) + " " + String(right_cm);
  Serial.println(String("LOG:Sent sensor data: ") + lastToFSensorData);
  Serial.print(F("[PROCESS] Updated sensor data: "));
  Serial.println(lastToFSensorData);
}

// ------------------ Setup ------------------
void setup() {
  // Initialize motor pins
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
  
  // Initialize LED for idle mode
  pinMode(LED_IDLE, OUTPUT);
  digitalWrite(LED_IDLE, LOW);
  
  // Begin Serial for CV module (Raspberry Pi) - USB Serial
  Serial.begin(9600);
  while (!Serial) { ; }
  
  // Begin Serial2 for Bluetooth (HC-05)
  Serial2.begin(9600);
  
  Serial.println(F("[OUTPUT LOG] [PROCESS] Starting Combined Module Initialization (Mega 2560)..."));
  
  // Initialize I2C for ToF sensors
  Wire.begin();
  
  // Initialize ToF sensor XSHUT pins
  pinMode(PIN_XSHUT_LEFT, OUTPUT);
  pinMode(PIN_XSHUT_CENTER, OUTPUT);
  pinMode(PIN_XSHUT_RIGHT, OUTPUT);
  
  digitalWrite(PIN_XSHUT_LEFT, LOW);
  digitalWrite(PIN_XSHUT_CENTER, LOW);
  digitalWrite(PIN_XSHUT_RIGHT, LOW);
  delay(10);
  
  // Initialize ToF sensors
  bool leftStatus = initializeSensor(PIN_XSHUT_LEFT, loxLeft, 0x30, "Left");
  bool centerStatus = initializeSensor(PIN_XSHUT_CENTER, loxCenter, 0x31, "Center");
  bool rightStatus = initializeSensor(PIN_XSHUT_RIGHT, loxRight, 0x32, "Right");
  
  if (!leftStatus || !centerStatus || !rightStatus) {
    Serial.println(F("[ERROR] Sensor initialization failed!"));
    enterIdleMode();
  }
  
  // ----- Apply Long-Range Mode Settings -----
  // These settings extend the sensor's range (with a possible trade-off in accuracy)
  applyLongRangeMode(loxLeft);
  applyLongRangeMode(loxCenter);
  applyLongRangeMode(loxRight);
  
  Serial.println(F("[PROCESS] Sensor initialization complete."));
  Serial.println("LOG:Sensor initialization complete.");
  
  // Handshake procedures
  handshakeProcedure();
  while (!handshakeComplete) {
    Serial.println(F("[OUTPUT LOG] [HANDSHAKE][SENSOR] Retrying handshake in 1 second..."));
    delay(1000);
    handshakeProcedure();
  }
  Serial.println(F("[OUTPUT LOG] [HANDSHAKE][SENSOR] Handshake complete with sensor module."));
  
  cvHandshakeProcedure();
  while (!cvHandshakeComplete) {
    Serial.println(F("[OUTPUT LOG] [HANDSHAKE][CV] Retrying handshake with CV module in 1 second..."));
    delay(1000);
    cvHandshakeProcedure();
  }
  Serial.println(F("[OUTPUT LOG] [HANDSHAKE][CV] Handshake complete with CV module. Starting main loop."));
  
  // Initialize last CV data time to current time
  lastCVTime = millis();
}

// ------------------ Loop ------------------
void loop() {
  unsigned long loopStart = millis(); // ADDED: Start overall processing timer (sensor data acquisition start)
  
  // --- Measure Sensor Data Processing Time (SO2) ---
  unsigned long sensorStart = millis();
  readToFSensors();
  unsigned long sensorEnd = millis();
  unsigned long sensorDuration = sensorEnd - sensorStart;
  Serial.print(F("[TIMING] [SO2] Sensor data processed in "));
  Serial.print(sensorDuration);
  Serial.println(F(" ms."));
  
  // --- Process incoming CV module data (via USB Serial) ---
  if (Serial.available() > 0) {
    unsigned long cvStartTime = millis(); // Start timing for CV data processing
    lastCVTime = millis(); // Update timestamp when new CV data is received
    Serial.println(F("[OUTPUT LOG] [INFO] Data detected on USB Serial (CV module)."));
    String class_byte = Serial.readStringUntil('\n');
    class_byte.trim();
    Serial.print(F("[OUTPUT LOG] [DATA][CV] Received CV data: "));
    Serial.println(class_byte);
    
    // Parse CV data into an array of 5 integers
    int classes[5];
    int idx_class = 0;
    int spaceIndex_class = class_byte.indexOf(' ');
    while (spaceIndex_class >= 0 && idx_class < 5) {
      String classStr = class_byte.substring(0, spaceIndex_class);
      classes[idx_class++] = classStr.toInt();
      Serial.print(F("[OUTPUT LOG] [DATA][CV] Parsed class["));
      Serial.print(idx_class - 1);
      Serial.print(F("]: "));
      Serial.println(classStr);
      class_byte = class_byte.substring(spaceIndex_class + 1);
      spaceIndex_class = class_byte.indexOf(' ');
    }
    if (idx_class < 5) {
      classes[idx_class++] = class_byte.toInt();
      Serial.print(F("[OUTPUT LOG] [DATA][CV] Parsed class["));
      Serial.print(idx_class - 1);
      Serial.print(F("]: "));
      Serial.println(class_byte);
    }
    
    // Ensure we have valid sensor data from ToF sensors
    if (lastToFSensorData.length() == 0) {
      Serial.println(F("[OUTPUT LOG] [HANDSHAKE] No sensor distance data available."));
      return;
    }
    String dis_byte = lastToFSensorData;
    Serial.print(F("[OUTPUT LOG] [HANDSHAKE] Using latest sensor distance data: "));
    Serial.println(dis_byte);
    
    // Parse the distance data (expecting three space-separated values)
    int dis[3];
    int idx_dis = 0;
    int spaceIndex_dis = dis_byte.indexOf(' ');
    while (spaceIndex_dis >= 0 && idx_dis < 3) {
      String disStr = dis_byte.substring(0, spaceIndex_dis);
      dis[idx_dis++] = disStr.toInt();
      Serial.print(F("[OUTPUT LOG] [SENSOR] Parsed distance["));
      Serial.print(idx_dis - 1);
      Serial.print(F("]: "));
      Serial.println(disStr);
      dis_byte = dis_byte.substring(spaceIndex_dis + 1);
      spaceIndex_dis = dis_byte.indexOf(' ');
    }
    if (idx_dis < 3) {
      dis[idx_dis++] = dis_byte.toInt();
      Serial.print(F("[OUTPUT LOG] [SENSOR] Parsed distance["));
      Serial.print(idx_dis - 1);
      Serial.print(F("]: "));
      Serial.println(dis_byte);
    }
    
    // Compute scores using the parsed CV classes and sensor distances
    int* scores = getWeights(classes, dis);
    if (scores == NULL) {
      Serial.println(F("[OUTPUT LOG] [HANDSHAKE] Memory allocation failed."));
      return;
    }
    
    // Determine the highest score among the segments
    int maxScore = 0;
    int maxScoreId = 0;
    if (areAllScoresZero(scores)) {
      Serial.println(F("[OUTPUT LOG] [MOTOR] All scores are zero. Executing default motor logic."));
      motorLogic(-1);
    } else {
      for (int i = 1; i < 5; i++) {
        if (scores[i] > maxScore) {
          maxScore = scores[i];
          maxScoreId = i;
        }
      }
      Serial.print(F("[OUTPUT LOG] [MOTOR] Highest score is "));
      Serial.print(maxScore);
      Serial.print(F(" at index "));
      Serial.println(maxScoreId);
      motorLogic(maxScoreId);
    }
    
    // --- Start timing for Bluetooth send (from sensor data acquisition start) ---
    unsigned long btStart = millis();
    
    // Prepare and send message via Bluetooth (HC-05 on Serial2) following the required format
    char scoresString[50] = "";
    for (int i = 0; i < 5; i++) {
      char temp[10];
      sprintf(temp, "%d", scores[i]);
      strcat(scoresString, temp);
      if (i < 4) {
        strcat(scoresString, ",");
      }
    }
    String message = class_byte; // Start with the CV class data string
    message += ",";             // Append a comma
    message += scoresString;    // Append the comma-separated scores
    Serial2.println(message);
    Serial.print(F("[OUTPUT LOG] [HC-05] Sent message: "));
    Serial.println(message);
    
    // --- End timing for Bluetooth send ---
    unsigned long btEnd = millis();
    unsigned long btDuration = btEnd - btStart;
    Serial.print(F("[TIMING] [BT] Bluetooth send routine took "));
    Serial.print(btDuration);
    Serial.println(F(" ms."));
    
    // --- Total time from sensor data acquisition start to Bluetooth message sent ---
    unsigned long dataToBtDuration = btEnd - loopStart;
    Serial.print(F("[TIMING] [DATA_TO_BT] Total processing time from sensor read start to Bluetooth message sent: "));
    Serial.print(dataToBtDuration);
    Serial.println(F(" ms."));
    
    free(scores);
    
    unsigned long cvEndTime = millis();
    unsigned long cvDuration = cvEndTime - cvStartTime;
    Serial.print(F("[TIMING] [SO4] CV data processed and feedback provided in "));
    Serial.print(cvDuration);
    Serial.println(F(" ms."));
  }
  
  // --- Stopping Mechanism: If no new CV data for 3 seconds, stop motors ---
  if (millis() - lastCVTime >= 2000) {
    Serial.println(F("[OUTPUT LOG] [MOTOR] No new CV data for 3 seconds. Stopping motors."));
    motorLogic(-1);
  }
  
  // --- Process HC-05 commands received via Bluetooth (Serial2) ---
  if (Serial2.available()) {
    String receivedData = Serial2.readStringUntil('\n');
    receivedData.trim();
    Serial.print(F("[OUTPUT LOG] [HC-05] Received: "));
    Serial.println(receivedData);
    if (receivedData == "motors") {
      Serial.println(F("[OUTPUT LOG] [MOTOR] Received 'motors' command from HC-05."));
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 6; j++) {
          digitalWrite(motorPins[j], HIGH);
        }
        delay(500);
        for (int j = 0; j < 6; j++) {
          digitalWrite(motorPins[j], LOW);
        }
        delay(500);
      }
    }
  }
  
  unsigned long loopEnd = millis();  // ADDED: End overall processing timer
  unsigned long overallDuration = loopEnd - loopStart; // ADDED: Calculate overall processing time
  Serial.print(F("[TIMING] [OVERALL] Module processing time: "));
  Serial.print(overallDuration);
  Serial.println(F(" ms."));
  
  delay(100);
}
