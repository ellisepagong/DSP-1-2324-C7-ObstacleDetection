// Updated modoutput code with additional console logs

#include <SoftwareSerial.h>

// Reassign RX and TX pins for HC-05
#define HC05_RX 2 // connect to HC-05 TX pin
#define HC05_TX 3 // connect to HC-05 RX pin
#define CMOS_RX 5 // connect to CMOS Arduino TX pin
#define CMOS_TX 6 // connect to CMOS Arduino RX pin

const int motorPins[] = {8, 9, 10, 11, 12, 13};

String lastToFSensorData = ""; // Global variable to store latest sensor data

// SoftwareSerial for HC-05 & CMOS module
SoftwareSerial hc05(HC05_RX, HC05_TX);
SoftwareSerial cmos(CMOS_RX, CMOS_TX);

// Define weights for classes
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

void handshakeProcedure() {
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Entering continuous stream mode..."));
  // Flush any stray data on the CMOS stream
  while (cmos.available()) {
    char flushed = cmos.read();
    Serial.print(F("[OUTPUT][HANDSHAKE][TOF] Flushed stray byte: "));
    Serial.println(flushed);
  }
  handshakeComplete = true;
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Continuous stream mode enabled."));
}

void cvHandshakeProcedure() {
  Serial.println("[OUTPUT][HANDSHAKE][CV] Disabling handshake for CV module. Continuous mode enabled.");
  cvHandshakeComplete = true;
}

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
      } else if (i >= 2 && i < 5) { // right
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
      Serial.print(F("[OUTPUT][WEIGHTS] Class index "));
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
      Serial.print(F("[OUTPUT][WEIGHTS] Class index "));
      Serial.print(i);
      Serial.println(F(" has no class (-1). Score set to 0."));
    } else {
      scores[i] = -1;
      Serial.print(F("[OUTPUT][WEIGHTS] Class index "));
      Serial.print(i);
      Serial.println(F(" invalid. Score set to -1."));
    }
  }
  return scores;
}

void motorLogic(int segment) {
  Serial.print(F("[OUTPUT][MOTOR] Activating motor logic for segment: "));
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
      for (int i = 0; i < 6; i++) {
        digitalWrite(motorPins[i], LOW);
      }
      break;
  }
}

int areAllScoresZero(int scores[5]) {
  for (int i = 0; i < 5; i++) {
    if (scores[i] != 0) {
      return 0;
    }
  }
  return 1;
}

void setup() {
  // Initialize motor pins
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
  hc05.begin(9600);
  cmos.begin(9600);
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println(F("[OUTPUT][PROCESS] Starting Output Module Initialization (Uno)..."));
  
  // Handshake with CMOS module (TOF sensor)
  while (!handshakeComplete) {
    handshakeProcedure();
    if (!handshakeComplete) {
      Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Retrying handshake in 1 second..."));
      delay(1000);
    }
  }
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Handshake complete with TOF module."));
  
  // Handshake with CV module
  while (!cvHandshakeComplete) {
    cvHandshakeProcedure();
    if (!cvHandshakeComplete) {
      Serial.println(F("[OUTPUT][HANDSHAKE][CV] Retrying handshake with CV module in 1 second..."));
      delay(1000);
    }
  }
  Serial.println(F("[OUTPUT][HANDSHAKE][CV] Handshake complete with CV module. Starting main loop."));
}

void loop() {
  // --- Check and process incoming CMOS data (TOF module) ---
  if (cmos.available()) {
    Serial.println(F("[OUTPUT][INFO] Data detected on CMOS stream (TOF module)."));
    String incoming = cmos.readStringUntil('\n');
    incoming.trim();
    // Check message type by prefix
    if (incoming.startsWith("SENS:")) {
      String sensorData = incoming.substring(5);
      sensorData.trim();
      lastToFSensorData = sensorData;
      Serial.print(F("[OUTPUT][SENSOR][TOF] Sensor data received: "));
      Serial.println(sensorData);
    } else if (incoming.startsWith("LOG:")) {
      String logMsg = incoming.substring(4);
      logMsg.trim();
      Serial.print(F("[OUTPUT][SENSOR][TOF] Log from TOF module: "));
      Serial.println(logMsg);
    } else {
      Serial.print(F("[OUTPUT][SENSOR][TOF] Unknown message from TOF module: "));
      Serial.println(incoming);
    }
  }

  // --- Check and process incoming CV module data (USB Serial) ---
  if (Serial.available() > 0) {
    Serial.println(F("[OUTPUT][INFO] Data detected on USB Serial (CV module)."));
    String class_byte = Serial.readStringUntil('\n');
    class_byte.trim();
    Serial.print(F("[OUTPUT][DATA][CV] Received CV data: "));
    Serial.println(class_byte);
    
    // Parse CV data into an array
    int classes[5];
    int idx_class = 0;
    int spaceIndex_class = class_byte.indexOf(' ');
    while (spaceIndex_class >= 0 && idx_class < 5) {
      String classStr = class_byte.substring(0, spaceIndex_class);
      classes[idx_class++] = classStr.toInt();
      Serial.print(F("[OUTPUT][DATA][CV] Parsed class["));
      Serial.print(idx_class - 1);
      Serial.print(F("]: "));
      Serial.println(classStr);
      class_byte = class_byte.substring(spaceIndex_class + 1);
      spaceIndex_class = class_byte.indexOf(' ');
    }
    if (idx_class < 5) {
      classes[idx_class++] = class_byte.toInt();
      Serial.print(F("[OUTPUT][DATA][CV] Parsed class["));
      Serial.print(idx_class - 1);
      Serial.print(F("]: "));
      Serial.println(class_byte);
    }
    
    // Make sure we have valid sensor data from the TOF module
    if (lastToFSensorData.length() == 0) {
      Serial.println(F("[OUTPUT][HANDSHAKE] No TOF sensor (CMOS) distance data available."));
      return;
    }
    String dis_byte = lastToFSensorData;
    Serial.print(F("[OUTPUT][HANDSHAKE] Using last TOF sensor distance data: "));
    Serial.println(dis_byte);
    
    // Parse the distance data
    int dis[3];
    int idx_dis = 0;
    int spaceIndex_dis = dis_byte.indexOf(' ');
    while (spaceIndex_dis >= 0 && idx_dis < 3) {
      String disStr = dis_byte.substring(0, spaceIndex_dis);
      dis[idx_dis++] = disStr.toInt();
      Serial.print(F("[OUTPUT][SENSOR][TOF] Parsed distance["));
      Serial.print(idx_dis - 1);
      Serial.print(F("]: "));
      Serial.println(disStr);
      dis_byte = dis_byte.substring(spaceIndex_dis + 1);
      spaceIndex_dis = dis_byte.indexOf(' ');
    }
    if (idx_dis < 3) {
      dis[idx_dis++] = dis_byte.toInt();
      Serial.print(F("[OUTPUT][SENSOR][TOF] Parsed distance["));
      Serial.print(idx_dis - 1);
      Serial.print(F("]: "));
      Serial.println(dis_byte);
    }
    
    // Compute scores using the parsed CV classes and TOF distances
    int* scores = getWeights(classes, dis);
    if (scores == NULL) {
      Serial.println(F("[OUTPUT][HANDSHAKE] Memory allocation failed."));
      return;
    }
    
    // Determine the highest score
    int maxScore = 0;
    int maxScoreId = 0;
    if (areAllScoresZero(scores)) {
      Serial.println(F("[OUTPUT][MOTOR] All scores are zero. Executing default motor logic."));
      motorLogic(-1);
    } else {
      for (int i = 1; i < 5; i++) {
        if (scores[i] > maxScore) {
          maxScore = scores[i];
          maxScoreId = i;
        }
      }
      Serial.print(F("[OUTPUT][MOTOR] Highest score is "));
      Serial.print(maxScore);
      Serial.print(F(" at index "));
      Serial.println(maxScoreId);
      motorLogic(maxScoreId);
    }
    
    // Prepare and send message via HC-05 (with clear prefixes)
    char scoresString[50] = "";
    for (int i = 0; i < 5; i++) {
      char temp[10];
      sprintf(temp, "%d", scores[i]);
      strcat(scoresString, temp);
      if (i < 4) {
        strcat(scoresString, ",");
      }
    }
    String message = String("CV_DATA: ") + class_byte + ",SCORES:" + scoresString;
    hc05.println(message);
    Serial.print(F("[OUTPUT][HC-05] Sent message: "));
    Serial.println(message);
  }

  // --- Process HC-05 commands ---
  if (hc05.available()) {
    String receivedData = hc05.readStringUntil('\n');
    receivedData.trim();
    Serial.print(F("[OUTPUT][HC-05] Received: "));
    Serial.println(receivedData);
    if (receivedData == "motors") {
      Serial.println(F("[OUTPUT][MOTOR] Received 'motors' command from HC-05."));
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
}
