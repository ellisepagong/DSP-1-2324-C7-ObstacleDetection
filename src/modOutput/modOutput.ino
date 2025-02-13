/*
   Output Module Sketch for Arduino Uno
   Modified to:
     • Distinguish handshake messages (prefixed with "HS:")
       from sensor data (prefixed with "DATA:")
     • Request a handshake if no sensor data is received for a timeout.
*/

#include <SoftwareSerial.h>

// Reassign RX and TX pins for HC-05 and CMOS module
#define HC05_RX 2 // connect to HC-05 TX pin
#define HC05_TX 3 // connect to HC-05 RX pin
#define CMOS_RX 5 // connect to Mega TX1 (CMOS module RX)
#define CMOS_TX 6 // connect to Mega RX1 (CMOS module TX)

const int motorPins[] = {8, 9, 10, 11, 12, 13};
int prev = 0;

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

// --- Handshake Procedure for ToF Module (via CMOS) ---
void handshakeProcedure() {
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Initiating handshake with ToF module..."));
  
  // Send handshake request to the Mega.
  cmos.println("HS:REQ_HANDSHAKE");
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Sent handshake request: HS:REQ_HANDSHAKE"));
  
  unsigned long startTime = millis();
  bool handshakeReceived = false;
  
  while ((millis() - startTime < 10000) && !handshakeReceived) {
    if (cmos.available() > 0) {
      String incoming = cmos.readStringUntil('\n');
      incoming.trim();
      Serial.print(F("[OUTPUT][HANDSHAKE][TOF] Received: "));
      Serial.println(incoming);
      if (incoming.equals("HS:HELLO_TOF")) {
         cmos.println("HS:HELLO_OUTPUT");
         Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Sent: HS:HELLO_OUTPUT"));
         handshakeReceived = true;
         handshakeComplete = true;
      }
      else {
         Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Received unknown message."));
      }
    }
    delay(50);
  }
  
  if (!handshakeReceived) {
    Serial.println(F("[OUTPUT][HANDSHAKE][TOF] No handshake received within timeout."));
  }
}

// --- Handshake Procedure for CV Module (via USB Serial) ---
void cvHandshakeProcedure() {
  Serial.println("[OUTPUT][HANDSHAKE][CV] Waiting for handshake from CV module...");
  unsigned long startTime = millis();
  bool handshakeReceived = false;
  
  while ((millis() - startTime < 10000) && !handshakeReceived) {
    if (Serial.available() > 0) {
      String incoming = Serial.readStringUntil('\n');
      incoming.trim();
      Serial.print("[OUTPUT][HANDSHAKE][CV] Received handshake message: ");
      Serial.println(incoming);
      if (incoming.equals("HELLO_OUTPUT")) {
         Serial.println("HELLO_CV");
         Serial.println("[OUTPUT][HANDSHAKE][CV] Sent handshake response: HELLO_CV");
         handshakeReceived = true;
         cvHandshakeComplete = true;
      } else {
         Serial.println("[OUTPUT][HANDSHAKE][CV] Received unknown handshake message.");
      }
    }
    delay(50);
  }
  
  if (!handshakeReceived) {
    Serial.println("[OUTPUT][HANDSHAKE][CV] Handshake with CV module timed out.");
  }
}

// --- Other Procedures ---
int* getWeights(int classes[5], int distance[3]) {
  int* scores = (int*)malloc(5 * sizeof(int));
  if (scores == NULL) {
      return NULL;
  }
  int current, dis, base_w, dis_w, dir_w, prox_score, score;
  
  for (int i = 0; i < 5; i++) {
    base_w = 0; dis_w = 0; dir_w = 0; prox_score = 0; score = 0;
    current = classes[i];
    
    if (current >= 0 && current <= 9) {
      base_w = weights[current + 1];
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
        return -1;
      }
      
      dis_w = map(dis, 0, 200, 5, 1);
      score = (dis_w * base_w) + dir_w;
      if (dis_w > 3) { score += 5; }
      scores[i] = score;
    }
    if (current == -1) {
      scores[i] = 0;
    }
    else {
      scores[i] = -1;
    }
  }
  return scores;
}

void motorLogic(int segment){
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
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);  
  }
  hc05.begin(9600);
  cmos.begin(9600);
  Serial.begin(9600); 
  while (!Serial) { ; }
  Serial.println(F("[OUTPUT][PROCESS] Starting Output Module Initialization (Uno)..."));
    
  // Perform initial handshake with the ToF module.
  while (!handshakeComplete) {
      handshakeProcedure();
      if (!handshakeComplete) {
        Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Retrying handshake in 1 second..."));
        delay(1000);
      }
  }
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Handshake complete with ToF module."));
    
  // Perform handshake with CV module (via USB Serial).
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
  // --- Process incoming data from the CMOS (Mega) ---
  if (cmos.available()) {
    String incoming = cmos.readStringUntil('\n');
    incoming.trim();
    
    // If sensor data, check for the "DATA:" prefix.
    if (incoming.startsWith("DATA:")) {
      String sensorData = incoming.substring(5); // remove the prefix
      Serial.print(F("[OUTPUT][SENSOR][TOF] Data received: "));
      Serial.println(sensorData);
    }
  }
  
  // --- Check for lost sensor data ---
  // If no sensor data is received for >3 seconds, request a new handshake.
  static unsigned long lastSensorTime = millis();
  if (cmos.available()) {
    lastSensorTime = millis();
  }
  if (millis() - lastSensorTime > 3000) { // 3-second timeout
    handshakeComplete = false;
    Serial.println(F("[OUTPUT][HANDSHAKE][TOF] No sensor data for 3 seconds, re-initiating handshake..."));
    handshakeProcedure();
    lastSensorTime = millis();
  }
  
  // --- Process data from CV module (via USB Serial) ---
  if (Serial.available() > 0) {
    String class_byte = Serial.readStringUntil('\n'); 
    class_byte.trim();
    Serial.print(F("[OUTPUT][DATA][CV] Received CV data: "));
    Serial.println(class_byte);
    
    int classes[5];
    int idx_class = 0;
    int spaceIndex_class = class_byte.indexOf(' ');
    while (spaceIndex_class >= 0) {
      String classStr = class_byte.substring(0, spaceIndex_class);  
      classes[idx_class++] = classStr.toInt();                  
      class_byte = class_byte.substring(spaceIndex_class + 1);            
      spaceIndex_class = class_byte.indexOf(' ');                    
    }
    
    if(cmos.available()){
      String dis_byte = cmos.readStringUntil('\n');
      dis_byte.trim();
      Serial.print(F("[OUTPUT][HANDSHAKE] Received CMOS distance data: "));
      Serial.println(dis_byte);
      int dis[3];
      int idx_dis = 0;
      int spaceIndex_dis = dis_byte.indexOf(' ');
      while (spaceIndex_dis >= 0) {
        String disStr = dis_byte.substring(0, spaceIndex_dis);  
        dis[idx_dis++] = disStr.toInt();                  
        dis_byte = dis_byte.substring(spaceIndex_dis + 1);            
        spaceIndex_dis = dis_byte.indexOf(' ');                    
      }
      
      int* scores = getWeights(classes, dis);
      if (scores == NULL) {
          Serial.println(F("[OUTPUT][HANDSHAKE] Memory allocation failed."));
          return;
      }
      
      int maxScore = 0;
      int maxScoreId = 0;
      if (areAllScoresZero(scores)){
        motorLogic(-1);
      } else {
        for (int i = 1; i < 5; i++) {  
          if (scores[i] > maxScore) {  
              maxScore = scores[i];
              maxScoreId = i;
          }
        }
        motorLogic(maxScoreId);
      }
      
      Serial.print(F("[OUTPUT][HANDSHAKE] Motor logic executed for segment: "));
      Serial.println(maxScoreId);
      
      char scoresString[50] = "";
      for (int i = 0; i < 5; i++) {
        char temp[10];
        sprintf(temp, "%d", scores[i]);
        strcat(scoresString, temp);
        if (i < 4) { strcat(scoresString, ","); }
      }
      
      String message = class_byte;
      message += ",";
      message += scoresString;
      hc05.println(message);
      Serial.print(F("[OUTPUT][HANDSHAKE] Sent a message to HC-05 message"));
    }
  }
  
  if (hc05.available()) {
    String receivedData = hc05.readStringUntil('\n');
    receivedData.trim();
    Serial.print(F("[OUTPUT][HC-05] Received: "));
    Serial.println(receivedData);
    if (receivedData == "motors") {
      Serial.println(F("[OUTPUT][HANDSHAKE] Received 'motors' command from HC-05."));
      for (int i = 0; i < 3; i++){
        for (int j = 0; j < 6; j++) {
          pinMode(motorPins[j], OUTPUT);
          digitalWrite(motorPins[j], HIGH);  
        }
        delay(500);
        for (int j = 0; j < 6; j++) {
          pinMode(motorPins[j], OUTPUT);
          digitalWrite(motorPins[j], LOW);  
        }
        delay(500);
      }
    }
  }
}
