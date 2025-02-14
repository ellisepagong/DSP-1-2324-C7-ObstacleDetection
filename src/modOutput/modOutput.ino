// Refer to https://docs.google.com/spreadsheets/d/1OpYB4adHl902PVdg5S_J-I7sS-uehVSkoKj3E2hwgD8/edit?usp=sharing 
// for byte formatting and class weight logic

#include <SoftwareSerial.h>

// Reassign RX and TX pins for HC-05
#define HC05_RX 2 // connect to HC-05 TX pin
#define HC05_TX 3 // connect to HC-05 RX pin
#define CMOS_RX 5 // connect to CMOS arduino TX pin
#define CMOS_TX 6 // connect to CMOS arduino RX pin

const int motorPins[] = {8, 9, 10, 11, 12, 13};
int prev = 0;

// Declare global variable to store the latest ToF sensor data.
String lastToFSensorData = "";

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
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Entering continuous stream mode..."));
  
  // Flush any stray data in the CMOS serial buffer.
  while (cmos.available()) {
    char flushed = cmos.read();
    Serial.print(F("[OUTPUT][HANDSHAKE][TOF] Flushed stray byte: "));
    Serial.println(flushed);
  }
  
  // Immediately mark handshake as complete so the module switches to continuous streaming.
  handshakeComplete = true;
  Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Continuous stream mode enabled."));
}


// --- Handshake Procedure for CV Module (via USB Serial) ---
void cvHandshakeProcedure() {
  Serial.println("[OUTPUT][HANDSHAKE][CV] Disabling handshake for CV module. Continuous mode enabled.");
  cvHandshakeComplete = true;
}


// --- Other Procedures ---
int* getWeights(int classes[5], int distance[3]) {
  int* scores = malloc(5 * sizeof(int)); // Dynamically allocate memory for the scores array
  if (scores == NULL) {  // Always check if malloc was successful
      return NULL; // If memory allocation fails, return NULL
  }
  int current;
  int dis;
  int base_w;
  int dis_w;
  int dir_w;
  int prox_score;
  int score;

  for (int i = 0; i < 5; i++) {
    // Reset values
    base_w = 0;
    dis_w = 0;
    dir_w = 0;
    prox_score = 0;
    score = 0;
    current = classes[i];

    // get base weight
    if (current >= 0 && current <= 9) {
      base_w = weights[current + 1];

      // get distance (dis) and directional weight (dir_w)
      if (i < 2){ //left 
        dis = distance[0];
        if (i == 1) {
          dir_w = 2;
        }
      } else if (i == 2){ // front
        dis = distance[1];
        dir_w = 3;
      } else if (i >= 2 && i < 5){ // right 
        dis = distance[2];
        if (i == 3) {
          dir_w = 2;
        }
      } else{
        return -1; // out of range
      }
      
      dis_w = map(dis, 0, 200, 5, 1);  // get distance weight (Smaller distance, higher weight)

      score = (dis_w * base_w) + dir_w; // base score + directional weight

      if (dis_w > 3){ // proximity adjustment
        score += 5;
      }
      
      scores[i] = score;

    } 
    if (current == -1){ // no class
      scores[i] = 0;
    }
    else {
      scores[i] = -1; // Invalid class
    }
  }
  return scores;
}

// Function to activate respective motor
void motorLogic(int segment){
  Serial.print(F("[OUTPUT][HANDSHAKE] Activating motor logic for segment: "));
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
      default: // if none of the cases match, turn all pins low
        for (int i = 0; i < 6; i++) {
          digitalWrite(motorPins[i], LOW);
        }
      break;
    }
}
int areAllScoresZero(int scores[5]) {
    for (int i = 0; i < 5; i++) {
        if (scores[i] != 0) {  // If any value is not zero
            return 0;  // Return false (0) if at least one value is not zero
        }
    }
    return 1;  // Return true (1) if all values are zero
}


void setup() { // Turn off all motors 
  // --- Setup Procedure ---
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);  
  }
  // Start serial communication at 9600 baud
  hc05.begin(9600);
  cmos.begin(9600);
  Serial.begin(9600); 
  while (!Serial) {
    ; 
  }
  Serial.println(F("[OUTPUT][PROCESS] Starting Output Module Initialization (Uno)..."));
    
    // Perform handshake with the ToF module (running on the Mega)
    while (!handshakeComplete) {
      handshakeProcedure();
      if (!handshakeComplete) {
        Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Retrying handshake in 1 second..."));
        delay(1000);
      }
    }
    Serial.println(F("[OUTPUT][HANDSHAKE][TOF] Handshake complete with ToF module."));

    // Perform handshake with CV module (via USB Serial)
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

  // --- Continuously update the latest CMOS sensor data ---
  if (cmos.available()) {
    String sensorData = cmos.readStringUntil('\n');
    sensorData.trim();
    if (sensorData.length() > 0) {
      lastToFSensorData = sensorData;  // Save the latest sensor data
      Serial.print(F("[OUTPUT][SENSOR][TOF] Data received: "));
      Serial.println(sensorData);
    }
  }

  // --- Process CV module data (from hardware Serial) ---
  if (Serial.available() > 0) {
    String class_byte = Serial.readStringUntil('\n'); // Read CV data until newline
    class_byte.trim();
    Serial.print(F("[OUTPUT][DATA][CV] Received CV data: "));
    Serial.println(class_byte);
    
    // Parse CV data into classes[]
    int classes[5];
    int idx_class = 0;
    int spaceIndex_class = class_byte.indexOf(' ');
    while (spaceIndex_class >= 0) {
      String classStr = class_byte.substring(0, spaceIndex_class);
      classes[idx_class++] = classStr.toInt();
      class_byte = class_byte.substring(spaceIndex_class + 1);
      spaceIndex_class = class_byte.indexOf(' ');
    }
    
    // Instead of reading from CMOS again, use the stored sensor data.
    if (lastToFSensorData.length() == 0) {
      Serial.println(F("[OUTPUT][HANDSHAKE] No CMOS distance data available."));
      return;
    }
    String dis_byte = lastToFSensorData;
    Serial.print(F("[OUTPUT][HANDSHAKE] Using last CMOS distance data: "));
    Serial.println(dis_byte);
    
    // Parse the distance data from dis_byte
    int dis[3];
    int idx_dis = 0;
    int spaceIndex_dis = dis_byte.indexOf(' ');
    while (spaceIndex_dis >= 0) {
      String disStr = dis_byte.substring(0, spaceIndex_dis);  
      dis[idx_dis++] = disStr.toInt();                  
      dis_byte = dis_byte.substring(spaceIndex_dis + 1);            
      spaceIndex_dis = dis_byte.indexOf(' ');
    }

    // Get class scores based on CV classes and CMOS distance data
    int* scores = getWeights(classes, dis);
    if (scores == NULL) { // Ensure malloc didn't fail
      Serial.println(F("[OUTPUT][HANDSHAKE] Memory allocation failed."));
      return;
    }

    // Determine the highest score (or take no action if all are zero)
    int maxScore = 0;
    int maxScoreId = 0;
    if (areAllScoresZero(scores)) { 
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

    // Send information to mobile via HC-05
    char scoresString[50] = "";
    for (int i = 0; i < 5; i++) {
      char temp[10];
      sprintf(temp, "%d", scores[i]);
      strcat(scoresString, temp);
      if (i < 4) {
        strcat(scoresString, ",");
      }
    }

    String message = class_byte;
    message += ",";
    message += scoresString;
    hc05.println(message);
    Serial.println(F("[OUTPUT][HANDSHAKE] Sent a message to HC-05."));
  }

  if (hc05.available()) {
      String receivedData = hc05.readStringUntil('\n');  // Read full message
      receivedData.trim();  // Remove trailing newline or spaces
      Serial.print(F("[OUTPUT][HC-05] Received: "));
      Serial.println(receivedData);
      if (receivedData == "motors") {
        Serial.println(F("[OUTPUT][HANDSHAKE] Received 'motors' command from HC-05."));
        for (int i = 0; i< 3;i++){ // repeat 3 times
          for (int i = 0; i < 6; i++) {
            pinMode(motorPins[i], OUTPUT);
            digitalWrite(motorPins[i], HIGH);  
          }
          delay(500);
          for (int i = 0; i < 6; i++) {
            pinMode(motorPins[i], OUTPUT);
            digitalWrite(motorPins[i], LOW);  
          }
          delay(500);
        }
      }
  }
}
