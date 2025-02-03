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

int* getWeights(int classes[5], int distance[3]) {
  int* scores = malloc(5 * sizeof(int)); // Dynamically allocate memory for the scores array
  if (scores == NULL) {  // Always check if malloc was successful
      return NULL; // If memory allocation fails, return NULL
  }
  int current
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
}


void loop() {
  if (Serial.available() > 0) { //data from comvis incoming
    String class_byte = Serial.readStringUntil('\n'); // Read computer vision data until newline
    class_byte.trim();
    int classes[5];
    int idx_class = 0;
    int spaceIndex_class = input.indexOf(' ');

    // Store string values into classes array
    while (spaceIndex_class >= 0) {
      String classStr = class_byte.substring(0, spaceIndex_class);  
      classes[idx_class++] = classStr.toInt();                  
      input = class_byte.substring(spaceIndex_class + 1);            
      spaceIndex_class = class_byte.indexOf(' ');                    
    }

    if(cmos.available() > 0){ // data from cmos incoming
      String dis_byte = cmos.readStringUntil('\n'); // Read CMOS data until newline
      dis_byte.trim();
      int dis[3];
      int idx_dis = 0;
      int spaceIndex_dis = input.indexOf(' ');

      // Store string values into dis array
      while (spaceIndex_dis >= 0) {
        String disStr = dis_byte.substring(0, spaceIndex_dis);  
        dis[idx_dis++] = disStr.toInt();                  
        input = dis_byte.substring(spaceIndex_dis + 1);            
        spaceIndex_dis = dis_byte.indexOf(' ');                    
      }

      // get class scores
      scores = getWeights(classes, dis)

      // get highest score
      int maxScore = 0;
      int maxScoreId = 0;
      if (areAllScoresZero(scores)){ // check if no object is detected
        motorLogic(-1);
      }else{
        for (int i = 1; i < 5; i++) {  
          if (scores[i] > maxScore) {  
              maxScore = scores[i];
              maxScoreId = i;
          }
        }
        motorLogic(maxScoreId);
      }

      // send information to mobile
      // Convert the scores array to a comma-separated string
      char scoresString[50] = "";
      for (int i = 0; i < 5; i++) {
        char temp[10];  // Temporary buffer for each number
        sprintf(temp, "%d", scores[i]);  // Convert the number to a string
        strcat(scoresString, temp);     // Append the number to the scoresString

        if (i < 4) {  // Add a comma after each number except the last
            strcat(scoresString, ",");
        }
      }

      String message = class_byte;
      // Append the scores string to the existing string
      message += ",";       // Add a comma to separate from the existing string
      message += scoresString;  // Append the scores string
//       message += "\n"; // append new line
      hc05.println(message);// Send the message to HC-05
    }
  }

    if (hc05.available()) {
        String receivedData = hc05.readString();  // Read full message
        receivedData.trim();  // Remove trailing newline or spaces
        if (receivedData == "motors") {
          for (int i = 0; i< 3;i++){ // repeat 3 times
            for (int i = 0; i < 6; i++) {
              pinMode(motorPins[i], OUTPUT);
              digitalWrite(motorPins[i], HIGH);  
            }
            delay(500)
            for (int i = 0; i < 6; i++) {
              pinMode(motorPins[i], OUTPUT);
              digitalWrite(motorPins[i], LOW);  
            }
            delay(500)
          }
        }
    }


}

