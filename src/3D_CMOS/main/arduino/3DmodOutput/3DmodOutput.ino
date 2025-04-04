/*
Output Module for Luxonis OAK-D-IoT-75 AI Camera Module

Expected CV message format (space-separated):
  "groupB_segment groupB_distance groupB_class groupA_class groupA_distance groupA_segment"
For example: "4 74 5 -1 -1 -1"

This sketch:
- Continuously sends "[OM_CV_REQUEST]" (handshake) to the CV module.
- Waits for a CV response.
- Parses the six numbers.
- Selects the candidate with the lower valid distance.
- Uses the candidate’s segment to drive motor activation.
- Sends a comma-separated output (e.g., "4, 74, 5, -1, -1, -1") via Bluetooth (Serial2).
- Processes incoming Bluetooth commands.
- Prints timing logs and other debugging info via USB Serial.
*/

const int motorPins[] = {8, 9, 10, 11, 12, 13};
const int LED_IDLE = 4;  // LED for debugging (if needed)
bool Error_1_sent = false;

void motorLogic(int segment) {
  // Activate motors based on the segment value.
  switch (segment) {
    case 0:
      digitalWrite(motorPins[0], HIGH); digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], LOW);  digitalWrite(motorPins[3], LOW);
      digitalWrite(motorPins[4], LOW);  digitalWrite(motorPins[5], LOW);
      break;
    case 1:
      digitalWrite(motorPins[0], LOW);  digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], HIGH); digitalWrite(motorPins[3], LOW);
      digitalWrite(motorPins[4], LOW);  digitalWrite(motorPins[5], LOW);
      break;
    case 2:
      digitalWrite(motorPins[0], LOW);  digitalWrite(motorPins[1], HIGH);
      digitalWrite(motorPins[2], HIGH); digitalWrite(motorPins[3], LOW);
      digitalWrite(motorPins[4], LOW);  digitalWrite(motorPins[5], LOW);
      break;
    case 3:
      digitalWrite(motorPins[0], LOW);  digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], HIGH); digitalWrite(motorPins[3], HIGH);
      digitalWrite(motorPins[4], LOW);  digitalWrite(motorPins[5], LOW);
      break;
    case 4:
      digitalWrite(motorPins[0], LOW);  digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);  digitalWrite(motorPins[3], HIGH);
      digitalWrite(motorPins[4], HIGH); digitalWrite(motorPins[5], LOW);
      break;
    case 5:
      digitalWrite(motorPins[0], LOW);  digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);  digitalWrite(motorPins[3], HIGH);
      digitalWrite(motorPins[4], HIGH); digitalWrite(motorPins[5], LOW);
      break;
    case 6:
      digitalWrite(motorPins[0], LOW);  digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);  digitalWrite(motorPins[3], LOW);
      digitalWrite(motorPins[4], HIGH); digitalWrite(motorPins[5], HIGH);
      break;
    default:
      for (int i = 0; i < 6; i++) {
        digitalWrite(motorPins[i], LOW);
      }
      break;
  }
}

void setup() {
  // Initialize motor pins
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
  pinMode(LED_IDLE, OUTPUT);
  digitalWrite(LED_IDLE, LOW);
  
  // Initialize serial communications:
  Serial.begin(9600);   // USB Serial (for CV module handshake & logs)
  Serial2.begin(9600);  // Bluetooth (HC-05)
  
  Serial.println("[OUTPUT LOG] Starting Output Module for Luxonis OAK-D-IoT-75");
}

void loop() {
  unsigned long loopStart = millis();

  // Clear any stale data from USB Serial.
  while (Serial.available() > 0) {
    Serial.read();
  }
  
  // Send handshake request to CV module.
  Serial.println("[OM_CV_REQUEST]");
  
  // Wait up to 1400 ms for a CV response.
  unsigned long motorStartTime = millis();
  unsigned long waitStart = millis();
  String cvMessage = "";
  while (cvMessage.length() == 0 && (millis() - waitStart < 1400)) {
    if (Serial.available() > 0) {
      cvMessage = Serial.readStringUntil('\n');
      cvMessage.trim();
    }
  }
  
  if (cvMessage.length() > 0) {
    // Record when CV data is received.
    unsigned long cvReceivedTime = millis();
    Serial.print("[OUTPUT LOG] Received CV data: ");
    Serial.println(cvMessage);
    
    // Expected format: "groupB_segment groupB_distance groupB_class groupA_class groupA_distance groupA_segment"
    int values[6];
    int index = 0;
    int spaceIndex = cvMessage.indexOf(' ');
    while (spaceIndex >= 0 && index < 6) {
      String part = cvMessage.substring(0, spaceIndex);
      values[index++] = part.toInt();
      cvMessage = cvMessage.substring(spaceIndex + 1);
      spaceIndex = cvMessage.indexOf(' ');
    }
    if (index < 6) {
      values[index++] = cvMessage.toInt();
    }
    
    int groupB_segment = values[0];   // For candidate from Group B: segment
    int groupB_distance = values[1];    // distance
    // int groupB_class = values[2];     // candidate class (unused for motor logic)
    int groupA_class = values[3];       // For candidate from Group A: class
    int groupA_distance = values[4];    // distance
    int groupA_segment = values[5];     // segment
    
    // Decide which candidate to use (choose the candidate with the lowest valid distance).
    bool validB = (groupB_distance != -1);
    bool validA = (groupA_distance != -1);
    int selected_segment = -1;
    if (!validB && !validA) {
      selected_segment = -1;  // No valid candidate.
    } else if (validB && validA) {
      selected_segment = (groupB_distance <= groupA_distance) ? groupB_segment : groupA_segment;
    } else if (validB) {
      selected_segment = groupB_segment;
    } else if (validA) {
      selected_segment = groupA_segment;
    }
    
    Serial.print("[OUTPUT LOG] Selected segment: ");
    Serial.println(selected_segment);
    
    // --- Measure Motor Activation Duration ---
    
    motorLogic(selected_segment);
    unsigned long motorActivationDuration = millis() - motorStartTime;
    Serial.print("[TIMING] [MOTOR_ACTIVATION] ");
    Serial.print(motorActivationDuration);
    Serial.println(" ms");
    
    // Prepare final output string in comma-separated format.
    String out_groupB = validB ? String(groupB_segment) + ", " + String(groupB_distance) + ", " + String(values[2])
                                : "-1, -1, -1";
    String out_groupA = validA ? String(groupA_class) + ", " + String(groupA_distance) + ", " + String(groupA_segment)
                                : "-1, -1, -1";
    String final_output = out_groupB + ", " + out_groupA;
    
    // --- Measure CV-to-Bluetooth Duration ---
    Serial2.println(final_output);
    unsigned long cvToBtDuration = millis() - cvReceivedTime;
    Serial.print("[TIMING] [CV_TO_BT] ");
    Serial.print(cvToBtDuration);
    Serial.println(" ms");
    
    Serial.print("[OUTPUT LOG] [HC-05] Sent message: ");
    Serial.println(final_output);
  }
  else {
    // No CV data received within timeout: if 3 sec elapsed, stop motors.
    if (millis() - loopStart >= 3000) {
      Serial.println("[OUTPUT LOG] No new CV data for 3 seconds. Stopping motors.");
      if (!Error_1_sent) {
        Serial2.println("Error_1");
        Error_1_sent = true;
      }
      motorLogic(-1);
    }
  }
  
  // Process incoming commands from Bluetooth.
  if (Serial2.available()) {
    String receivedData = Serial2.readStringUntil('\n');
    receivedData.trim();
    Serial.print("[OUTPUT LOG] [HC-05] Received: ");
    Serial.println(receivedData);
    if (receivedData == "motors") {
      Serial.println("[OUTPUT LOG] [MOTOR] Received 'motors' command from HC-05.");
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
  
  unsigned long loopEnd = millis();
  unsigned long overallDuration = loopEnd - loopStart;
  Serial.print("[TIMING] [OVERALL] ");
  Serial.print(overallDuration);
  Serial.println(" ms");
  
  delay(100);
}
