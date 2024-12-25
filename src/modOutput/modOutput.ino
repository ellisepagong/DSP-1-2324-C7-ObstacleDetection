const int motorPins[] = {2, 3, 4, 5, 6, 7};
int prev = 0;

void setup() {
    // Turn off all motors before setting new states
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW); 
  }
  Serial.begin(9600); // Start serial communication at 9600 baud
  while (!Serial) {
    ; // Wait for the serial port to connect
  }
}
void loop() {
  if (Serial.available() > 0) {
    // Read incoming data
    String data = Serial.readStringUntil('\n'); // Read data until newline
    int commaIndex = data.indexOf(',');        // Locate the comma

    if (commaIndex > 0) {
      // Split the string at the comma
      String clsStr = data.substring(0, commaIndex);       // Extract `cls`
      String segmentIndexStr = data.substring(commaIndex + 1); // Extract `segment_index`

      // Convert strings to integers
      int cls = clsStr.toInt();
      int segment_index = segmentIndexStr.toInt();

      // activate motors accordingly
      if (segment_index != prev){
        switch (segment_index) {
          case 1: // left
            digitalWrite(motorPins[0], HIGH);
            digitalWrite(motorPins[1], HIGH);
            digitalWrite(motorPins[2], LOW);
            digitalWrite(motorPins[3], LOW);
            digitalWrite(motorPins[4], LOW);
            digitalWrite(motorPins[5], LOW);
            break;
          case 2: // front left
            digitalWrite(motorPins[0], LOW);
            digitalWrite(motorPins[1], HIGH);
            digitalWrite(motorPins[2], HIGH);
            digitalWrite(motorPins[3], LOW);
            digitalWrite(motorPins[4], LOW);
            digitalWrite(motorPins[5], LOW);
            break;
          case 3: // front
            digitalWrite(motorPins[0], LOW);
            digitalWrite(motorPins[1], LOW);
            digitalWrite(motorPins[2], HIGH);
            digitalWrite(motorPins[3], HIGH);
            digitalWrite(motorPins[4], LOW);
            digitalWrite(motorPins[5], LOW);
            break;
          case 4: // front right
            digitalWrite(motorPins[0], LOW);
            digitalWrite(motorPins[1], LOW);
            digitalWrite(motorPins[2], LOW);
            digitalWrite(motorPins[3], HIGH);
            digitalWrite(motorPins[4], HIGH);
            digitalWrite(motorPins[5], LOW);
            break;
          case 5: // right
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
        prev = segment_index;
      }
    }
  }
}

