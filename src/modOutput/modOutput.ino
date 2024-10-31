int x;

const int motorPins[] = {3, 4, 5, 6, 7, 8};  // Motor Pins

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);  // Start with all motors off
  }
}

void  loop() {
  while (!Serial.available()); 
  String input = Serial.readString(); 
  x = input.toInt(); // Convert the string to an integer

  // Turn off all motors before setting new states
  for (int i = 0; i < 6; i++) {
    digitalWrite(motorPins[i], LOW); 
  }
  
  if (x == 1) { // left
    digitalWrite(motorPins[0], HIGH);
    digitalWrite(motorPins[1], HIGH);
  } else if (x == 2) { //front left
    digitalWrite(motorPins[1], HIGH);
    digitalWrite(motorPins[2], HIGH);
  } else if (x == 3) { //front
    digitalWrite(motorPins[2], HIGH);
    digitalWrite(motorPins[3], HIGH);
  }else if (x == 4) { // front right
    digitalWrite(motorPins[3], HIGH);
    digitalWrite(motorPins[4], HIGH);
  }else if (x == 5) { //right
    digitalWrite(motorPins[4], HIGH);
    digitalWrite(motorPins[5], HIGH);
  }
}