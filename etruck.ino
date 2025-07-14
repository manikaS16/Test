#include <Servo.h>

Servo esc;
const int escPin = A0;

void setup() {
  Serial.begin(9600);
  esc.attach(escPin);
  delay(2000);    // Wait 2 seconds for ESC to initialize
  stopMotor();
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();
    angle = constrain(angle, 0, 180);
    moveMotor(angle);
  }
}

void moveMotor(int angle) {
  esc.write(angle);
}

void stopMotor() {
  esc.write(90);
}
