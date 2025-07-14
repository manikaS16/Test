#include <Servo.h>

Servo esc;      // ESC for motor
Servo steer;    // Servo for steering

const int escPin = A0;
const int steerPin = A3;

void armESC() {
  esc.write(90);    // Neutral signal
  delay(2000);      // Wait 2 seconds

  esc.write(135);   // Full throttle (or max allowed by your ESC)
  delay(1000);

  esc.write(90);    // Back to neutral
  delay(1000);
}

void setup() {
  Serial.begin(9600);
  esc.attach(escPin);
  steer.attach(steerPin);
  
  armESC();
  
  stopMotor();
  centerSteer();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read until newline

    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      String motorStr = input.substring(0, commaIndex);
      String steerStr = input.substring(commaIndex + 1);

      int motorAngle = motorStr.toInt();
      int steerAngle = steerStr.toInt();

      motorAngle = constrain(motorAngle, 45, 135);   // safe range
      steerAngle = constrain(steerAngle, 45, 135);

      moveMotor(motorAngle);
      moveSteer(steerAngle);
    }
  }
}

void moveMotor(int angle) {
  esc.write(angle);
}

void stopMotor() {
  esc.write(90);   // Neutral ESC position to stop motor
}

void moveSteer(int angle) {
  steer.write(angle);
}

void centerSteer() {
  steer.write(90);  // Center steering servo
}
