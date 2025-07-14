#include <Servo.h>

Servo esc;      // ESC for motor
Servo steer;    // Servo for steering

const int escPin = A0;
const int steerPin = A3;

const int ESC_MIN = 1500;  // Neutral
const int ESC_MAX = 2000;  // Full forward
const int ESC_REV = 1000;  // Full reverse (if supported)

const int STEER_LEFT = 1300;
const int STEER_CENTER = 1500;
const int STEER_RIGHT = 1700;

void setup() {
  Serial.begin(9600);
  esc.attach(escPin);
  steer.attach(steerPin);

  delay(2000);  // Let ESC initialize
  stopMotor();
  centerSteer();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Expected format: "0.8,-0.2"

    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      String motorStr = input.substring(0, commaIndex);
      String steerStr = input.substring(commaIndex + 1);

      float motorVal = motorStr.toFloat();  // from -1.0 to 1.0
      float steerVal = steerStr.toFloat();  // from -1.0 to 1.0

      motorVal = constrain(motorVal, -1.0, 1.0);
      steerVal = constrain(steerVal, -1.0, 1.0);

      // Map motorVal (-1 to 1) to 1000–2000 µs
      int pwmMotor = mapFloat(motorVal, -1.0, 1.0, ESC_REV, ESC_MAX);
      int pwmSteer = mapFloat(steerVal, -1.0, 1.0, STEER_LEFT, STEER_RIGHT);

      esc.writeMicroseconds(pwmMotor);
      steer.writeMicroseconds(pwmSteer);
    }
  }
}

void stopMotor() {
  esc.writeMicroseconds(ESC_MIN);  // Stop (neutral)
}

void centerSteer() {
  steer.writeMicroseconds(STEER_CENTER);  // Center position
}

// Helper: float map with ranges
int mapFloat(float val, float inMin, float inMax, int outMin, int outMax) {
  return (int)((val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
}
