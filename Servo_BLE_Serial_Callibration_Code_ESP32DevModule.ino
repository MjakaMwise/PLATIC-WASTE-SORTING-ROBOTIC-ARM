#include <Servo.h>

Servo myServo;

// === Parameters ===
const int SERVO_PIN = 9;    // Servo control pin

void setup() {
  myServo.attach(SERVO_PIN);
  myServo.write(0); // Start at 0° (optional)
  delay(500);
}

void loop() {
  // Example usage
  moveToAngle(150, 10, 100);   // Move to 150° in 10° steps, 100ms delay per step
  delay(2000);

  moveToAngle(45, 5, 50);      // Move to 45° in 5° steps, 50ms delay per step
  delay(2000);
}

// === Function: move in steps without tracking ===
void moveToAngle(int targetAngle, int stepSize, int stepDelay) {
  // Clamp target angle to servo range (0–180)
  targetAngle = constrain(targetAngle, 0, 180);

  // Get current servo position
  int currentAngle = myServo.read();

  // Determine direction
  int direction = (targetAngle > currentAngle) ? 1 : -1;

  // Move gradually
  while (currentAngle != targetAngle) {
    currentAngle += direction * stepSize;

    // Avoid overshooting
    if ((direction == 1 && currentAngle > targetAngle) ||
        (direction == -1 && currentAngle < targetAngle)) {
      currentAngle = targetAngle;
    }

    myServo.write(currentAngle);
    delay(stepDelay);
  }
}
