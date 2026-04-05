#ifndef TEST_H
#define TEST_H

#include <Arduino.h>

#include "config.h"
#include "motor.h"

namespace TestMode {

// Rotates both joint motors to the standard position using the default-offset frame.
// 1. Command zero pose and wait until both raw driver readings are near 0 turns.
// 2. Command motor 1 to standard offset from default pose; motor 2 mirrors negative.
// 3. Wait until motor 1 reaches the standard pose.
bool rotateToStandardPosition(unsigned long timeoutMs = 5000) {
  Serial.println("\n========== MOTOR CALIBRATION: ROTATE TO STANDARD POSITION ==========");

  Serial.println("Step 1: Moving to zero pose (raw driver reading near 0 turns)...");
  MotorControl::setMirroredLegJointAngles(0.0f);
  MotorControl::setWheelTorques(0.0f, 0.0f);

  const unsigned long zeroStartMs = millis();
  const float zeroToleranceTurns = 0.05f;
  bool zeroReached = false;

  while (millis() - zeroStartMs < 3000) {
    MotorControl::update();

    const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
    const float motor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);

    if (fabsf(motor1Pos) <= zeroToleranceTurns && fabsf(motor2Pos) <= zeroToleranceTurns) {
      zeroReached = true;
      Serial.println("  ✓ Reached zero position");
      break;
    }

    delay(20);
  }

  if (!zeroReached) {
    Serial.println("  ✗ Failed to reach zero position, continuing anyway...");
  }

  Serial.print("Step 2: Moving motor 1 to standard offset from default (+");
  Serial.print(AppConfig::Motor::kStandardJointAngleDeg, 1);
  Serial.println(" deg; motor 2 mirrors negative)");

  MotorControl::setMirroredLegJointAngles(AppConfig::Motor::kStandardJointAngle);
  MotorControl::setWheelTorques(0.0f, 0.0f);

  const unsigned long startMs = millis();
  unsigned long lastPrintMs = startMs;
  bool targetReached = false;

  Serial.println("Starting positions: raw m1-pos:0.000, m2-pos:0.000 (zero pose)");

  while (millis() - startMs < timeoutMs) {
    MotorControl::update();

    const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
    const float motor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
    const float motor1AngleDeg = MotorControl::getJointAngleDeg(AppConfig::Motor::kJointMotorLeftNodeId);
    const float angleErrorDeg = fabsf(motor1AngleDeg - AppConfig::Motor::kStandardJointAngleDeg);

    if (millis() - lastPrintMs >= 500) {
      lastPrintMs = millis();
      Serial.print("  [");
      Serial.print(millis() - startMs);
      Serial.print("ms] m1-pos:");
      Serial.print(motor1Pos, 3);
      Serial.print(", m2-pos:");
      Serial.print(motor2Pos, 3);
      Serial.print(", m1-angle:");
      Serial.print(motor1AngleDeg, 2);
      Serial.print(" deg, Error:");
      Serial.print(angleErrorDeg, 2);
      Serial.println(" deg");
    }

    if (angleErrorDeg <= 2.0f) {
      targetReached = true;
      break;
    }

    delay(20);
  }

  const float motor1Final = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
  const float motor2Final = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
  const float finalJointAngleDeg = MotorControl::getJointAngleDeg(AppConfig::Motor::kJointMotorLeftNodeId);

  Serial.print("Motor calibration result: ");
  if (targetReached) {
    Serial.println("SUCCESS");
    Serial.print("Final joint angle: ");
    Serial.print(finalJointAngleDeg, 2);
    Serial.println(" deg");
    Serial.print("Motor positions - m1-pos:");
    Serial.print(motor1Final, 3);
    Serial.print(", m2-pos:");
    Serial.println(motor2Final, 3);
  } else {
    Serial.println("TIMEOUT - target not reached");
    Serial.print("Final joint angle: ");
    Serial.print(finalJointAngleDeg, 2);
    Serial.print(" deg (target: ");
    Serial.print(AppConfig::Motor::kStandardJointAngleDeg, 2);
    Serial.println(" deg)");
  }

  Serial.println("====================================================================\n");
  return targetReached;
}

} // namespace TestMode

#endif // TEST_H
