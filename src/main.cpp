#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"

IMUManager imu;
PIDController pitchPid(AppConfig::PID::kPitchKp, AppConfig::PID::kPitchKi, AppConfig::PID::kPitchKd);

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }

  imu.begin();

  pitchPid.setOutputLimits(AppConfig::PID::kOutputMin, AppConfig::PID::kOutputMax);
  pitchPid.setOutputRamp(AppConfig::PID::kOutputRampPerSecond);

  if (!MotorControl::begin()) {
    Serial.println("Motor initialization failed.");
    while (true) {
      delay(100);
    }
  }

  if (!MotorControl::initializeRobotPose(AppConfig::Motor::kLegStartupAngleRad)) {
    Serial.println("Robot pose initialization failed.");
    while (true) {
      delay(100);
    }
  }

  // Keep non-tested motors in safe idle during single-motor bench test.
  MotorControl::setMotorPosition(AppConfig::Motor::kJointMotorRightNodeId, 0.0f, 0.0f, 0.0f, 0.0f);
  MotorControl::setWheelTorques(0.0f, 0.0f);

  Serial.println("Single motor test: motor 1 IMU pitch-based velocity command.");
}

void loop() {
  static unsigned long lastMonitorPrintMs = 0;
  static const float kPitchDeadbandDeg = 1.0f;
  static const float kVelGainTurnsPerSecPerDeg = 0.18f;
  static const float kMaxVelTurnsPerSec = 4.0f;

  imu.update();
  MotorControl::update();

  const float pitchDeg = imu.data().pitchDeg;
  float velocityCmd = 0.0f;
  if (fabsf(pitchDeg) > kPitchDeadbandDeg) {
    velocityCmd = -kVelGainTurnsPerSecPerDeg * pitchDeg;
    if (velocityCmd > kMaxVelTurnsPerSec) {
      velocityCmd = kMaxVelTurnsPerSec;
    }
    if (velocityCmd < -kMaxVelTurnsPerSec) {
      velocityCmd = -kMaxVelTurnsPerSec;
    }
  }
  MotorControl::setMotorVelocity(AppConfig::Motor::kJointMotorLeftNodeId, velocityCmd, 0.0f);

  if (MotorControl::hasNewFeedback()) {
    // Feedback is still consumed to keep freshest motor telemetry available.
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastMonitorPrintMs >= AppConfig::Behavior::kMonitorPrintIntervalMs) {
    lastMonitorPrintMs = nowMs;
    Serial.print("pitch:");
    Serial.print(pitchDeg, 2);
    Serial.print(", vel-cmd:");
    Serial.print(velocityCmd, 3);
    Serial.print(", m1-pos:");
    Serial.print(MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId), 3);
    Serial.print(", m1-vel:");
    Serial.print(MotorControl::getMotorVelocity(AppConfig::Motor::kJointMotorLeftNodeId), 3);
    Serial.print(", can-tx:");
    Serial.print(MotorControl::getTxCount());
    Serial.print(", can-rx:");
    Serial.print(MotorControl::getRxCount());
    Serial.print(", tx-ok:");
    Serial.print(MotorControl::lastTxSucceeded() ? 1 : 0);
    Serial.print(", fb-ever:");
    Serial.println(MotorControl::hasFeedbackEver() ? 1 : 0);
  }

  delay(5);
}
