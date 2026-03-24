#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "xbox_controller.h"

IMUManager imu;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }

  imu.begin();
  XboxController::begin();

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

  RobotControl::begin();
  Serial.println("Control pipeline ready: IMU/Xbox -> PID -> Motor");
}

void loop() {
  static unsigned long lastMonitorPrintMs = 0;

  imu.update();
  MotorControl::update();
  XboxController::update();
  RobotControl::process(imu.data(),
                        XboxController::isConnected(),
                        XboxController::getData(),
                        XboxController::getTimeSinceLastPacket(),
                        XboxController::getErrorCount());

  const ControlDebugState& dbg = RobotControl::debugState();

  if (MotorControl::hasNewFeedback()) {
    // Keep the feedback path hot for latest telemetry.
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastMonitorPrintMs >= AppConfig::Behavior::kMonitorPrintIntervalMs) {
    lastMonitorPrintMs = nowMs;
    Serial.print("drive:");
    Serial.print(dbg.driveEnabled ? 1 : 0);
    Serial.print(", xbox:");
    Serial.print(dbg.xboxConnected ? 1 : 0);
    Serial.print(", pitch:");
    Serial.print(imu.data().pitchDeg, 2);
    Serial.print(", ltrq:");
    Serial.print(dbg.leftTorqueCmd, 3);
    Serial.print(", rtrq:");
    Serial.print(dbg.rightTorqueCmd, 3);
    Serial.print(", leg:");
    Serial.print(dbg.legAngleCmd, 3);
    Serial.print(", dt:");
    Serial.print(dbg.dtSeconds, 4);
    Serial.print(", p-term:");
    Serial.print(dbg.pitchTerm, 3);
    Serial.print(", g-term:");
    Serial.print(dbg.gyroTerm, 3);
    Serial.print(", s-term:");
    Serial.print(dbg.speedTerm, 3);
    Serial.print(", rsx:");
    Serial.print(dbg.rightStickX);
    Serial.print(", rsy:");
    Serial.print(dbg.rightStickY);
    Serial.print(", lsy:");
    Serial.print(dbg.leftStickY);
    Serial.print(", btn:");
    Serial.print(dbg.buttons, HEX);
    Serial.print(", xbox-age:");
    Serial.print(dbg.xboxAgeMs);
    Serial.print(", xbox-err:");
    Serial.print(dbg.xboxErrorCount);
    Serial.print(", can-tx:");
    Serial.print(MotorControl::getTxCount());
    Serial.print(", can-rx:");
    Serial.print(MotorControl::getRxCount());
    Serial.print(", tx-ok:");
    Serial.print(MotorControl::lastTxSucceeded() ? 1 : 0);
    Serial.print(", fb-ever:");
    Serial.println(MotorControl::hasFeedbackEver() ? 1 : 0);
  }

  delay(AppConfig::Behavior::kMainLoopDelayMs);
}
