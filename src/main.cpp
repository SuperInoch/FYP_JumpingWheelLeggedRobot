#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "test.h"
#include "xbox_controller.h"

IMUManager imu;

namespace {

// Blocks startup until at least one valid Xbox packet is observed.
bool waitForXboxSignal() {
  Serial.println("Waiting for Xbox signal from ESP32...");
  while (!XboxController::isConnected()) {
    XboxController::update();

    static unsigned long lastWaitPrintMs = 0;
    const unsigned long nowMs = millis();
    if (nowMs - lastWaitPrintMs >= 500) {
      lastWaitPrintMs = nowMs;
      
      const auto& data = XboxController::getData();
      Serial.print("xbox-wait[dt=");
      Serial.print(XboxController::getTimeSinceLastPacket());
      Serial.print("ms, err=");
      Serial.print(XboxController::getErrorCount());
      Serial.print("] sticks: L(");
      Serial.print(data.leftStickX);
      Serial.print(",");
      Serial.print(data.leftStickY);
      Serial.print(") R(");
      Serial.print(data.rightStickX);
      Serial.print(",");
      Serial.print(data.rightStickY);
      Serial.print(") triggers: L=");
      Serial.print(data.leftTrigger);
      Serial.print(" R=");
      Serial.print(data.rightTrigger);
      Serial.print(" buttons=");
      Serial.println(data.buttons);
    }

    delay(5);
  }

  Serial.println("Xbox signal received. Starting robot.");
  return true;
}

} // namespace

// Initializes serial, sensors, comms, motors, and control pipeline.
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }

  imu.begin();
  XboxController::begin();

  if (AppConfig::Behavior::kRequireXboxSignalOnStartup) {
    if (!waitForXboxSignal()) {
      while (true) {
        delay(100);
      }
    }
  } else {
    Serial.println("Skipping Xbox startup check (kRequireXboxSignalOnStartup=false)");
  }

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

  // Test mode: Rotate motors to standard position for mechanical calibration
  Serial.println("Starting motor calibration...");
  if (!TestMode::rotateToStandardPosition(5000)) {
    Serial.println("WARNING: Motor calibration timeout. Continuing with safety check anyway.");
  }

  // Safety check: verify joint angles are within physical limits
  if (!MotorControl::checkJointLimits()) {
    Serial.println("Joint limit check failed.");
    while (true) {
      delay(100);
    }
  }

  RobotControl::begin();
  Serial.println("Control pipeline ready: IMU/Xbox -> PID -> Motor");
}

// Runs the control loop: acquire state, process control, print telemetry.
void loop() {
  static unsigned long lastMonitorPrintMs = 0;
  static bool prevAPressed = false;

  imu.update();
  MotorControl::update();
  XboxController::update();

  const XboxControllerData& xboxData = XboxController::getData();
  const bool aPressed = (xboxData.buttons & (1U << AppConfig::XboxController::kButtonABit)) != 0;
  if (aPressed && !prevAPressed) {
    Serial.print("Jump! joint-fb m1=");
    Serial.print(MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId), 3);
    Serial.print(", m2=");
    Serial.println(MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId), 3);
  }
  prevAPressed = aPressed;

  RobotControl::process(imu.data(),
                        XboxController::isConnected(),
                        xboxData,
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
    Serial.print(", a:");
    Serial.print(aPressed ? 1 : 0);
    Serial.print(", leg-cmd:");
    Serial.print(dbg.legAngleCmd, 3);
    Serial.print(", m1-fb:");
    Serial.print(MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId), 3);
    Serial.print(", m2-fb:");
    Serial.print(MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId), 3);
    Serial.print(", m1-st:");
    Serial.print(MotorControl::getAxisState(AppConfig::Motor::kJointMotorLeftNodeId));
    Serial.print(", m2-st:");
    Serial.print(MotorControl::getAxisState(AppConfig::Motor::kJointMotorRightNodeId));
    Serial.print(", m1-err:0x");
    Serial.print(MotorControl::getAxisError(AppConfig::Motor::kJointMotorLeftNodeId), HEX);
    Serial.print(", m2-err:0x");
    Serial.print(MotorControl::getAxisError(AppConfig::Motor::kJointMotorRightNodeId), HEX);
    Serial.print(", btn:");
    Serial.print(dbg.buttons, HEX);
    Serial.print(", can-tx-fail:");
    Serial.print(MotorControl::getTxFailCount());
    Serial.print(", can-rx:");
    Serial.println(MotorControl::getRxCount());
  }

  delay(AppConfig::Behavior::kMainLoopDelayMs);
}
