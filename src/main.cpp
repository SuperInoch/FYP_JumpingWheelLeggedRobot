#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "test.h"
#include "xbox_controller.h"

IMUManager imu;

namespace {

constexpr float kBalancePitchErrorLimitDeg = 6.0f;
constexpr float kBalanceYawRateLimitDegPerSec = 120.0f;

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

  if (!imu.begin()) {
    Serial.println("IMU initialization failed (MPU6050 not detected on I2C).");
    Serial.println("Check wiring: VCC, GND, SDA, SCL and IMU power.");
    while (true) {
      delay(200);
    }
  }
  Serial.println("IMU initialization OK.");
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

    if (!MotorControl::initializeRobotPose(0.0f)) {
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

  const bool imuOk = imu.update();
  MotorControl::update();
  XboxController::update();

  const XboxControllerData& xboxData = XboxController::getData();
  const bool aPressed = (xboxData.buttons & (1U << AppConfig::XboxController::kButtonABit)) != 0;
  if (aPressed && !prevAPressed) {
    Serial.print("Sneak! joint-fb m1=");
    Serial.print(MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId), 3);
    Serial.print(", m2=");
    Serial.println(MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId), 3);
  }
  if (!aPressed && prevAPressed) {
    Serial.print("jump! joint-fb m1=");
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
    const float joint1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
    const float joint2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
    const float wheel3Vel = MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorLeftNodeId);
    const float wheel4Vel = MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorRightNodeId);
    const ImuData& imuData = imu.data();

    const float pitchErrorDeg = fabsf(imuData.pitchDeg - dbg.desiredPitchDeg);
    const bool balanced = imuOk &&
                          (pitchErrorDeg <= kBalancePitchErrorLimitDeg) &&
                          (fabsf(imuData.yawRateDegPerSec) <= kBalanceYawRateLimitDegPerSec);

    Serial.print("Controller[L_X:");
    Serial.print(xboxData.leftStickX);
    Serial.print(", L_Y:");
    Serial.print(xboxData.leftStickY);
    Serial.print(", A:");
    Serial.print(aPressed ? 1 : 0);
    Serial.print("] ");

    Serial.print("Joints[M1_pos:");
    Serial.print(joint1Pos, 3);
    Serial.print(", M2_pos:");
    Serial.print(joint2Pos, 3);
    Serial.print("] ");

    Serial.print("Wheels[M3_vel:");
    Serial.print(wheel3Vel, 3);
    Serial.print(", M4_vel:");
    Serial.print(wheel4Vel, 3);
    Serial.print("] ");

    Serial.print("IMU[accelX:");
    Serial.print(imuData.accelX);
    Serial.print(", accelY:");
    Serial.print(imuData.accelY);
    Serial.print(", accelZ:");
    Serial.print(imuData.accelZ);
    Serial.print(", pitch:");
    Serial.print(imuData.pitchDeg, 2);
    Serial.print(", roll:");
    Serial.print(imuData.rollDeg, 2);
    Serial.print(", yawRate:");
    Serial.print(imuData.yawRateDegPerSec, 2);
    Serial.print("] ");

    Serial.print("Balanced[");
    Serial.print(balanced ? "YES" : "NO");
    Serial.print(", pitchErr:");
    Serial.print(pitchErrorDeg, 2);
    Serial.println("]");
  }

  delay(AppConfig::Behavior::kMainLoopDelayMs);
}
