#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "xbox_controller.h"

IMUManager imu;

namespace {

constexpr float kBalancePitchErrorLimitDeg = 6.0f;
constexpr float kBalanceYawRateLimitDegPerSec = 120.0f;
constexpr unsigned long kStartupPoseSettleMs = 1200UL;
constexpr unsigned long kStartupPoseUpdateDelayMs = 5UL;
constexpr unsigned long kStartupPosePrintIntervalMs = 200UL;

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

// Requires both joint motors to be near zero pose before startup continues.
bool checkJointZeroPoseOnStartup() {
  const float joint1RawTurns = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
  const float joint2RawTurns = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
  const float toleranceTurns = AppConfig::Motor::kStartupZeroPoseToleranceTurns;

  const bool joint1AtZero = fabsf(joint1RawTurns) <= toleranceTurns;
  const bool joint2AtZero = fabsf(joint2RawTurns) <= toleranceTurns;
  if (joint1AtZero && joint2AtZero) {
    return true;
  }

  Serial.println("Joint zero-pose check failed.");
  Serial.print("Expected |m1|,|m2| <= ");
  Serial.print(toleranceTurns, 3);
  Serial.println(" turns before startup.");
  Serial.print("Current m1=");
  Serial.print(joint1RawTurns, 3);
  Serial.print(", m2=");
  Serial.println(joint2RawTurns, 3);
  Serial.println("Place robot at zero pose and reboot.");
  return false;
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

  if (AppConfig::Behavior::kRequireJointZeroPoseOnStartup) {
    if (!checkJointZeroPoseOnStartup()) {
      while (true) {
        delay(100);
      }
    }
  } else {
    Serial.println("Skipping joint zero-pose check (kRequireJointZeroPoseOnStartup=false)");
  }

  Serial.print("Moving joints from zero pose to default offset: ");
  Serial.print(AppConfig::Motor::kDefaultFromZero * AppConfig::Motor::kRadToDeg, 2);
  Serial.println(" deg");
  if (!MotorControl::initializeRobotPose(AppConfig::Motor::kDefaultJointAngle)) {
    Serial.println("Robot default-pose initialization failed.");
    while (true) {
      delay(100);
    }
  }

  // Drive startup pose commands for a short window before enforcing hard limits.
  Serial.println("Settling joints to default pose...");
  const unsigned long settleStartMs = millis();
  unsigned long lastSettlePrintMs = settleStartMs;
  while (millis() - settleStartMs < kStartupPoseSettleMs) {
    MotorControl::update();
    delay(kStartupPoseUpdateDelayMs);
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
    const float wheel3VelRaw = MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorLeftNodeId);
    const float wheel4VelRaw = MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorRightNodeId);
    const float wheel3VelLogical = wheel3VelRaw * AppConfig::Motor::kWheelLeftSign;
    const float wheel4VelLogical = wheel4VelRaw * AppConfig::Motor::kWheelRightSign;
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

    Serial.print("Wheels[M3_raw:");
    Serial.print(wheel3VelRaw, 3);
    Serial.print(", M4_raw:");
    Serial.print(wheel4VelRaw, 3);
    Serial.print(", M3_logical:");
    Serial.print(wheel3VelLogical, 3);
    Serial.print(", M4_logical:");
    Serial.print(wheel4VelLogical, 3);
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
