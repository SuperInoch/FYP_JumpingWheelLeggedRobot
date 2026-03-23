#include <Arduino.h>

#include "config.h"
#include "imu.h"
#include "motor.h"
#include "pid.h"
#include "xbox_controller.h"

IMUManager imu;

namespace {

bool gDriveEnabled = true;
bool gPrevMenuPressed = false;
bool gPrevViewPressed = false;

constexpr uint8_t kButtonRB = 5;
constexpr uint8_t kButtonMenu = 6;
constexpr uint8_t kButtonView = 7;

PIDController gPitchPid(AppConfig::PID::kPitchPidKp,
                        AppConfig::PID::kPitchPidKi,
                        AppConfig::PID::kPitchPidKd);
PIDController gGyroPid(AppConfig::PID::kGyroPidKp,
                       AppConfig::PID::kGyroPidKi,
                       AppConfig::PID::kGyroPidKd);
PIDController gSpeedPid(AppConfig::PID::kSpeedPidKp,
                        AppConfig::PID::kSpeedPidKi,
                        AppConfig::PID::kSpeedPidKd);

float gForwardCmdFiltered = 0.0f;
unsigned long gLastControlUs = 0;

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float lowPass(float prev, float input, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

bool isPressed(uint8_t buttons, uint8_t bit) {
  return (buttons & (1U << bit)) != 0;
}

float axisToUnit(uint8_t raw, uint8_t deadband) {
  const int centered = static_cast<int>(raw) - 128;
  if (abs(centered) <= deadband) {
    return 0.0f;
  }
  return clampf(static_cast<float>(centered) / 127.0f, -1.0f, 1.0f);
}

void applySafeIdle() {
  MotorControl::setWheelTorques(0.0f, 0.0f);
  MotorControl::setMirroredLegJointAngles(AppConfig::Motor::kLegStartupAngleRad);
  gPitchPid.reset();
  gGyroPid.reset();
  gSpeedPid.reset();
  gForwardCmdFiltered = 0.0f;
}

} // namespace

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }

  imu.begin();
  XboxController::begin();

  gPitchPid.setOutputLimits(-AppConfig::PID::kPidOutputLimit, AppConfig::PID::kPidOutputLimit);
  gGyroPid.setOutputLimits(-AppConfig::PID::kPidOutputLimit, AppConfig::PID::kPidOutputLimit);
  gSpeedPid.setOutputLimits(-AppConfig::PID::kPidOutputLimit, AppConfig::PID::kPidOutputLimit);

  gPitchPid.setOutputRamp(AppConfig::PID::kOutputRampPerSecond);
  gGyroPid.setOutputRamp(AppConfig::PID::kOutputRampPerSecond);
  gSpeedPid.setOutputRamp(AppConfig::PID::kOutputRampPerSecond);

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

  applySafeIdle();
  gLastControlUs = micros();
  Serial.println("Xbox control ready: RSY=forward, RSX=turn, LSY=leg angle, Menu=enable, View=stop");
}

void loop() {
  static unsigned long lastMonitorPrintMs = 0;

  imu.update();
  MotorControl::update();
  XboxController::update();

  if (!XboxController::isConnected()) {
    applySafeIdle();

    const unsigned long nowMs = millis();
    if (nowMs - lastMonitorPrintMs >= AppConfig::Behavior::kMonitorPrintIntervalMs) {
      lastMonitorPrintMs = nowMs;
      Serial.print("xbox:disconnected, dt-ms:");
      Serial.print(XboxController::getTimeSinceLastPacket());
      Serial.print(", rx-errors:");
      Serial.println(XboxController::getErrorCount());
    }
    delay(5);
    return;
  }

  const XboxControllerData& pad = XboxController::getData();
  const bool menuPressed = isPressed(pad.buttons, kButtonMenu);
  const bool viewPressed = isPressed(pad.buttons, kButtonView);
  const bool rbPressed = isPressed(pad.buttons, kButtonRB);

  const unsigned long nowUs = micros();
  float dt = static_cast<float>(nowUs - gLastControlUs) * 1e-6f;
  if (dt <= 0.0f || dt > 0.2f) {
    dt = 0.005f;
  }
  gLastControlUs = nowUs;

  // Edge-triggered enable/disable, similar to the open-source sit/stand behavior.
  if (menuPressed && !gPrevMenuPressed) {
    gDriveEnabled = true;
  }
  if (viewPressed && !gPrevViewPressed) {
    gDriveEnabled = false;
  }
  gPrevMenuPressed = menuPressed;
  gPrevViewPressed = viewPressed;

  float leftTorqueCmd = 0.0f;
  float rightTorqueCmd = 0.0f;
  float legAngleCmd = AppConfig::Motor::kLegStartupAngleRad;

  if (gDriveEnabled) {
    // Open-source style mapping: right stick drives robot; deadband handled in axisToUnit.
    float joyX = axisToUnit(pad.rightStickX, AppConfig::XboxController::kStickDeadband);
    float joyY = axisToUnit(pad.rightStickY, AppConfig::XboxController::kStickDeadband);
    const float legY = axisToUnit(pad.leftStickY, AppConfig::XboxController::kStickDeadband);

    // Match open-source sign: pushing stick up should move robot forward.
    joyY = -joyY;

    const float turbo = rbPressed ? AppConfig::XboxController::kTurboScale : 1.0f;
    const float forwardCmd = joyY * AppConfig::XboxController::kForwardScale * turbo;
    gForwardCmdFiltered = lowPass(gForwardCmdFiltered, forwardCmd, AppConfig::PID::kForwardCommandFilterAlpha);

    // PID stack inspired by the open-source PID project:
    // pitch-angle term + pitch-rate term + speed term.
    const float desiredPitchDeg = AppConfig::PID::kPitchSetpointDeg +
                    gForwardCmdFiltered * AppConfig::PID::kPitchOffsetFromCommandDeg;
    const float pitchTerm = gPitchPid.compute(desiredPitchDeg, imu.data().pitchDeg, dt);
    const float gyroTerm = gGyroPid.compute(0.0f, imu.data().yawRateDegPerSec, dt);

    const float wheelSpeed = 0.5f *
      (MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorLeftNodeId) +
       MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorRightNodeId));
    const float desiredSpeed = gForwardCmdFiltered * AppConfig::PID::kWheelSpeedTargetScale;
    const float speedTerm = gSpeedPid.compute(desiredSpeed, wheelSpeed, dt);

    const float balanceTorque = clampf(pitchTerm + gyroTerm + speedTerm,
                       -AppConfig::Motor::kWheelTorqueLimit,
                       AppConfig::Motor::kWheelTorqueLimit);

    const float turnTorque = joyX * AppConfig::Motor::kWheelTorqueLimit * AppConfig::XboxController::kTurnScale;

    leftTorqueCmd = clampf(balanceTorque - turnTorque,
                           -AppConfig::Motor::kWheelTorqueLimit,
                           AppConfig::Motor::kWheelTorqueLimit);
    rightTorqueCmd = clampf(balanceTorque + turnTorque,
                            -AppConfig::Motor::kWheelTorqueLimit,
                            AppConfig::Motor::kWheelTorqueLimit);

    // Use left stick Y for mirrored leg posture (height surrogate on this mechanism).
    legAngleCmd = AppConfig::Motor::kLegStartupAngleRad +
                  (legY * AppConfig::XboxController::kMaxLegAngleOffsetRad);
    legAngleCmd = clampf(legAngleCmd,
                         -AppConfig::XboxController::kMaxLegAngleOffsetRad,
                         AppConfig::XboxController::kMaxLegAngleOffsetRad);
  }

  MotorControl::setMirroredLegJointAngles(legAngleCmd);
  MotorControl::setWheelTorques(leftTorqueCmd, rightTorqueCmd);

  if (MotorControl::hasNewFeedback()) {
    // Keep the feedback path hot for latest telemetry.
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastMonitorPrintMs >= AppConfig::Behavior::kMonitorPrintIntervalMs) {
    lastMonitorPrintMs = nowMs;
    Serial.print("drive:");
    Serial.print(gDriveEnabled ? 1 : 0);
    Serial.print(", pitch:");
    Serial.print(imu.data().pitchDeg, 2);
    Serial.print(", ltrq:");
    Serial.print(leftTorqueCmd, 3);
    Serial.print(", rtrq:");
    Serial.print(rightTorqueCmd, 3);
    Serial.print(", leg:");
    Serial.print(legAngleCmd, 3);
    Serial.print(", dt:");
    Serial.print(dt, 4);
    Serial.print(", rsx:");
    Serial.print(pad.rightStickX);
    Serial.print(", rsy:");
    Serial.print(pad.rightStickY);
    Serial.print(", lsy:");
    Serial.print(pad.leftStickY);
    Serial.print(", btn:");
    Serial.print(pad.buttons, HEX);
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
