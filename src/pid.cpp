#include "pid.h"

#include <Arduino.h>

#include "config.h"
#include "motor.h"

// Generic clamp utility used by PID and control mixing stages.
static float clampValue(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

namespace {

PIDController pitchPid(AppConfig::PID::kPitchPidKp,
                       AppConfig::PID::kPitchPidKi,
                       AppConfig::PID::kPitchPidKd);
PIDController gyroPid(AppConfig::PID::kGyroPidKp,
                      AppConfig::PID::kGyroPidKi,
                      AppConfig::PID::kGyroPidKd);
PIDController speedPid(AppConfig::PID::kSpeedPidKp,
                       AppConfig::PID::kSpeedPidKi,
                       AppConfig::PID::kSpeedPidKd);

// Secondary PD loop for joint angle holding (not integrated; holds standard position).
// These are fixed PD gains used to maintain leg height during balanced locomotion.
constexpr float kJointAngleKp = 15.0f;   // Proportional gain for joint angle error
constexpr float kJointAngleKd = 1.0f;    // Derivative gain for joint angle rate

ControlDebugState gDebug = {};

float forwardCmdFiltered = 0.0f;
float jointAngleDtPrevious = 0.0f;  // Previous joint angle for velocity estimation
unsigned long lastControlUs = 0;

bool prevMenuPressed = false;
bool prevViewPressed = false;

// Returns true if the requested button bit is currently set.
bool isPressed(uint8_t buttons, uint8_t bit) {
  return (buttons & (1U << bit)) != 0;
}

// First-order low-pass filter for command smoothing.
float lowPass(float prev, float input, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

// Converts raw stick byte to normalized [-1, 1] with deadband around center.
float axisToUnit(uint8_t raw, uint8_t deadband) {
  const int centered = static_cast<int>(raw) - AppConfig::XboxController::kStickCenter;
  if (abs(centered) <= deadband) {
    return 0.0f;
  }
  return clampValue(static_cast<float>(centered) / AppConfig::XboxController::kStickNormalizeDen, -1.0f, 1.0f);
}

// Clears all PID internal memory and command filters.
void resetPidStates() {
  pitchPid.reset();
  gyroPid.reset();
  speedPid.reset();
  forwardCmdFiltered = 0.0f;
}

// Commands a safe neutral pose and resets control outputs.
void applySafeIdle() {
  MotorControl::setWheelTorques(0.0f, 0.0f);
  MotorControl::setMirroredLegJointAngles(AppConfig::Motor::kLegStartupAngleRad);

  gDebug.leftTorqueCmd = 0.0f;
  gDebug.rightTorqueCmd = 0.0f;
  gDebug.legAngleCmd = AppConfig::Motor::kLegStartupAngleRad;
  gDebug.desiredPitchDeg = AppConfig::PID::kPitchSetpointDeg;
  gDebug.pitchTerm = 0.0f;
  gDebug.gyroTerm = 0.0f;
  gDebug.speedTerm = 0.0f;
  gDebug.balanceTorque = 0.0f;

  jointAngleDtPrevious = 0.0f;  // Reset joint angle velocity estimator
  resetPidStates();
}

} // namespace

// Constructs a PID object with default limits and zeroed state.
PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), previousError_(0.0f), previousOutput_(0.0f),
      minOutput_(AppConfig::PID::kDefaultOutputMin), maxOutput_(AppConfig::PID::kDefaultOutputMax),
      outputRampPerSecond_(0.0f) {}

    // Updates proportional, integral, and derivative gains.
void PIDController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

// Sets hard output saturation limits for this PID.
void PIDController::setOutputLimits(float minOutput, float maxOutput) {
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

// Sets max output slope (units/sec) for slew limiting.
void PIDController::setOutputRamp(float rampPerSecond) {
  outputRampPerSecond_ = rampPerSecond;
}

// Resets integral accumulation and previous-step history.
void PIDController::reset() {
  integral_ = 0.0f;
  previousError_ = 0.0f;
  previousOutput_ = 0.0f;
}

// Computes one PID update with clamping and optional slew-rate limiting.
float PIDController::compute(float setpoint, float measurement, float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    dtSeconds = AppConfig::PID::kComputeDtResetSec;
  }
  if (dtSeconds > AppConfig::PID::kComputeDtMaxSec) {
    dtSeconds = AppConfig::PID::kComputeDtResetSec;
  }

  const float error = setpoint - measurement;

  // Match lesson implementation: Tustin integral with bounded integrator.
  integral_ += ki_ * dtSeconds * 0.5f * (error + previousError_);
  integral_ = clampValue(integral_,
                         minOutput_ / AppConfig::PID::kIntegratorLimitDivisor,
                         maxOutput_ / AppConfig::PID::kIntegratorLimitDivisor);

  const float derivative = (error - previousError_) / dtSeconds;
  float output = kp_ * error + integral_ + kd_ * derivative;
  output = clampValue(output, minOutput_, maxOutput_);

  // Optional output slew-rate limiting from the lesson PID style.
  if (outputRampPerSecond_ > 0.0f) {
    const float outputRate = (output - previousOutput_) / dtSeconds;
    if (outputRate > outputRampPerSecond_) {
      output = previousOutput_ + outputRampPerSecond_ * dtSeconds;
    } else if (outputRate < -outputRampPerSecond_) {
      output = previousOutput_ - outputRampPerSecond_ * dtSeconds;
    }
  }

  previousError_ = error;
  previousOutput_ = output;
  return output;
}

namespace RobotControl {

// Initializes controller limits, defaults, and safe idle state.
void begin() {
  pitchPid.setOutputLimits(-AppConfig::PID::kPidOutputLimit, AppConfig::PID::kPidOutputLimit);
  gyroPid.setOutputLimits(-AppConfig::PID::kPidOutputLimit, AppConfig::PID::kPidOutputLimit);
  speedPid.setOutputLimits(-AppConfig::PID::kPidOutputLimit, AppConfig::PID::kPidOutputLimit);

  pitchPid.setOutputRamp(AppConfig::PID::kOutputRampPerSecond);
  gyroPid.setOutputRamp(AppConfig::PID::kOutputRampPerSecond);
  speedPid.setOutputRamp(AppConfig::PID::kOutputRampPerSecond);

  gDebug.driveEnabled = AppConfig::XboxController::kDriveEnabledOnBoot;
  gDebug.xboxConnected = false;
  gDebug.dtSeconds = AppConfig::XboxController::kControlDtFallbackSec;
  gDebug.leftTorqueCmd = 0.0f;
  gDebug.rightTorqueCmd = 0.0f;
  gDebug.legAngleCmd = AppConfig::Motor::kLegStartupAngleRad;
  gDebug.desiredPitchDeg = AppConfig::PID::kPitchSetpointDeg;
  gDebug.pitchTerm = 0.0f;
  gDebug.gyroTerm = 0.0f;
  gDebug.speedTerm = 0.0f;
  gDebug.balanceTorque = 0.0f;
  gDebug.rightStickX = 128;
  gDebug.rightStickY = 128;
  gDebug.leftStickY = 128;
  gDebug.buttons = 0;
  gDebug.xboxAgeMs = 0;
  gDebug.xboxErrorCount = 0;

  prevMenuPressed = false;
  prevViewPressed = false;

  applySafeIdle();
  lastControlUs = micros();
}

// Executes one full robot control iteration from sensors/input to motor commands.
void process(const ImuData& imuData,
             bool xboxConnected,
             const XboxControllerData& xboxData,
             uint32_t xboxAgeMs,
             uint16_t xboxErrorCount) {
  gDebug.xboxConnected = xboxConnected;
  gDebug.xboxAgeMs = xboxAgeMs;
  gDebug.xboxErrorCount = xboxErrorCount;

  const unsigned long nowUs = micros();
  float dt = static_cast<float>(nowUs - lastControlUs) * 1e-6f;
  if (dt <= 0.0f || dt > AppConfig::XboxController::kControlDtMaxSec) {
    dt = AppConfig::XboxController::kControlDtFallbackSec;
  }
  lastControlUs = nowUs;
  gDebug.dtSeconds = dt;

  if (!xboxConnected) {
    applySafeIdle();
    return;
  }

  gDebug.rightStickX = xboxData.rightStickX;
  gDebug.rightStickY = xboxData.rightStickY;
  gDebug.leftStickY = xboxData.leftStickY;
  gDebug.buttons = xboxData.buttons;

  const bool aPressed = isPressed(xboxData.buttons, AppConfig::XboxController::kButtonABit);
  const bool menuPressed = isPressed(xboxData.buttons, AppConfig::XboxController::kButtonMenuBit);
  const bool viewPressed = isPressed(xboxData.buttons, AppConfig::XboxController::kButtonViewBit);
  const bool rbPressed = isPressed(xboxData.buttons, AppConfig::XboxController::kButtonRbBit);

  if (menuPressed && !prevMenuPressed) {
    gDebug.driveEnabled = true;
  }
  if (viewPressed && !prevViewPressed) {
    gDebug.driveEnabled = false;
  }
  prevMenuPressed = menuPressed;
  prevViewPressed = viewPressed;

  if (!gDebug.driveEnabled) {
    const float jumpOffset = aPressed ? AppConfig::XboxController::kMaxLegAngleOffsetRad : 0.0f;
    gDebug.legAngleCmd = AppConfig::Motor::kLegStartupAngleRad + jumpOffset;
    gDebug.legAngleCmd = clampValue(gDebug.legAngleCmd,
                                    -AppConfig::XboxController::kMaxLegAngleOffsetRad,
                                    AppConfig::XboxController::kMaxLegAngleOffsetRad);

    // Keep wheels idle when drive is disabled, but allow A-button leg motion for testing.
    gDebug.leftTorqueCmd = 0.0f;
    gDebug.rightTorqueCmd = 0.0f;
    MotorControl::setMirroredLegJointAngles(gDebug.legAngleCmd);
    MotorControl::setWheelTorques(0.0f, 0.0f);
    return;
  }

  // New mapping:
  // - Left stick Y: forward/backward
  // - Left stick X: turning
  // - A button: jump command (leg-angle pulse while held)
  float joyX = axisToUnit(xboxData.leftStickX, AppConfig::XboxController::kStickDeadband);
  float joyY = axisToUnit(xboxData.leftStickY, AppConfig::XboxController::kStickDeadband);

  joyY = -joyY;

  const float turbo = rbPressed ? AppConfig::XboxController::kTurboScale : 1.0f;
  const float forwardCmd = joyY * AppConfig::XboxController::kForwardScale * turbo;
  forwardCmdFiltered = lowPass(forwardCmdFiltered, forwardCmd, AppConfig::PID::kForwardCommandFilterAlpha);

  gDebug.desiredPitchDeg = AppConfig::PID::kPitchSetpointDeg +
                           forwardCmdFiltered * AppConfig::PID::kPitchOffsetFromCommandDeg;
  gDebug.pitchTerm = pitchPid.compute(gDebug.desiredPitchDeg, imuData.pitchDeg, dt);
  gDebug.gyroTerm = gyroPid.compute(0.0f, imuData.yawRateDegPerSec, dt);

  const float wheelSpeed = 0.5f *
      (MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorLeftNodeId) +
       MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorRightNodeId));
  const float desiredSpeed = forwardCmdFiltered * AppConfig::PID::kWheelSpeedTargetScale;
  gDebug.speedTerm = speedPid.compute(desiredSpeed, wheelSpeed, dt);

  gDebug.balanceTorque = clampValue(gDebug.pitchTerm + gDebug.gyroTerm + gDebug.speedTerm,
                                    -AppConfig::Motor::kWheelTorqueLimit,
                                    AppConfig::Motor::kWheelTorqueLimit);

  const float turnTorque = joyX * AppConfig::Motor::kWheelTorqueLimit * AppConfig::XboxController::kTurnScale;
  gDebug.leftTorqueCmd = clampValue(gDebug.balanceTorque - turnTorque,
                                    -AppConfig::Motor::kWheelTorqueLimit,
                                    AppConfig::Motor::kWheelTorqueLimit);
  gDebug.rightTorqueCmd = clampValue(gDebug.balanceTorque + turnTorque,
                                     -AppConfig::Motor::kWheelTorqueLimit,
                                     AppConfig::Motor::kWheelTorqueLimit);

  // === Secondary joint angle control: Hold standard posture during balance ===
  // 1. Read current joint motor positions and convert to logical angles (accounting for direction inversion)
  const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
  const float motor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
  
  // Convert motor turns to logical joint angles (inverse of command calculation for motors)
  // These track the actual external linkage angle despite motor direction inversion
  const float currentJointAngle = AppConfig::Motor::kLegStartupAngleRad +
      ((motor1Pos - AppConfig::Motor::kJoint1Vertical90DegMotorTurns) / 
       (AppConfig::Motor::kJointGearRatio * AppConfig::Motor::kJoint1LegDirectionSign));
  
  // 2. Estimate joint angle velocity via finite difference for damping
  const float jointAngleDtCurrent = currentJointAngle;
  const float jointAngleVelocity = (jointAngleDtCurrent - jointAngleDtPrevious) / dt;
  jointAngleDtPrevious = jointAngleDtCurrent;
  
  // 3. Check if jump button active; if so, apply temporary jump override
  const float jumpOffset = aPressed ? AppConfig::Motor::kJumpAngleOffset : 0.0f;
  const float targetJointAngle = AppConfig::Motor::kStandardJointAngle + jumpOffset;
  
  // 4. Calculate joint angle error and apply PD control
  const float jointAngleError = targetJointAngle - currentJointAngle;
  const float jointPdOutput = (kJointAngleKp * jointAngleError) - (kJointAngleKd * jointAngleVelocity);
  
  // 5. Apply joint angle PD output as a modulation on the joint command
  //    This ensures legs try to return to standard height unless forced by jump command.
  gDebug.legAngleCmd = clampValue(
      targetJointAngle + jointPdOutput,
      AppConfig::Motor::kMinJointAngle,
      AppConfig::Motor::kMaxJointAngle
  );
  
  // Debug telemetry for joint angle holding loop
  gDebug.legAngleCmd = clampValue(gDebug.legAngleCmd,
                                  AppConfig::Motor::kMinJointAngle,
                                  AppConfig::Motor::kMaxJointAngle);

  MotorControl::setMirroredLegJointAngles(gDebug.legAngleCmd);
  MotorControl::setWheelTorques(gDebug.leftTorqueCmd, gDebug.rightTorqueCmd);
}

// Exposes latest debug/telemetry snapshot to other modules.
const ControlDebugState& debugState() {
  return gDebug;
}

} // namespace RobotControl
