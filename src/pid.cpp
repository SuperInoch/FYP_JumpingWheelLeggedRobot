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

constexpr float kTwoPi = 2.0f * PI;

PIDController pitchPid(AppConfig::PID::kPitchPidKp,
                       AppConfig::PID::kPitchPidKi,
                       AppConfig::PID::kPitchPidKd);
PIDController gyroPid(AppConfig::PID::kGyroPidKp,
                      AppConfig::PID::kGyroPidKi,
                      AppConfig::PID::kGyroPidKd);
PIDController speedPid(AppConfig::PID::kSpeedPidKp,
                       AppConfig::PID::kSpeedPidKi,
                       AppConfig::PID::kSpeedPidKd);

// Jump state machine enumerator.
enum JumpState {
  JUMP_IDLE,       // Legs at standard height, waiting for A button.
  JUMP_SNEAKING,   // Legs compressing; wait for A button release.
  JUMP_JUMPING,    // Legs explosively extending; detect completion.
  JUMP_AIRBORNE    // Legs retracting; wait for landing detection.
};

// Jump sequence state machine variables
JumpState jumpState = JUMP_IDLE;
unsigned long jumpStateStartTimeMs = 0;  // Timestamp when entering current state
bool prevAPressed = false;               // Track A button edge for state transitions
bool rawAPressedPrev = false;            // Last raw A reading for debounce.
bool debouncedAPressed = false;          // Debounced A state used by jump state machine.
unsigned long aDebounceStartTimeMs = 0;  // Raw-edge timestamp for debounce window.

// Hold jump extension for 1 second after A release, then recover to default.
constexpr unsigned long kJumpHoldMs = 1000;
constexpr unsigned long kRecoverToIdleMs = 150;
constexpr unsigned long kAButtonDebounceMs = 35;

ControlDebugState gDebug = {};

float forwardCmdFiltered = 0.0f;
unsigned long lastControlUs = 0;

bool prevMenuPressed = false;
bool prevViewPressed = false;

struct WheelSpeedTargets {
  float leftVelocity;
  float rightVelocity;
};

// Returns true if the requested button bit is currently set.
bool isPressed(uint8_t buttons, uint8_t bit) {
  return (buttons & (1U << bit)) != 0;
}

// First-order low-pass filter for command smoothing.
float lowPass(float prev, float input, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

// Debounces A-button transitions to avoid false jump triggers from packet jitter.
bool debounceAButton(bool aPressedRaw, unsigned long nowMs) {
  if (aPressedRaw != rawAPressedPrev) {
    rawAPressedPrev = aPressedRaw;
    aDebounceStartTimeMs = nowMs;
  }

  if (debouncedAPressed != rawAPressedPrev &&
      (nowMs - aDebounceStartTimeMs) >= kAButtonDebounceMs) {
    debouncedAPressed = rawAPressedPrev;
  }

  return debouncedAPressed;
}

// Converts raw stick byte to normalized [-1, 1] with deadband around center.
float axisToUnit(uint8_t raw, uint8_t deadband) {
  const int centered = static_cast<int>(raw) - AppConfig::XboxController::kStickCenter;
  if (abs(centered) <= deadband) {
    return 0.0f;
  }
  return clampValue(static_cast<float>(centered) / AppConfig::XboxController::kStickNormalizeDen, -1.0f, 1.0f);
}

// Maps left joystick (x,y) to differential wheel velocity targets in turns/second.
WheelSpeedTargets mapLeftStickToWheelSpeeds(float joyX, float joyY, float turboScale) {
  const float forwardVelocity = joyY * AppConfig::XboxController::kForwardScale *
                                AppConfig::XboxController::kMaxMotorVelocity * turboScale;
  const float turnVelocity = joyX * AppConfig::XboxController::kTurnScale *
                             AppConfig::XboxController::kMaxTurningVelocity;

  float leftVelocity = forwardVelocity - turnVelocity;
  float rightVelocity = forwardVelocity + turnVelocity;

  const float leftAbs = fabsf(leftVelocity);
  const float rightAbs = fabsf(rightVelocity);
  const float peakAbs = (leftAbs > rightAbs) ? leftAbs : rightAbs;
  if (peakAbs > AppConfig::XboxController::kMaxMotorVelocity && peakAbs > 0.0f) {
    const float scale = AppConfig::XboxController::kMaxMotorVelocity / peakAbs;
    leftVelocity *= scale;
    rightVelocity *= scale;
  }

  return {leftVelocity, rightVelocity};
}

// Clears all PID internal memory and command filters.
void resetPidStates() {
  pitchPid.reset();
  gyroPid.reset();
  speedPid.reset();
  forwardCmdFiltered = 0.0f;
}

// Jump state machine: orchestrates sneak -> jump -> airborne -> landing sequence.
// Returns the target joint angle based on current jump state and transitions.
float updateJumpStateMachine(bool aPressed, float currentMotorPos, const ImuData& imuData, unsigned long nowMs) {
  (void)currentMotorPos;
  (void)imuData;

  const bool aJustPressed = aPressed && !prevAPressed;
  const bool aJustReleased = !aPressed && prevAPressed;
  prevAPressed = aPressed;

  // State transition logic
  switch (jumpState) {
    case JUMP_IDLE:
      // Idle: track standard position; A press triggers sneak.
      if (aJustPressed) {
        jumpState = JUMP_SNEAKING;
        jumpStateStartTimeMs = nowMs;
      }
      break;

    case JUMP_SNEAKING:
      // Sneaking: compress legs; A release triggers jump.
      if (aJustReleased) {
        jumpState = JUMP_JUMPING;
        jumpStateStartTimeMs = nowMs;
      }
      break;

    case JUMP_JUMPING:
      // Jumping: hold jump target for a fixed window after release.
      if ((nowMs - jumpStateStartTimeMs) >= kJumpHoldMs) {
        jumpState = JUMP_AIRBORNE;
        jumpStateStartTimeMs = nowMs;
      }
      break;

    case JUMP_AIRBORNE:
      // Recovery: settle at standard/default pose then return to idle.
      if ((nowMs - jumpStateStartTimeMs) >= kRecoverToIdleMs) {
        jumpState = JUMP_IDLE;
        jumpStateStartTimeMs = nowMs;
      }
      break;
  }

  // Output target joint angle based on current state
  float targetAngle = AppConfig::Motor::kStandardJointAngle;
  
  switch (jumpState) {
    case JUMP_IDLE:
      targetAngle = AppConfig::Motor::kStandardJointAngle;
      break;
    case JUMP_SNEAKING:
      targetAngle = AppConfig::Motor::kStandardJointAngle + AppConfig::Motor::kSneakAngleOffset;
      break;
    case JUMP_JUMPING:
      targetAngle = AppConfig::Motor::kStandardJointAngle + AppConfig::Motor::kJumpAngleOffset;
      break;
    case JUMP_AIRBORNE:
      targetAngle = AppConfig::Motor::kStandardJointAngle;
      break;
  }

  return targetAngle;
}

// Commands a safe neutral pose and resets control outputs.
void applySafeIdle() {
  MotorControl::setWheelVelocities(0.0f, 0.0f);
  MotorControl::setMirroredLegJointAngles(AppConfig::Motor::kStandardJointAngle);

  gDebug.leftTorqueCmd = 0.0f;
  gDebug.rightTorqueCmd = 0.0f;
  gDebug.legAngleCmd = AppConfig::Motor::kStandardJointAngle;
  gDebug.desiredPitchDeg = AppConfig::PID::kPitchSetpointDeg;
  gDebug.pitchTerm = 0.0f;
  gDebug.gyroTerm = 0.0f;
  gDebug.speedTerm = 0.0f;
  gDebug.balanceTorque = 0.0f;
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
  gDebug.legAngleCmd = AppConfig::Motor::kStandardJointAngle;
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
  rawAPressedPrev = false;
  debouncedAPressed = false;
  aDebounceStartTimeMs = 0;
  prevAPressed = false;
  jumpState = JUMP_IDLE;
  jumpStateStartTimeMs = 0;

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

  const bool aPressedRaw = isPressed(xboxData.buttons, AppConfig::XboxController::kButtonABit);
  const unsigned long nowMs = millis();
  const bool aPressed = debounceAButton(aPressedRaw, nowMs);
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
    const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
    const float targetJointAngle = updateJumpStateMachine(aPressed, motor1Pos, imuData, nowMs);
    gDebug.legAngleCmd = clampValue(targetJointAngle,
                                    AppConfig::Motor::kMinJointAngle,
                                    AppConfig::Motor::kMaxJointAngle);

    // Keep wheels idle when drive is disabled.
    gDebug.leftTorqueCmd = 0.0f;
    gDebug.rightTorqueCmd = 0.0f;
    MotorControl::setMirroredLegJointAngles(gDebug.legAngleCmd);
    MotorControl::setWheelVelocities(0.0f, 0.0f);
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
  const WheelSpeedTargets stickTargets = mapLeftStickToWheelSpeeds(joyX, joyY, turbo);
  const float stickForwardVelocity = 0.5f * (stickTargets.leftVelocity + stickTargets.rightVelocity);
  const float stickTurnVelocity = 0.5f * (stickTargets.rightVelocity - stickTargets.leftVelocity);
  const float forwardCmdUnit = clampValue(stickForwardVelocity / AppConfig::XboxController::kMaxMotorVelocity,
                                          -1.0f,
                                          1.0f);
  forwardCmdFiltered = lowPass(forwardCmdFiltered, forwardCmdUnit, AppConfig::PID::kForwardCommandFilterAlpha);

  gDebug.desiredPitchDeg = AppConfig::PID::kPitchSetpointDeg +
                           forwardCmdFiltered * AppConfig::PID::kPitchOffsetFromCommandDeg;
  gDebug.pitchTerm = pitchPid.compute(gDebug.desiredPitchDeg, imuData.pitchDeg, dt);
  gDebug.gyroTerm = gyroPid.compute(0.0f, imuData.yawRateDegPerSec, dt);

  const float wheelSpeed = 0.5f *
      (MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorLeftNodeId) +
       MotorControl::getMotorVelocity(AppConfig::Motor::kWheelMotorRightNodeId));
  const float desiredSpeed = stickForwardVelocity;
  gDebug.speedTerm = speedPid.compute(desiredSpeed, wheelSpeed, dt);

  gDebug.balanceTorque = clampValue(gDebug.pitchTerm + gDebug.gyroTerm + gDebug.speedTerm,
                                    -AppConfig::XboxController::kMaxMotorVelocity,
                                    AppConfig::XboxController::kMaxMotorVelocity);

  gDebug.leftTorqueCmd = clampValue(gDebug.balanceTorque - stickTurnVelocity,
                                    -AppConfig::XboxController::kMaxMotorVelocity,
                                    AppConfig::XboxController::kMaxMotorVelocity);
  gDebug.rightTorqueCmd = clampValue(gDebug.balanceTorque + stickTurnVelocity,
                                     -AppConfig::XboxController::kMaxMotorVelocity,
                                     AppConfig::XboxController::kMaxMotorVelocity);

  // === Secondary joint angle control: A-button gated jump sequence ===
  // 1. Read current joint motor positions
  const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
  // 2. Update jump state machine to get target angle (sneak -> jump -> airborne -> idle)
  const float targetJointAngleFromStateMachine = updateJumpStateMachine(aPressed, motor1Pos, imuData, nowMs);

  // 3. Command the exact jump-state target and let motor position mode track it.
  gDebug.legAngleCmd = clampValue(targetJointAngleFromStateMachine,
                                  AppConfig::Motor::kMinJointAngle,
                                  AppConfig::Motor::kMaxJointAngle);
  
  // Clamp to safety limits
  gDebug.legAngleCmd = clampValue(gDebug.legAngleCmd,
                                  AppConfig::Motor::kMinJointAngle,
                                  AppConfig::Motor::kMaxJointAngle);

  MotorControl::setMirroredLegJointAngles(gDebug.legAngleCmd);
  MotorControl::setWheelVelocities(gDebug.leftTorqueCmd, gDebug.rightTorqueCmd);
}

// Exposes latest debug/telemetry snapshot to other modules.
const ControlDebugState& debugState() {
  return gDebug;
}

} // namespace RobotControl
