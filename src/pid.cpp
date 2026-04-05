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

// Secondary PD loop for joint angle holding (not integrated; holds standard position).
// These are fixed PD gains used to maintain leg height during balanced locomotion.
constexpr float kJointAngleKp = 15.0f;   // Proportional gain for joint angle error
constexpr float kJointAngleKd = 1.0f;    // Derivative gain for joint angle rate

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
float jumpMotorPosAtStateStart = 0.0f;   // Motor position when jump extension starts
bool prevAPressed = false;               // Track A button edge for state transitions

// Landing detection threshold: Z-axis acceleration spike (raw sensor units).
constexpr int16_t kLandingAccelThreshold = 15000;  // Adjust based on landing impact sensitivity

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

// Jump state machine: orchestrates sneak -> jump -> airborne -> landing sequence.
// Returns the target joint angle based on current jump state and transitions.
float updateJumpStateMachine(bool aPressed, float currentMotorPos, const ImuData& imuData, unsigned long nowMs) {
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
        jumpMotorPosAtStateStart = currentMotorPos;
      }
      break;

    case JUMP_JUMPING:
      // Jumping: hold explosive extension until motor displacement detected.
      // Use a 200ms timeout or motor movement threshold.
      {
        const unsigned long jumpDurationMs = nowMs - jumpStateStartTimeMs;
        const float motorDisplacementTurns = abs(currentMotorPos - jumpMotorPosAtStateStart);
        const float motorDisplacementRad = motorDisplacementTurns * kTwoPi / AppConfig::Motor::kJointGearRatio;
        const bool jumpExtensionComplete = (jumpDurationMs > 200) || (motorDisplacementRad > 2.0f);
        
        if (jumpExtensionComplete) {
          jumpState = JUMP_AIRBORNE;
          jumpStateStartTimeMs = nowMs;
        }
      }
      break;

    case JUMP_AIRBORNE:
      // Airborne: retract to standard, detect landing via Z-accel spike.
      // Landing is high Z acceleration indicating impact.
      if (imuData.accelZ > kLandingAccelThreshold) {
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
      targetAngle = AppConfig::Motor::kStandardJointAngle - AppConfig::Motor::kSneakAngleOffset;
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
  MotorControl::setWheelTorques(0.0f, 0.0f);
  MotorControl::setMirroredLegJointAngles(AppConfig::Motor::kStandardJointAngle);

  gDebug.leftTorqueCmd = 0.0f;
  gDebug.rightTorqueCmd = 0.0f;
  gDebug.legAngleCmd = AppConfig::Motor::kStandardJointAngle;
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
    gDebug.legAngleCmd = AppConfig::Motor::kStandardJointAngle + jumpOffset;
    gDebug.legAngleCmd = clampValue(gDebug.legAngleCmd,
                                    AppConfig::Motor::kStandardJointAngle - AppConfig::XboxController::kMaxLegAngleOffsetRad,
                                    AppConfig::Motor::kStandardJointAngle + AppConfig::XboxController::kMaxLegAngleOffsetRad);

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

  // === Secondary joint angle control: Hold standard posture + jump sequence ===
  // 1. Read current joint motor positions and convert to logical angles (accounting for direction inversion)
  const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
  const float motor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
  
  // Convert motor turns to logical joint angles (inverse of command calculation for motors)
  // These track the actual external linkage angle despite motor direction inversion
  const float currentJointAngle = MotorControl::getJointAngleRad(AppConfig::Motor::kJointMotorLeftNodeId);
  
  // 2. Update jump state machine to get target angle (sneak -> jump -> airborne -> idle)
  const unsigned long nowMs = millis();
  const float targetJointAngleFromStateMachine = updateJumpStateMachine(aPressed, motor1Pos, imuData, nowMs);
  
  // 3. Estimate joint angle velocity via finite difference for damping
  const float jointAngleDtCurrent = currentJointAngle;
  const float jointAngleVelocity = (jointAngleDtCurrent - jointAngleDtPrevious) / dt;
  jointAngleDtPrevious = jointAngleDtCurrent;
  
  // 4. Calculate joint angle error and apply PD control to reach state-machine target
  const float jointAngleError = targetJointAngleFromStateMachine - currentJointAngle;
  const float jointPdOutput = (kJointAngleKp * jointAngleError) - (kJointAngleKd * jointAngleVelocity);
  
  // 5. Apply joint angle PD output as a modulation on the joint command
  //    This ensures smooth motion between state machine targets.
  gDebug.legAngleCmd = clampValue(
      targetJointAngleFromStateMachine + jointPdOutput,
      AppConfig::Motor::kMinJointAngle,
      AppConfig::Motor::kMaxJointAngle
  );
  
  // Clamp to safety limits
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
