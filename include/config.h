#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace AppConfig {
namespace PID {
constexpr float kOutputRampPerSecond = 80.0f;
constexpr float kPitchSetpointDeg = 0.0f;

// Match PID folder gains: angle + speed + turn loops.
// Balance output is mapped directly to wheel velocity (turns/sec).
constexpr float kPitchPidKp = 0.35f;
constexpr float kPitchPidKi = 0.0f;
constexpr float kPitchPidKd = 0.0f;

// Pitch-rate damping (deg/s -> turns/sec).
constexpr float kPitchRatePidKp = 0.02f;
constexpr float kPitchRatePidKi = 0.0f;
constexpr float kPitchRatePidKd = 0.0f;

// Turn-rate controller (turns/sec target -> turns/sec output).
constexpr float kGyroPidKp = 1.0f;
constexpr float kGyroPidKi = 0.0f;
constexpr float kGyroPidKd = 0.0f;

// Speed loop outputs desired pitch offset (deg).
constexpr float kSpeedPidKp = 0.8f;
constexpr float kSpeedPidKi = 0.0f;
constexpr float kSpeedPidKd = 0.0f;

constexpr float kAnglePidOutputMin = -6.0f;
constexpr float kAnglePidOutputMax = 6.0f;
constexpr float kSpeedPidOutputMin = -8.0f;
constexpr float kSpeedPidOutputMax = 8.0f;
constexpr float kTurnPidOutputMin = -4.0f;
constexpr float kTurnPidOutputMax = 4.0f;
constexpr float kMaxLeanDeg = 10.0f;
constexpr float kStickTargetMax = 127.0f;
// Small deadbands to prevent jitter-induced wheel drift.
constexpr float kPitchErrorDeadbandDeg = 0.75f;
constexpr float kWheelSpeedDeadband = 0.05f; // turns/second
// LQR-style stop detection thresholds.
constexpr float kWheelStopSpeedThreshold = 0.5f;
constexpr float kWheelFastSpeedThreshold = 15.0f;

constexpr float kComputeDtResetSec = 0.001f;
constexpr float kComputeDtMaxSec = 0.5f;
constexpr float kDefaultOutputMin = -5.0f;
constexpr float kDefaultOutputMax = 5.0f;
} // namespace PID

namespace Behavior {
// Serial debug print interval.
constexpr unsigned long kMonitorPrintIntervalMs = 100;
constexpr unsigned long kMainLoopDelayMs = 1.5;
// If true, setup waits for Xbox packets before enabling motors.
constexpr bool kRequireXboxSignalOnStartup = true;
// If true, setup requires both joint motors to be near zero pose before startup continues.
constexpr bool kRequireJointZeroPoseOnStartup = false;
} // namespace Behavior

namespace Motor {
constexpr unsigned long kCanBitrateSecondary = 250000UL;

// Per-motor enable switches for staged bring-up.
constexpr bool kEnableMotor1 = true;
constexpr bool kEnableMotor2 = true;
constexpr bool kEnableMotor3 = true;
constexpr bool kEnableMotor4 = true;

// Motor numbering from user request.
constexpr unsigned char kJointMotorLeftNodeId  = 0x01;  // motor 1
constexpr unsigned char kJointMotorRightNodeId = 0x02;  // motor 2
constexpr unsigned char kWheelMotorLeftNodeId  = 0x03;  // motor 3
constexpr unsigned char kWheelMotorRightNodeId = 0x04;  // motor 4

constexpr float kJointGearRatio = 8.0f;
// Joint hold gains during balancing.
constexpr float kJointKp = 60.0f;
constexpr float kJointKd = 4.5f;

// Joint angle tuning is authored in degrees.
// Motor 1 physical reference:
// - Zero pose is the raw driver reading of 0.
// - Motors 1 and 2 share this same zero pose reference.
// - Default pose is a common offset from zero pose.
// - Runtime control code consumes radians and uses zero pose as the logical reference.
constexpr float kRadToDeg = 180.0f / PI;

// Common default pose offset from zero pose.
// Positive = anti-clockwise for motor 1 (motor 2 is mirrored negative).
constexpr float kDefaultFromZero = -10.0f * PI / 180.0f;
constexpr float kJoint1Trim = -1.0f * PI / 180.0f;
constexpr float kJoint2Trim = 0.0f;
constexpr float kStartupZeroPoseToleranceTurns = 0.05f;

// Default pose angle relative to zero pose.
// Changing this does not require retuning the other joint parameters.
constexpr float kDefaultJointAngle = kDefaultFromZero;

// Joint angle limits relative to the zero pose.
// These are fixed physical limits and do not depend on the default pose.
constexpr float kMinJointAngle = -30.0f * PI / 180.0f; // Compressed/folded from default pose
constexpr float kMaxJointAngle = 70.0f * PI / 180.0f; // Extended from default pose

// Sneak/jump targets relative to the zero pose.
// These are fixed physical targets and do not depend on the default pose.
constexpr float kSneakJointAngle = -1.0f * PI / 180.0f;
constexpr float kJumpJointAngle = -60.0f * PI / 180.0f;
// Joint motor velocity feedforward during jump motion (joint output RPM).
constexpr float kJumpJointVelocityRpm = 180.0f;
constexpr float kJumpJointVelocityTurnsPerSec = (kJumpJointVelocityRpm / 60.0f) * kJointGearRatio;
// Stop jump velocity feedforward once close to target.
constexpr float kJumpVelocityStopErrorDeg = 10.0f;
// Duration to apply jump velocity feedforward at the start of jump extension.
constexpr unsigned long kJumpVelocityKickMs = 120UL;

// Wheel balance output settings.
constexpr float kWheelTorqueLimit = 5.0f;
// Back-to-back wheel installation: motor 3 needs inverted command sign.
constexpr float kWheelLeftSign  =  1.0f;
constexpr float kWheelRightSign = -1.0f;
} // namespace Motor

namespace IMU {
constexpr unsigned long kI2cClockHz = 400000UL;
constexpr float kAccCoef = 0.1f;
constexpr float kGyroCoef = 0.9f;
constexpr float kGyroYLowPassAlpha = 0.05f;
constexpr float kGyroZDeadband = 1.0f;
constexpr float kRemoteBalanceOffsetDeg = 0.0f;

constexpr long kMapInputMin = -17000;
constexpr long kMapInputMax = 17000;
constexpr long kMapOutputMin = 0;
constexpr long kMapOutputMax = 255;
} // namespace IMU

namespace XboxController {
// UART2 communication (pins 16=RX2, 17=TX2)
constexpr unsigned long kUartBaudrate = 115200UL;
constexpr unsigned long kConnectionTimeoutMs = 200UL;

// Controller deadbands (stick values 0-255, centered at 128)
constexpr unsigned char kStickDeadband = 15;  // ~6% deadband
constexpr unsigned char kTriggerDeadband = 5;
constexpr int kStickCenter = 128;
constexpr float kStickNormalizeDen = 127.0f;

// Joystick to motor command conversion
// Left stick Y → forward/backward motion
// Left stick X → turning
constexpr float kMaxMotorVelocity = 4.0f;      // turns/second
constexpr float kMaxTurningVelocity = 4.0f;    // turns/second for turning
constexpr float kForwardScale = 1.0f;
constexpr float kTurnScale = 0.8f;
constexpr float kTurboScale = 1.4f;

// Bit mapping in packet byte 7 from ESP32 sender.
constexpr uint8_t kButtonABit = 0;
constexpr uint8_t kButtonRbBit = 5;
constexpr uint8_t kButtonMenuBit = 6;
constexpr uint8_t kButtonViewBit = 7;

// Control timing and startup behavior.
constexpr float kControlDtFallbackSec = 0.005f;
constexpr float kControlDtMaxSec = 0.2f;
constexpr bool kDriveEnabledOnBoot = true;
} // namespace XboxController
} // namespace AppConfig

#endif // CONFIG_H
