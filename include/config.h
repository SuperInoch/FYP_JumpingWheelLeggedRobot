#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace AppConfig {
namespace PID {
constexpr float kOutputRampPerSecond = 40.0f;
constexpr float kPitchSetpointDeg = 0.0f;

// Match PID folder gains: angle + speed + turn loops.
constexpr float kPitchPidKp = 3.0f;
constexpr float kPitchPidKi = 0.1f;
constexpr float kPitchPidKd = 3.0f;

constexpr float kGyroPidKp = 4.0f;
constexpr float kGyroPidKi = 3.0f;
constexpr float kGyroPidKd = 0.0f;

constexpr float kSpeedPidKp = 2.0f;
constexpr float kSpeedPidKi = 0.05f;
constexpr float kSpeedPidKd = 0.0f;

constexpr float kAnglePidOutputMin = -100.0f;
constexpr float kAnglePidOutputMax = 100.0f;
constexpr float kSpeedPidOutputMin = -20.0f;
constexpr float kSpeedPidOutputMax = 20.0f;
constexpr float kTurnPidOutputMin = -50.0f;
constexpr float kTurnPidOutputMax = 50.0f;
constexpr float kStickTargetMax = 127.0f;

constexpr float kComputeDtResetSec = 0.001f;
constexpr float kComputeDtMaxSec = 0.5f;
constexpr float kDefaultOutputMin = -5.0f;
constexpr float kDefaultOutputMax = 5.0f;
} // namespace PID

namespace Behavior {
// Serial debug print interval.
constexpr unsigned long kMonitorPrintIntervalMs = 100;
constexpr unsigned long kMainLoopDelayMs = 5;
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
constexpr unsigned char kJointMotorLeftNodeId = 0x01;   // motor 1
constexpr unsigned char kJointMotorRightNodeId = 0x02;  // motor 2
constexpr unsigned char kWheelMotorLeftNodeId = 0x03;   // motor 3
constexpr unsigned char kWheelMotorRightNodeId = 0x04;  // motor 4

constexpr float kJointGearRatio = 8.0f;
// Joint hold gains during balancing.
constexpr float kJointKp = 25.0f;
constexpr float kJointKd = 1.0f;

// Joint angle tuning is authored in degrees.
// Motor 1 physical reference:
// - Zero pose is the raw driver reading of 0.
// - Motors 1 and 2 share this same zero pose reference.
// - Default pose is a common offset from zero pose.
// - Standard pose is controlled relative to default pose.
// Runtime control code consumes radians.
constexpr float kRadToDeg = 180.0f / PI;

// Common default pose offset from zero pose.
// Positive = anti-clockwise for motor 1 (motor 2 is mirrored negative).
constexpr float kDefaultFromZero = -24.06967262304f * PI / 180.0f;
constexpr float kJoint1Trim = -1.0f * PI / 180.0f;
constexpr float kJoint2Trim = 0.0f;
constexpr float kStartupZeroPoseToleranceTurns = 0.05f;

// Default pose in controller coordinates (relative to default pose baseline).
constexpr float kDefaultJointAngle = 0.0f;

// Joint angle limits relative to the default pose.
// TODO: Measure and calibrate these based on your mechanical range of motion.
constexpr float kMinJointAngle = -22.401f * PI / 180.0f;  // Compressed/folded from default pose
constexpr float kMaxJointAngle = 35.0f * PI / 180.0f;   // Extended from default pose

// Pre-jump compression and jump extension offsets relative to the standard pose.
// Negative sneak offset shortens the legs; positive jump offset extends them.
constexpr float kSneakAngleOffset = -25.0f * PI / 180.0f;   // A pressed target: +25 deg
constexpr float kJumpAngleOffset = 20.0f * PI / 180.0f;   // A released target: -20 deg

// Wheel balance output settings.
constexpr float kWheelTorqueLimit = 5.0f;
// Back-to-back wheel installation: motor 3 needs inverted command sign.
constexpr float kWheelLeftSign = -1.0f;
constexpr float kWheelRightSign = 1.0f;
} // namespace Motor

namespace IMU {
constexpr unsigned long kI2cClockHz = 400000UL;
constexpr float kAccCoef = 0.02f;
constexpr float kGyroCoef = 0.98f;
constexpr float kGyroYLowPassAlpha = 0.005f;
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
constexpr float kMaxTurningVelocity = 2.0f;    // turns/second for turning
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
