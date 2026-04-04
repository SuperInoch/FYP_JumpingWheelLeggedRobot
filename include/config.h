#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

namespace AppConfig {
namespace PID {
constexpr float kPitchKp = -0.18f;
constexpr float kPitchKi = 0.0f;
constexpr float kPitchKd = 0.015f;
constexpr float kOutputMin = -6.0f;
constexpr float kOutputMax = 6.0f;
constexpr float kOutputRampPerSecond = 40.0f;
constexpr float kPitchSetpointDeg = 0.0f;

// PID-system ported from the "Controller with PID" style (angle + gyro + speed).
constexpr float kPitchPidKp = 1.0f;
constexpr float kPitchPidKi = 0.0f;
constexpr float kPitchPidKd = 0.0f;

constexpr float kGyroPidKp = 0.06f;
constexpr float kGyroPidKi = 0.0f;
constexpr float kGyroPidKd = 0.0f;

constexpr float kSpeedPidKp = 0.7f;
constexpr float kSpeedPidKi = 0.0f;
constexpr float kSpeedPidKd = 0.0f;

constexpr float kPidOutputLimit = 8.0f;
constexpr float kPitchOffsetFromCommandDeg = 8.0f;
constexpr float kWheelSpeedTargetScale = 2.0f;
constexpr float kForwardCommandFilterAlpha = 0.2f;
constexpr float kComputeDtResetSec = 0.001f;
constexpr float kComputeDtMaxSec = 0.5f;
constexpr float kIntegratorLimitDivisor = 3.0f;
constexpr float kDefaultOutputMin = -5.0f;
constexpr float kDefaultOutputMax = 5.0f;
} // namespace PID

namespace Behavior {
// Serial debug print interval.
constexpr unsigned long kMonitorPrintIntervalMs = 100;
constexpr unsigned long kMainLoopDelayMs = 5;
// If true, setup waits for Xbox packets before enabling motors.
constexpr bool kRequireXboxSignalOnStartup = false;
} // namespace Behavior

namespace Motor {
constexpr unsigned long kCanBitratePrimary = 1000000UL;
constexpr unsigned long kCanBitrateSecondary = 500000UL;

// Per-motor enable switches for staged bring-up.
constexpr bool kEnableMotor1 = true;
constexpr bool kEnableMotor2 = true;
constexpr bool kEnableMotor3 = false;
constexpr bool kEnableMotor4 = false;

// Motor numbering from user request.
constexpr unsigned char kJointMotorLeftNodeId = 0x01;   // motor 1
constexpr unsigned char kJointMotorRightNodeId = 0x02;  // motor 2
constexpr unsigned char kWheelMotorLeftNodeId = 0x03;   // motor 3
constexpr unsigned char kWheelMotorRightNodeId = 0x04;  // motor 4

// MIT mode limits aligned with lesson values.
constexpr float kPosMin = -25.12f;
constexpr float kPosMax = 25.12f;
constexpr float kVelMin = -65.0f;
constexpr float kVelMax = 65.0f;
constexpr float kKpMin = 0.0f;
constexpr float kKpMax = 500.0f;
constexpr float kKdMin = 0.0f;
constexpr float kKdMax = 5.0f;
constexpr float kTorMin = -18.0f;
constexpr float kTorMax = 18.0f;

constexpr float kJointGearRatio = 8.0f;
// Measure and tune these two per motor on your hardware.
constexpr float kJoint1Vertical90DegMotorTurns = 1.89f;
constexpr float kJoint2Vertical90DegMotorTurns = 3.69f;

// Startup leg angle (radians): 0 means exactly lesson's reference pose.
constexpr float kLegStartupAngleRad = 0.0f;

// Joint hold gains during balancing.
constexpr float kJointKp = 25.0f;
constexpr float kJointKd = 1.0f;

// Wheel balance output settings.
constexpr float kWheelTorqueLimit = 5.0f;
constexpr float kWheelLeftSign = 1.0f;
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
// Right stick X → turning
constexpr float kMaxMotorVelocity = 4.0f;      // turns/second
constexpr float kMaxTurningVelocity = 2.0f;    // turns/second for turning
constexpr float kForwardScale = 1.0f;
constexpr float kTurnScale = 0.8f;
constexpr float kTurboScale = 1.4f;
constexpr float kMaxLegAngleOffsetRad = 0.35f;

// Bit mapping in packet byte 7 from ESP32 sender.
constexpr uint8_t kButtonABit = 0;
constexpr uint8_t kButtonBBit = 1;
constexpr uint8_t kButtonXBit = 2;
constexpr uint8_t kButtonYBit = 3;
constexpr uint8_t kButtonLbBit = 4;
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
