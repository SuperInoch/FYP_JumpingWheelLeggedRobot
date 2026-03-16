#ifndef CONFIG_H
#define CONFIG_H

namespace AppConfig {
namespace PID {
constexpr float kPitchKp = -0.18f;
constexpr float kPitchKi = 0.0f;
constexpr float kPitchKd = 0.015f;
constexpr float kOutputMin = -6.0f;
constexpr float kOutputMax = 6.0f;
constexpr float kOutputRampPerSecond = 40.0f;
constexpr float kPitchSetpointDeg = 0.0f;
} // namespace PID

namespace Behavior {
// Serial debug print interval.
constexpr unsigned long kMonitorPrintIntervalMs = 100;
} // namespace Behavior

namespace Motor {
constexpr unsigned long kCanBitratePrimary = 1000000UL;
constexpr unsigned long kCanBitrateSecondary = 500000UL;
constexpr bool kSingleMotorTestMode = true;

// Motor numbering from user request.
constexpr uint8_t kJointMotorLeftNodeId = 0x01;   // motor 1
constexpr uint8_t kJointMotorRightNodeId = 0x02;  // motor 2
constexpr uint8_t kWheelMotorLeftNodeId = 0x03;   // motor 3
constexpr uint8_t kWheelMotorRightNodeId = 0x04;  // motor 4

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
} // namespace AppConfig

#endif // CONFIG_H
