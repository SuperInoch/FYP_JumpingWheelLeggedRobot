#ifndef PID_H
#define PID_H

#include <cstdint>

#include "imu.h"
#include "xbox_controller.h"

class PIDController {
public:
  // Creates a PID controller.
  // Input: kp/ki/kd gains.
  PIDController(float kp, float ki, float kd);

  // Input: new kp/ki/kd values.
  void setGains(float kp, float ki, float kd);
  // Input: min/max output limits after PID compute.
  void setOutputLimits(float minOutput, float maxOutput);
  // Input: max output slope (units/second), 0 to disable ramp limiting.
  void setOutputRamp(float rampPerSecond);
  // Clears integrator and previous-step history.
  void reset();

  // Input: desired setpoint, current measurement, and dt seconds.
  // Output: control value after PID + clamping/ramp limiting.
  float compute(float setpoint, float measurement, float dtSeconds);

private:
  float kp_;
  float ki_;
  float kd_;

  float integral_;
  float previousError_;
  float previousOutput_;
  float minOutput_;
  float maxOutput_;
  float outputRampPerSecond_;
};

struct ControlDebugState {
  bool driveEnabled;
  bool xboxConnected;
  float dtSeconds;

  float leftTorqueCmd;
  float rightTorqueCmd;
  float legAngleCmd;

  float desiredPitchDeg;
  float pitchTerm;
  float gyroTerm;
  float speedTerm;
  float balanceTorque;

  uint8_t rightStickX;
  uint8_t rightStickY;
  uint8_t leftStickY;
  uint8_t buttons;

  uint32_t xboxAgeMs;
  uint16_t xboxErrorCount;
};

namespace RobotControl {

// Initializes controller state and safe startup outputs.
void begin();
// Runs one full control step.
// Input: IMU state + Xbox link/data + packet health info.
// Output: updates motor command targets and debug telemetry.
void process(const ImuData& imuData,
             bool xboxConnected,
             const XboxControllerData& xboxData,
             uint32_t xboxAgeMs,
             uint16_t xboxErrorCount);
// Output: latest debug snapshot for serial monitor/telemetry.
const ControlDebugState& debugState();

} // namespace RobotControl

#endif // PID_H
