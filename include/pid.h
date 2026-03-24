#ifndef PID_H
#define PID_H

#include <cstdint>

#include "imu.h"
#include "xbox_controller.h"

class PIDController {
public:
  PIDController(float kp, float ki, float kd);

  void setGains(float kp, float ki, float kd);
  void setOutputLimits(float minOutput, float maxOutput);
  void setOutputRamp(float rampPerSecond);
  void reset();

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

void begin();
void process(const ImuData& imuData,
             bool xboxConnected,
             const XboxControllerData& xboxData,
             uint32_t xboxAgeMs,
             uint16_t xboxErrorCount);
const ControlDebugState& debugState();

} // namespace RobotControl

#endif // PID_H
