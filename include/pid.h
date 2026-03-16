#ifndef PID_H
#define PID_H

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

#endif // PID_H
