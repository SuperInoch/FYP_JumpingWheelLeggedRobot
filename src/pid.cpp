#include "pid.h"

static float clampValue(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), previousError_(0.0f), previousOutput_(0.0f), minOutput_(-5.0f),
      maxOutput_(5.0f), outputRampPerSecond_(0.0f) {}

void PIDController::setGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(float minOutput, float maxOutput) {
  minOutput_ = minOutput;
  maxOutput_ = maxOutput;
}

void PIDController::setOutputRamp(float rampPerSecond) {
  outputRampPerSecond_ = rampPerSecond;
}

void PIDController::reset() {
  integral_ = 0.0f;
  previousError_ = 0.0f;
  previousOutput_ = 0.0f;
}

float PIDController::compute(float setpoint, float measurement, float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    dtSeconds = 0.001f;
  }
  if (dtSeconds > 0.5f) {
    dtSeconds = 0.001f;
  }

  const float error = setpoint - measurement;

  // Match lesson implementation: Tustin integral with bounded integrator.
  integral_ += ki_ * dtSeconds * 0.5f * (error + previousError_);
  integral_ = clampValue(integral_, minOutput_ / 3.0f, maxOutput_ / 3.0f);

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
