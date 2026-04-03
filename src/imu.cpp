#include <Arduino.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#include "config.h"
#include "imu.h"

namespace {

MPU6050 mpu;
float fusedRollDeg = 0.0f;
float fusedPitchDeg = 0.0f;
unsigned long lastUpdateMs = 0;

constexpr float kAccCoef = 0.02f;
constexpr float kGyroCoef = 0.98f;
constexpr float kGyroYLowPassAlpha = 0.005f;
constexpr float kGyroZDeadband = 1.0f;
constexpr float kRemoteBalanceOffsetDeg = 0.0f;

// Applies a first-order low-pass filter to smooth noisy sensor values.
float lowPassFilter(float currentValue, float previousValue, float alpha) {
  return alpha * currentValue + (1.0f - alpha) * previousValue;
}

// Clamps a signed integer into the uint8 range [0, 255].
uint8_t clampToByte(long value) {
  if (value < 0) {
    return 0;
  }
  if (value > 255) {
    return 255;
  }
  return static_cast<uint8_t>(value);
}

} // namespace

// Initializes MPU6050 and seeds complementary-filter roll/pitch values.
bool IMUManager::begin() {
  data_ = {0.0f, 0.0f, 0.0f, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  Wire.begin();
  Wire.setClock(AppConfig::IMU::kI2cClockHz);
  mpu.initialize();
  if (!mpu.testConnection()) {
    return false;
  }

  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  const float axf = static_cast<float>(ax);
  const float ayf = static_cast<float>(ay);
  const float azf = static_cast<float>(az);
  fusedRollDeg = atan2f(ayf, azf + fabsf(axf)) * 57.29578f;
  fusedPitchDeg = atan2f(axf, azf + fabsf(ayf)) * -57.29578f;
  lastUpdateMs = millis();
  return true;
}

// Reads IMU sensors and updates both raw and fused orientation outputs.
bool IMUManager::update() {
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Keep your original 0..255 axis mapping from the standalone IMU sketch.
  data_.mappedX = clampToByte(map(ax, AppConfig::IMU::kMapInputMin, AppConfig::IMU::kMapInputMax,
                                 AppConfig::IMU::kMapOutputMin, AppConfig::IMU::kMapOutputMax));
  data_.mappedY = clampToByte(map(ay, AppConfig::IMU::kMapInputMin, AppConfig::IMU::kMapInputMax,
                                 AppConfig::IMU::kMapOutputMin, AppConfig::IMU::kMapOutputMax));
  data_.mappedZ = clampToByte(map(az, AppConfig::IMU::kMapInputMin, AppConfig::IMU::kMapInputMax,
                                 AppConfig::IMU::kMapOutputMin, AppConfig::IMU::kMapOutputMax));

  data_.accelX = ax;
  data_.accelY = ay;
  data_.accelZ = az;
  data_.gyroX = gx;
  data_.gyroY = gy;
  data_.gyroZ = gz;

  const float axf = static_cast<float>(ax);
  const float ayf = static_cast<float>(ay);
  const float azf = static_cast<float>(az);

  const float accRollDeg = atan2f(ayf, azf + fabsf(axf)) * 57.29578f;
  const float accPitchDeg = atan2f(axf, azf + fabsf(ayf)) * -57.29578f;

  unsigned long nowMs = millis();
  float dt = static_cast<float>(nowMs - lastUpdateMs) * 0.001f;
  if (dt <= 0.0f || dt > 0.5f) {
    dt = 0.001f;
  }
  lastUpdateMs = nowMs;

  // Lesson uses gyro scale 65.5 LSB/(deg/s) with corresponding gyro config.
  const float gyroXDegPerSec = static_cast<float>(gx) / 65.5f;
  float gyroYDegPerSec = static_cast<float>(gy) / 65.5f;
  float gyroZDegPerSec = static_cast<float>(gz) / 65.5f;

  gyroYDegPerSec = lowPassFilter(gyroYDegPerSec, data_.yawRateDegPerSec, AppConfig::IMU::kGyroYLowPassAlpha);
  if (gyroZDegPerSec > -AppConfig::IMU::kGyroZDeadband && gyroZDegPerSec < AppConfig::IMU::kGyroZDeadband) {
    gyroZDegPerSec = 0.0f;
  }

  fusedRollDeg =
      (AppConfig::IMU::kGyroCoef * (fusedRollDeg + gyroXDegPerSec * dt)) + (AppConfig::IMU::kAccCoef * accRollDeg);
  fusedPitchDeg =
      (AppConfig::IMU::kGyroCoef * (fusedPitchDeg + gyroYDegPerSec * dt)) + (AppConfig::IMU::kAccCoef * accPitchDeg);

  data_.rollDeg = fusedRollDeg;
  data_.pitchDeg = fusedPitchDeg - AppConfig::IMU::kRemoteBalanceOffsetDeg;

  data_.yawRateDegPerSec = gyroYDegPerSec;

  return true;
}

// Returns the latest IMU sample structure.
const ImuData& IMUManager::data() const {
  return data_;
}
