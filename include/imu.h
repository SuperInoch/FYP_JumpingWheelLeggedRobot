#ifndef IMU_H
#define IMU_H

struct ImuData {
  float pitchDeg;
  float rollDeg;
  float yawRateDegPerSec;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
  uint8_t mappedX;
  uint8_t mappedY;
  uint8_t mappedZ;
};

class IMUManager {
public:
  // Starts I2C and the MPU6050 sensor.
  // Output: true if the IMU responds, false if hardware init failed.
  bool begin();
  // Reads one sensor sample and updates pitch/roll/yaw-rate values.
  // Output: true when a new sample was processed.
  bool update();
  // Gives the latest IMU data struct (already computed and cached).
  // Output: reference to ImuData with raw and fused values.
  const ImuData& data() const;

private:
  ImuData data_ = {0.0f, 0.0f, 0.0f};
};

#endif // IMU_H
