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
  bool begin();
  bool update();
  const ImuData& data() const;

private:
  ImuData data_ = {0.0f, 0.0f, 0.0f};
};

#endif // IMU_H
