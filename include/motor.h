#ifndef MOTOR_H
#define MOTOR_H

namespace MotorControl {

bool begin();
void update();
bool isReady();

bool initializeRobotPose(float legAngleRad);
bool setMirroredLegJointAngles(float legAngleRad);
bool setWheelTorques(float leftTorque, float rightTorque);
bool setMotorPosition(uint8_t nodeId, float position, float kp, float kd, float velocityFeedforward = 0.0f);
bool setMotorVelocity(uint8_t nodeId, float velocity, float torqueFeedforward = 0.0f);

float getMotorPosition(uint8_t nodeId);
float getMotorVelocity(uint8_t nodeId);
float getMotorTorque(uint8_t nodeId);
bool hasMotorFeedback(uint8_t nodeId);

bool hasNewFeedback();

unsigned long getTxCount();
unsigned long getRxCount();
bool hasFeedbackEver();
bool lastTxSucceeded();

} // namespace MotorControl

#endif // MOTOR_H
