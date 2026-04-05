#ifndef MOTOR_H
#define MOTOR_H

namespace MotorControl {

// Initializes CAN + ODrive motor interfaces and enters closed-loop mode.
// Output: true if motors are ready to accept commands.
bool begin();
// Runs one motor service step: receive CAN feedback and send latest commands.
void update();
// Output: true when initialization has completed successfully.
bool isReady();

// Input: desired startup leg angle in radians.
// Output: true if pose and zero-torque commands were accepted.
bool initializeRobotPose(float legAngleRad);
// Input: leg angle in radians; function maps to mirrored left/right joints.
// Output: true when joint targets were updated.
bool setMirroredLegJointAngles(float legAngleRad);
// Input: left/right wheel torques before sign correction and clamping.
// Output: true when wheel torque targets were updated.
bool setWheelTorques(float leftTorque, float rightTorque);
// Input: nodeId and position-control parameters for one motor.
// Output: true if nodeId exists and command was stored.
bool setMotorPosition(uint8_t nodeId, float position, float kp, float kd, float velocityFeedforward = 0.0f);
// Input: nodeId, target velocity, and optional torque feedforward.
// Output: true if nodeId exists and command was stored.
bool setMotorVelocity(uint8_t nodeId, float velocity, float torqueFeedforward = 0.0f);
// Input: nodeId and absolute encoder position to assign.
// Output: true if nodeId exists and the absolute position command was stored.
bool setMotorAbsolutePosition(uint8_t nodeId, float absolutePosition);

// Input: nodeId to query.
// Output: joint angle in radians relative to the zero pose (motor2 is mirrored negative).
float getJointAngleRad(uint8_t nodeId);

// Input: nodeId to query.
// Output: joint angle in degrees relative to the zero pose.
float getJointAngleDeg(uint8_t nodeId);

// Input: nodeId to query.
// Output: latest position estimate (0.0 if unavailable).
float getMotorPosition(uint8_t nodeId);
// Input: nodeId to query.
// Output: latest velocity estimate (0.0 if unavailable).
float getMotorVelocity(uint8_t nodeId);
// Input: nodeId to query.
// Output: latest torque estimate (0.0 if unavailable).
float getMotorTorque(uint8_t nodeId);
// Input: nodeId to query.
// Output: true if feedback has ever been received for that motor.
bool hasMotorFeedback(uint8_t nodeId);
// Input: nodeId to query.
// Output: last heartbeat axis state (0 if unavailable).
uint8_t getAxisState(uint8_t nodeId);
// Input: nodeId to query.
// Output: last heartbeat axis error bitmask (0 if unavailable).
uint32_t getAxisError(uint8_t nodeId);

// Compatibility helper used by callers that expect a feedback-ready flag.
bool hasNewFeedback();

// Output: total number of transmitted command frames.
unsigned long getTxCount();
// Output: total number of failed command transmissions.
unsigned long getTxFailCount();
// Output: total number of received feedback/status callbacks.
unsigned long getRxCount();
// Output: true if at least one motor feedback packet has ever been seen.
bool hasFeedbackEver();
// Compatibility helper for legacy "TX health" checks.
bool lastTxSucceeded();

// Safety: Read current joint angles and check against physical limits.
// Returns true if both joints are within safe range.
// If limits exceeded: disables motors, prints error, and halts controller.
// Accounts for motor direction inversion (motor1 clockwise, motor2 anti-clockwise).
bool checkJointLimits();

} // namespace MotorControl

#endif // MOTOR_H
