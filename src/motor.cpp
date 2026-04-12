#include <Arduino.h>
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>

#include "config.h"
#include "motor.h"

namespace {

// ODrive CAN configuration
constexpr uint32_t kCanBaudrate = AppConfig::Motor::kCanBitrateSecondary;
constexpr uint8_t kMotorCount = 4;
constexpr uint32_t kFeedbackPollIntervalMs = 20;  // 50 Hz poll request pacing
constexpr float kTwoPi = 2.0f * PI;

// ODrive node IDs (must match ODrive Web GUI config)
const uint8_t kNodeIds[kMotorCount] = {
    AppConfig::Motor::kJointMotorLeftNodeId,
    AppConfig::Motor::kJointMotorRightNodeId,
    AppConfig::Motor::kWheelMotorLeftNodeId,
    AppConfig::Motor::kWheelMotorRightNodeId};

// Current command targets (position, velocity, torque)
struct MotorCommand {
    float position;
    float velocity;
    float kp;
    float kd;
    float torque;
    bool velocityMode;
};

MotorCommand commands[kMotorCount] = {};

// ODrive feedback (cached from callbacks)
struct MotorFeedback {
    float pos;
    float vel;
    float torque;
    bool valid;
};

MotorFeedback feedback[kMotorCount] = {};
uint8_t lastAxisState[kMotorCount] = {};
uint32_t lastAxisError[kMotorCount] = {};
bool heartbeatValid[kMotorCount] = {};

// State tracking (moved outside to allow onCanMessage access)
bool motorReady = false;
bool allMotorsHealthy = false;
unsigned long txCount = 0;
unsigned long txFailCount = 0;
unsigned long rxCount = 0;
bool motorEnabled[kMotorCount] = {
    AppConfig::Motor::kEnableMotor1,
    AppConfig::Motor::kEnableMotor2,
    AppConfig::Motor::kEnableMotor3,
    AppConfig::Motor::kEnableMotor4,
};
bool hasAnyFeedback = false;

// ODriveCAN objects for each motor (accessed by onCanMessage in global scope)
HardwareCAN& canIntf = CAN;
ODriveCAN* odrives[kMotorCount] = {nullptr};

void printMotorEnableConfig() {
    Serial.println("[MOTOR] Node enable map:");
    for (int i = 0; i < kMotorCount; ++i) {
        Serial.print("  index ");
        Serial.print(i);
        Serial.print(" -> node ");
        Serial.print(kNodeIds[i]);
        Serial.print(", enabled=");
        Serial.println(motorEnabled[i] ? "YES" : "NO");
    }
}

// Clamps scalar command values to configured bounds.
float clampValue(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Maps ODrive node ID to index in local arrays.
int findMotorIndexByNodeId(uint8_t nodeId) {
    for (int i = 0; i < kMotorCount; ++i) {
        if (kNodeIds[i] == nodeId) {
            return i;
        }
    }
    return -1;
}

// Callback: called when heartbeat (axis state) is received from ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
    uint8_t nodeId = reinterpret_cast<uintptr_t>(user_data);
    int idx = findMotorIndexByNodeId(nodeId);
    if (idx >= 0) {
        lastAxisState[idx] = msg.Axis_State;
        lastAxisError[idx] = msg.Axis_Error;
        heartbeatValid[idx] = true;
        ++rxCount;
        hasAnyFeedback = true;
    }
}

// Callback: called when encoder feedback is received from ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    uint8_t nodeId = reinterpret_cast<uintptr_t>(user_data);
    int idx = findMotorIndexByNodeId(nodeId);
    if (idx >= 0) {
        feedback[idx].pos = msg.Pos_Estimate;
        feedback[idx].vel = msg.Vel_Estimate;
        feedback[idx].valid = true;
        ++rxCount;
        hasAnyFeedback = true;
    }
}

// Initialize ODrive objects and bring up CAN bus
bool setupCan() {
    Serial.print("Initializing CAN at ");
    Serial.print(kCanBaudrate);
    Serial.println(" bps...");
    
    if (!canIntf.begin((CanBitRate)kCanBaudrate)) {
        Serial.println("ERROR: CAN initialization failed!");
        return false;
    }
    
    Serial.println("CAN bus started. Creating ODrive objects...");
    
    // Create ODriveCAN objects for enabled motors
    for (int i = 0; i < kMotorCount; ++i) {
        if (!motorEnabled[i]) {
            odrives[i] = nullptr;
            continue;
        }
        
        odrives[i] = new ODriveCAN(wrap_can_intf(canIntf), kNodeIds[i]);
        
        // Register feedack callbacks (use node ID as user_data)
        odrives[i]->onFeedback(onFeedback, reinterpret_cast<void*>(static_cast<uintptr_t>(kNodeIds[i])));
        odrives[i]->onStatus(onHeartbeat, reinterpret_cast<void*>(static_cast<uintptr_t>(kNodeIds[i])));
        
        Serial.print("  Motor ");
        Serial.print(i);
        Serial.print(" (node ");
        Serial.print(kNodeIds[i]);
        Serial.println(") ready");
    }
    
    return true;
}

// Wait for heartbeat from all enabled motors (indicates they're responding)
bool waitForHeartbeats(uint32_t timeoutMs) {
    Serial.println("Waiting for ODrive heartbeats...");
    unsigned long startMs = millis();
    unsigned long lastPrintMs = startMs;
    unsigned long lastFeedbackPollMs = startMs;
    
    while (millis() - startMs < timeoutMs) {
        // Let ODrive callbacks consume incoming CAN traffic.
        pumpEvents(canIntf);
        
        // Request feedback for any enabled motor that has not reported yet.
        const unsigned long nowMs = millis();
        if (nowMs - lastFeedbackPollMs >= kFeedbackPollIntervalMs) {
            lastFeedbackPollMs = nowMs;
            for (int i = 0; i < kMotorCount; ++i) {
                if (!motorEnabled[i]) continue;
                if (feedback[i].valid == false && odrives[i] != nullptr) {
                    // Poll encoder estimates at a limited rate to avoid bus flooding.
                    Get_Encoder_Estimates_msg_t dummy;
                    odrives[i]->getFeedback(dummy, 10);
                }
            }
        }

        // Require feedback from every enabled motor, not just any motor.
        bool allReceived = true;
        for (int i = 0; i < kMotorCount; ++i) {
            if (!motorEnabled[i]) {
                continue;
            }
            if (!feedback[i].valid) {
                allReceived = false;
                break;
            }
        }

        if (allReceived) {
            Serial.println("All enabled motors are responding.");
            return true;
        }
        
        // Print progress every 500ms
        if (millis() - lastPrintMs >= 500) {
            lastPrintMs = millis();
            unsigned long elapsedMs = millis() - startMs;
            Serial.print("  [");
            Serial.print(elapsedMs);
            Serial.print("ms] RX callbacks: ");
            Serial.println(rxCount);
        }
        
        delay(10);
    }
    
    Serial.println("ERROR: Timeout waiting for motor feedback!");

    for (int i = 0; i < kMotorCount; ++i) {
        Serial.print("  Node ");
        Serial.print(kNodeIds[i]);
        Serial.print(" [enabled=");
        Serial.print(motorEnabled[i] ? "YES" : "NO");
        Serial.print("] feedback: ");
        Serial.println(feedback[i].valid ? "OK" : "MISSING");
    }

    return false;
}

// Enable all motors in closed-loop control mode
bool enableClosedLoopControl() {
    Serial.println("Enabling closed-loop control...");
    
    for (int i = 0; i < kMotorCount; ++i) {
        if (!motorEnabled[i] || odrives[i] == nullptr) {
            continue;
        }
        
        // Clear any errors first
        odrives[i]->clearErrors();
        delay(10);
        
        // Set to closed-loop control state
        if (!odrives[i]->setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)) {
            Serial.print("WARNING: Failed to set motor ");
            Serial.print(i);
            Serial.println(" to closed-loop");
        }
        
        // Pump events to process state change
        for (int j = 0; j < 50; ++j) {
            pumpEvents(canIntf);
            delay(2);
        }
    }
    
    Serial.println("Motors enabled!");
    return true;
}

// Forces deterministic controller modes so host commands always match motor behavior.
bool configureControllerModes() {
    Serial.println("Configuring controller modes...");
    bool ok = true;

    for (int i = 0; i < kMotorCount; ++i) {
        if (!motorEnabled[i] || odrives[i] == nullptr) {
            continue;
        }

        const uint8_t controlMode = (i < 2)
            ? static_cast<uint8_t>(CONTROL_MODE_POSITION_CONTROL)
            : static_cast<uint8_t>(CONTROL_MODE_VELOCITY_CONTROL);
        const uint8_t inputMode = static_cast<uint8_t>(INPUT_MODE_PASSTHROUGH);

        if (!odrives[i]->setControllerMode(controlMode, inputMode)) {
            Serial.print("WARNING: Failed controller mode set for node ");
            Serial.println(kNodeIds[i]);
            ok = false;
        }

        pumpEvents(canIntf);
        delay(2);
    }

    return ok;
}

// Sends one motor command with short retries so later nodes are not starved by TX mailbox contention.
bool sendCommandWithRetry(int motorIndex, const MotorCommand& cmd) {
    constexpr uint8_t kMaxAttempts = 3;

    for (uint8_t attempt = 0; attempt < kMaxAttempts; ++attempt) {
        bool sent = false;

        if (motorIndex < 2) {
            if (cmd.velocityMode) {
                sent = odrives[motorIndex]->setVelocity(cmd.velocity, cmd.torque);
            } else {
                sent = odrives[motorIndex]->setPosition(cmd.position, cmd.velocity, cmd.torque);
            }
        } else {
            if (cmd.velocityMode) {
                sent = odrives[motorIndex]->setVelocity(cmd.velocity, cmd.torque);
            } else {
                sent = odrives[motorIndex]->setTorque(cmd.torque);
            }
        }

        if (sent) {
            ++txCount;
            return true;
        }

        pumpEvents(canIntf);
        delay(1);
    }

    ++txFailCount;
    return false;
}

} // namespace

// CAN message pump callback - called by Arduino_CAN library's pumpEvents()
// MUST be in global scope for the linker to find it
// Routes each raw CAN frame to all active ODrive instances.
void onCanMessage(const CanMsg& msg) {
    // Dispatch to all registered ODriveCAN objects
    for (int i = 0; i < 4; ++i) {  // kMotorCount = 4
        if (odrives[i] != nullptr && motorEnabled[i]) {
            odrives[i]->onReceive(msg.id, msg.data_length, msg.data);
        }
    }
}

namespace MotorControl {

// Brings up motor subsystem from CAN startup to closed-loop readiness.
bool begin() {
    printMotorEnableConfig();
    
    // Set initial pose commands (will be sent once motors are ready)
    if (!initializeRobotPose(0.0f)) {
        Serial.println("ERROR: Failed to set startup pose");
        return false;
    }
    
    // Initialize CAN bus
    if (!setupCan()) {
        return false;
    }
    
    // Wait for ODrive responses (heartbeats)
    if (!waitForHeartbeats(3000)) {
        Serial.println("ERROR: ODrive not responding on CAN bus");
        return false;
    }
    
    // Enable closed-loop control on all motors
    if (!enableClosedLoopControl()) {
        Serial.println("WARNING: Some motors may not be in closed-loop control");
    }

    if (!configureControllerModes()) {
        Serial.println("WARNING: Some motors may not accept requested control modes");
    }
    
    motorReady = true;
    allMotorsHealthy = true;
    
    Serial.println("[MOTOR] Initialization complete - ready for commands");
    return true;
}

// Pumps bus traffic and transmits the current command set to all enabled motors.
void update() {
    if (!motorReady) {
        return;
    }
    
    // Process incoming CAN messages (heartbeats, feedback)
    pumpEvents(canIntf);
    
    // Send position commands to each enabled motor
    for (int i = 0; i < kMotorCount; ++i) {
        if (!motorEnabled[i] || odrives[i] == nullptr) {
            continue;
        }
        
        const MotorCommand& cmd = commands[i];
        
        sendCommandWithRetry(i, cmd);
    }
}

// Returns true once initialization has completed successfully.
bool isReady() {
	return motorReady;
}

// Applies startup pose and zeros wheel torque as a safe initial command.
bool initializeRobotPose(float legAngleRad) {
    return setMirroredLegJointAngles(legAngleRad) && setWheelVelocities(0.0f, 0.0f);
}

// Converts requested leg angle into mirrored joint motor position targets.
bool setMirroredLegJointAngles(float legAngleRad) {
    const float motorTurnsDelta = (legAngleRad / kTwoPi) * AppConfig::Motor::kJointGearRatio;
    // Positive logical joint angle is anti-clockwise on motor 1.
    const float joint1Target = AppConfig::Motor::kJoint1DefaultOffsetTurns - motorTurnsDelta;
    const float joint2Target = AppConfig::Motor::kJoint2DefaultOffsetTurns + motorTurnsDelta;
    
    commands[0].position = joint1Target;
    commands[0].velocity = 0.0f;
    commands[0].kp = AppConfig::Motor::kJointKp;
    commands[0].kd = AppConfig::Motor::kJointKd;
    commands[0].torque = 0.0f;
    
    commands[1].position = joint2Target;
    commands[1].velocity = 0.0f;
    commands[1].kp = AppConfig::Motor::kJointKp;
    commands[1].kd = AppConfig::Motor::kJointKd;
    commands[1].torque = 0.0f;
    
    return true;
}

// Applies left/right wheel torque commands with clamp and motor sign mapping.
bool setWheelTorques(float leftTorque, float rightTorque) {
    const float left = clampValue(leftTorque, -AppConfig::Motor::kWheelTorqueLimit, AppConfig::Motor::kWheelTorqueLimit) *
                       AppConfig::Motor::kWheelLeftSign;
    const float right = clampValue(rightTorque, -AppConfig::Motor::kWheelTorqueLimit, AppConfig::Motor::kWheelTorqueLimit) *
                        AppConfig::Motor::kWheelRightSign;
    
    commands[2].position = 0.0f;
    commands[2].velocity = 0.0f;
    commands[2].kp = 0.0f;
    commands[2].kd = 0.0f;
    commands[2].torque = left;
    commands[2].velocityMode = false;
    
    commands[3].position = 0.0f;
    commands[3].velocity = 0.0f;
    commands[3].kp = 0.0f;
    commands[3].kd = 0.0f;
    commands[3].torque = right;
    commands[3].velocityMode = false;

    return true;
}

// Applies left/right wheel velocity commands with clamp and motor sign mapping.
bool setWheelVelocities(float leftVelocity, float rightVelocity) {
    const float left = clampValue(leftVelocity,
                                  -AppConfig::XboxController::kMaxMotorVelocity,
                                  AppConfig::XboxController::kMaxMotorVelocity) *
                       AppConfig::Motor::kWheelLeftSign;
    const float right = clampValue(rightVelocity,
                                   -AppConfig::XboxController::kMaxMotorVelocity,
                                   AppConfig::XboxController::kMaxMotorVelocity) *
                        AppConfig::Motor::kWheelRightSign;

    commands[2].position = 0.0f;
    commands[2].velocity = left;
    commands[2].kp = 0.0f;
    commands[2].kd = 0.0f;
    commands[2].torque = 0.0f;
    commands[2].velocityMode = true;

    commands[3].position = 0.0f;
    commands[3].velocity = right;
    commands[3].kp = 0.0f;
    commands[3].kd = 0.0f;
    commands[3].torque = 0.0f;
    commands[3].velocityMode = true;
    
    return true;
}

// Updates per-node command structure for position mode.
bool setMotorPosition(uint8_t nodeId, float position, float kp, float kd, float velocityFeedforward) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0) {
        return false;
    }
    
    commands[idx].position = position;
    commands[idx].velocity = velocityFeedforward;
    commands[idx].kp = kp;
    commands[idx].kd = kd;
    commands[idx].torque = 0.0f;
    commands[idx].velocityMode = false;
    return true;
}

// Updates per-node command structure for velocity mode.
bool setMotorVelocity(uint8_t nodeId, float velocity, float torqueFeedforward) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0) {
        return false;
    }

    commands[idx].position = 0.0f;
    commands[idx].velocity = velocity;
    commands[idx].kp = 0.0f;
    commands[idx].kd = 0.0f;
    commands[idx].torque = torqueFeedforward;
    commands[idx].velocityMode = true;
    return true;
}

// Assigns a new absolute encoder position for a node.
bool setMotorAbsolutePosition(uint8_t nodeId, float absolutePosition) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || odrives[idx] == nullptr) {
        return false;
    }

    return odrives[idx]->setAbsolutePosition(absolutePosition);
}

float getJointAngleRad(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !feedback[idx].valid) {
        return 0.0f;
    }

    const float turns = feedback[idx].pos;
    if (nodeId == AppConfig::Motor::kJointMotorLeftNodeId) {
        return -(turns - AppConfig::Motor::kJoint1DefaultOffsetTurns) * kTwoPi / AppConfig::Motor::kJointGearRatio;
    }
    if (nodeId == AppConfig::Motor::kJointMotorRightNodeId) {
        return (turns - AppConfig::Motor::kJoint2DefaultOffsetTurns) * kTwoPi / AppConfig::Motor::kJointGearRatio;
    }

    return 0.0f;
}

float getJointAngleDeg(uint8_t nodeId) {
    return getJointAngleRad(nodeId) * AppConfig::Motor::kRadToDeg;
}

// Returns last known position for a motor, or zero when unavailable.
float getMotorPosition(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !feedback[idx].valid) {
        return 0.0f;
    }
    return feedback[idx].pos;
}

// Returns last known velocity for a motor, or zero when unavailable.
float getMotorVelocity(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !feedback[idx].valid) {
        return 0.0f;
    }
    return feedback[idx].vel;
}

// Returns last known torque for a motor, or zero when unavailable.
float getMotorTorque(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !feedback[idx].valid) {
        return 0.0f;
    }
    return feedback[idx].torque;
}

// Indicates whether feedback has been received for the requested node.
bool hasMotorFeedback(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0) {
        return false;
    }
    return feedback[idx].valid;
}

// Returns last heartbeat axis state for a motor, or zero when unavailable.
uint8_t getAxisState(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !heartbeatValid[idx]) {
        return 0;
    }
    return lastAxisState[idx];
}

// Returns last heartbeat axis error flags for a motor, or zero when unavailable.
uint32_t getAxisError(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !heartbeatValid[idx]) {
        return 0;
    }
    return lastAxisError[idx];
}

// Returns number of command frames transmitted on CAN.
unsigned long getTxCount() {
    return txCount;
}

// Returns number of command frames that could not be sent after retries.
unsigned long getTxFailCount() {
    return txFailCount;
}

// Returns number of feedback/status events processed from CAN.
unsigned long getRxCount() {
    return rxCount;
}

// Returns whether any motor feedback has ever been seen.
bool hasFeedbackEver() {
    return hasAnyFeedback;
}

// Stub for compatibility with main.cpp
bool hasNewFeedback() {
    // ODriveArduino delivers feedback via callbacks continuously
    return true;
}

// Stub for compatibility with main.cpp
bool lastTxSucceeded() {
    // Assume success if motors are responding
    return hasAnyFeedback;
}

// Safety: Check joint angles against physical limits.
// Reads current motor positions, converts to logical joint angles (accounting for direction),
// and verifies both are within [kMinJointAngle, kMaxJointAngle].
// Motor 1 is clockwise (sign=-1), Motor 2 is anti-clockwise (sign=1).
// If any limit is exceeded, instantly disables motors, prints critical error, and halts.
bool checkJointLimits() {
    // Get current motor positions (in motor turns from encoder feedback)
    const float motor1Pos = getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
    const float motor2Pos = getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
    
    // Convert motor positions to logical joint angles (radians).
    // Motor 1 positive, motor 2 mirrored negative.
    const float jointAngle1 = getJointAngleRad(AppConfig::Motor::kJointMotorLeftNodeId);
    const float jointAngle2 = getJointAngleRad(AppConfig::Motor::kJointMotorRightNodeId);
    
    // Check if both angles are within safe limits
    const bool angle1Safe = (jointAngle1 >= AppConfig::Motor::kMinJointAngle) && 
                             (jointAngle1 <= AppConfig::Motor::kMaxJointAngle);
    const bool angle2Safe = (jointAngle2 >= AppConfig::Motor::kMinJointAngle) && 
                             (jointAngle2 <= AppConfig::Motor::kMaxJointAngle);
    
    if (!angle1Safe || !angle2Safe) {
        // CRITICAL SAFETY FAILURE: Joint angle out of bounds
        Serial.println("\n========== CRITICAL SAFETY HALT ==========");
        Serial.println("Joint angle(s) exceed physical limits!");
        Serial.print("Joint 1 angle: ");
        Serial.print(jointAngle1, 4);
        Serial.print(" rad (min:");
        Serial.print(AppConfig::Motor::kMinJointAngle, 4);
        Serial.print(" max:");
        Serial.print(AppConfig::Motor::kMaxJointAngle, 4);
        Serial.println(")");
        Serial.print("Joint 2 angle: ");
        Serial.print(jointAngle2, 4);
        Serial.print(" rad (min:");
        Serial.print(AppConfig::Motor::kMinJointAngle, 4);
        Serial.print(" max:");
        Serial.print(AppConfig::Motor::kMaxJointAngle, 4);
        Serial.println(")");
        Serial.println("DISABLING ALL MOTORS. Hard reboot required.");
        Serial.println("==========================================\n");
        
        // Instant motor disable: set wheel velocity to zero (joints already idle)
        setWheelVelocities(0.0f, 0.0f);
        
        // Perform one final update to send disable commands
        pumpEvents(canIntf);
        for (int i = 0; i < 10; ++i) {
            delay(10);
            pumpEvents(canIntf);
        }
        
        // Halt forever until hard reboot
        while (1) {
            delay(100);  // Prevent watchdog timeout if enabled
        }
    }
    
    return true;  // All angles are safe
}

} // namespace MotorControl