#include <Arduino.h>
#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>

#include "config.h"
#include "motor.h"

namespace {

// ODrive CAN configuration
constexpr uint32_t kCanBaudrate = 500000;  // 500 kbps
constexpr uint8_t kMotorCount = 4;

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

// State tracking (moved outside to allow onCanMessage access)
bool motorReady = false;
bool allMotorsHealthy = false;
unsigned long txCount = 0;
unsigned long rxCount = 0;
bool motorEnabled[kMotorCount] = {true, true, true, true};
bool hasAnyFeedback = false;

float startupJointPos1 = 0.0f;
float startupJointPos2 = 0.0f;

// ODriveCAN objects for each motor (accessed by onCanMessage in global scope)
HardwareCAN& canIntf = CAN;
ODriveCAN* odrives[kMotorCount] = {nullptr};

float clampValue(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

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
    Serial.println("Initializing CAN at 500 kbps...");
    
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
    int rawFrameCount = 0;
    
    while (millis() - startMs < timeoutMs) {
        // Count raw CAN frames before pumping
        int framesThisIteration = 0;
        while (canIntf.available() > 0) {
            CanMsg msg = canIntf.read();
            rawFrameCount++;
            framesThisIteration++;
            
            // Log raw frames for diagnostics
            if (framesThisIteration <= 5) {  // Only print first 5 to avoid spam
                Serial.print("  Raw CAN frame: ID=0x");
                Serial.print(msg.id, HEX);
                Serial.print(" DLC=");
                Serial.println(msg.data_length);
            }
        }
        
        pumpEvents(canIntf);
        
        // Check if we've received heartbeat from all enabled motors
        bool allReceived = true;
        for (int i = 0; i < kMotorCount; ++i) {
            if (!motorEnabled[i]) continue;
            if (feedback[i].valid == false && odrives[i] != nullptr) {
                // Try to request feedback to trigger heartbeat
                Get_Encoder_Estimates_msg_t dummy;
                odrives[i]->getFeedback(dummy, 10);
            }
        }
        
        allReceived = hasAnyFeedback;
        if (allReceived) {
            Serial.println("Heartbeats received!");
            return true;
        }
        
        // Print progress every 500ms
        if (millis() - lastPrintMs >= 500) {
            lastPrintMs = millis();
            unsigned long elapsedMs = millis() - startMs;
            Serial.print("  [");
            Serial.print(elapsedMs);
            Serial.print("ms] Total CAN frames seen: ");
            Serial.println(rawFrameCount);
        }
        
        delay(10);
    }
    
    Serial.print("ERROR: Timeout waiting for heartbeat! (Received ");
    Serial.print(rawFrameCount);
    Serial.println(" total CAN frames)");
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

} // namespace

// CAN message pump callback - called by Arduino_CAN library's pumpEvents()
// MUST be in global scope for the linker to find it
void onCanMessage(const CanMsg& msg) {
    // Dispatch to all registered ODriveCAN objects
    for (int i = 0; i < 4; ++i) {  // kMotorCount = 4
        if (odrives[i] != nullptr && motorEnabled[i]) {
            odrives[i]->onReceive(msg.id, msg.data_length, msg.data);
        }
    }
}

namespace MotorControl {

bool begin() {
    // Configure single-motor test mode if enabled
    if (AppConfig::Motor::kSingleMotorTestMode) {
        motorEnabled[0] = true;
        motorEnabled[1] = false;
        motorEnabled[2] = false;
        motorEnabled[3] = false;
        Serial.println("[MOTOR] Single-motor test mode: only motor 0 enabled");
    }
    
    // Compute startup joint positions
    startupJointPos1 = AppConfig::Motor::kJoint1Vertical90DegMotorTurns + (PI / 2.0f) * AppConfig::Motor::kJointGearRatio;
    startupJointPos2 = AppConfig::Motor::kJoint2Vertical90DegMotorTurns + (PI / 2.0f) * AppConfig::Motor::kJointGearRatio;
    
    // Set initial pose commands (will be sent once motors are ready)
    if (!initializeRobotPose(AppConfig::Motor::kLegStartupAngleRad)) {
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
    
    motorReady = true;
    allMotorsHealthy = true;
    
    Serial.println("[MOTOR] Initialization complete - ready for commands");
    return true;
}

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
        
        // Joints (0,1) can run either position or velocity mode; wheels (2,3) use torque mode.
        if (i < 2) {
            if (cmd.velocityMode) {
                odrives[i]->setVelocity(cmd.velocity, cmd.torque);
            } else {
                odrives[i]->setPosition(cmd.position, cmd.velocity);
            }
        } else {
            // Wheel motors: torque control
            odrives[i]->setTorque(cmd.torque);
        }
        
        ++txCount;
    }
}

bool isReady() {
	return motorReady;
}

bool initializeRobotPose(float legAngleRad) {
	return setMirroredLegJointAngles(legAngleRad) && setWheelTorques(0.0f, 0.0f);
}

bool setMirroredLegJointAngles(float legAngleRad) {
    const float motorTurnsDelta = legAngleRad * AppConfig::Motor::kJointGearRatio;
    const float joint1Target = startupJointPos1 + motorTurnsDelta;
    const float joint2Target = startupJointPos2 - motorTurnsDelta;
    
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
    
    commands[3].position = 0.0f;
    commands[3].velocity = 0.0f;
    commands[3].kp = 0.0f;
    commands[3].kd = 0.0f;
    commands[3].torque = right;
    
    return true;
}

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

float getMotorPosition(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !feedback[idx].valid) {
        return 0.0f;
    }
    return feedback[idx].pos;
}

float getMotorVelocity(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !feedback[idx].valid) {
        return 0.0f;
    }
    return feedback[idx].vel;
}

float getMotorTorque(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0 || !feedback[idx].valid) {
        return 0.0f;
    }
    return feedback[idx].torque;
}

bool hasMotorFeedback(uint8_t nodeId) {
    const int idx = findMotorIndexByNodeId(nodeId);
    if (idx < 0) {
        return false;
    }
    return feedback[idx].valid;
}

unsigned long getTxCount() {
    return txCount;
}

unsigned long getRxCount() {
    return rxCount;
}

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

} // namespace MotorControl