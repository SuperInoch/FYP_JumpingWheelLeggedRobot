#ifndef TEST_H
#define TEST_H

#include <Arduino.h>
#include "config.h"
#include "motor.h"

namespace TestMode {

// Rotates both joint motors to the standard (balanced upright) position.
// This is useful for mechanical calibration and verification before legs are installed.
// 
// Procedure:
// 1. Commands motors to kStandardJointAngle
// 2. Waits up to timeoutMs for motors to reach target (within tolerance)
// 3. Prints progress and final position to Serial
// 4. Returns true if successfully reached target, false on timeout
bool rotateToStandardPosition(unsigned long timeoutMs = 5000) {
    Serial.println("\n========== MOTOR CALIBRATION: ROTATE TO STANDARD POSITION ==========");
    Serial.print("Target joint angle: ");
    Serial.print(AppConfig::Motor::kStandardJointAngle, 4);
    Serial.println(" rad");
    
    // Command motors to standard position
    MotorControl::setMirroredLegJointAngles(AppConfig::Motor::kStandardJointAngle);
    MotorControl::setWheelTorques(0.0f, 0.0f);  // Keep wheels idle during calibration
    
    const unsigned long startMs = millis();
    const float toleranceRad = 0.1f;  // Position tolerance in radians
    bool targetReached = false;
    unsigned long lastPrintMs = startMs;
    
    while (millis() - startMs < timeoutMs) {
        // Pump CAN events to receive feedback
        MotorControl::update();
        
        // Get current motor positions and convert to joint angle
        const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
        const float motor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
        
        const float currentJointAngle = AppConfig::Motor::kLegStartupAngleRad +
            ((motor1Pos - AppConfig::Motor::kJoint1Vertical90DegMotorTurns) / 
             (AppConfig::Motor::kJointGearRatio * AppConfig::Motor::kJoint1LegDirectionSign));
        
        const float angleError = abs(currentJointAngle - AppConfig::Motor::kStandardJointAngle);
        
        // Print progress every 500ms
        if (millis() - lastPrintMs >= 500) {
            lastPrintMs = millis();
            Serial.print("  [");
            Serial.print(millis() - startMs);
            Serial.print("ms] Current angle: ");
            Serial.print(currentJointAngle, 4);
            Serial.print(" rad, Error: ");
            Serial.print(angleError, 4);
            Serial.println(" rad");
        }
        
        // Check if target reached within tolerance
        if (angleError <= toleranceRad) {
            targetReached = true;
            break;
        }
        
        delay(20);
    }
    
    // Final status report
    const float motor1Final = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
    const float motor2Final = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
    
    const float finalJointAngle = AppConfig::Motor::kLegStartupAngleRad +
        ((motor1Final - AppConfig::Motor::kJoint1Vertical90DegMotorTurns) / 
         (AppConfig::Motor::kJointGearRatio * AppConfig::Motor::kJoint1LegDirectionSign));
    
    Serial.print("Motor calibration result: ");
    if (targetReached) {
        Serial.println("SUCCESS");
        Serial.print("Final joint angle: ");
        Serial.print(finalJointAngle, 4);
        Serial.println(" rad");
    } else {
        Serial.println("TIMEOUT - target not reached");
        Serial.print("Final joint angle: ");
        Serial.print(finalJointAngle, 4);
        Serial.print(" rad (target: ");
        Serial.print(AppConfig::Motor::kStandardJointAngle, 4);
        Serial.println(" rad)");
    }
    Serial.println("====================================================================\n");
    
    return targetReached;
}

}  // namespace TestMode

#endif  // TEST_H
