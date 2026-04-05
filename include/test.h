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
// 1. Commands motors to 0.0 position first (reference point)
// 2. Waits for motors to reach 0 position
// 3. Commands motors to kStandardJointAngle
// 4. Waits up to timeoutMs for motors to reach target (within tolerance)
// 5. Prints progress and final position to Serial
// 6. Returns true if successfully reached standard position, false on timeout
bool rotateToStandardPosition(unsigned long timeoutMs = 5000) {
    Serial.println("\n========== MOTOR CALIBRATION: ROTATE TO STANDARD POSITION ==========");
    
    float calibrationMotor1Pos = 0.0f;
    float calibrationMotor2Pos = 0.0f;

    // Step 1: Move to zero position first
    Serial.println("Step 1: Moving motors to reference position (0.0 rad)...");
    MotorControl::setMotorAbsolutePosition(AppConfig::Motor::kJointMotorLeftNodeId, 0.0f);
    MotorControl::setMotorAbsolutePosition(AppConfig::Motor::kJointMotorRightNodeId, 0.0f);
    MotorControl::setMirroredLegJointAngles(0.0f);
    MotorControl::setWheelTorques(0.0f, 0.0f);
    
    const unsigned long zeroStartMs = millis();
    const float toleranceRad = 0.1f;
    bool zeroReached = false;
    
    while (millis() - zeroStartMs < 3000) {  // 3 second timeout for zero position
        MotorControl::update();
        
        const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
        const float motor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
        const float currentJointAngle = AppConfig::Motor::kLegStartupAngleRad +
            ((motor1Pos - AppConfig::Motor::kJoint1Vertical90DegMotorTurns) / 
             (AppConfig::Motor::kJointGearRatio * AppConfig::Motor::kJoint1LegDirectionSign));
        
        if (abs(currentJointAngle - 0.0f) <= toleranceRad) {
            zeroReached = true;
            calibrationMotor1Pos = motor1Pos;
            calibrationMotor2Pos = motor2Pos;
            Serial.println("  ✓ Reached zero position");
            break;
        }
        delay(20);
    }
    
    if (!zeroReached) {
        Serial.println("  ✗ Failed to reach zero position, continuing anyway...");
        calibrationMotor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
        calibrationMotor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
    }
    
    // Step 2: Move to standard position
    Serial.print("Step 2: Moving motors to standard angle: ");
    Serial.print(AppConfig::Motor::kStandardJointAngle, 4);
    Serial.println(" rad");
    
    MotorControl::setMirroredLegJointAngles(AppConfig::Motor::kStandardJointAngle);
    MotorControl::setWheelTorques(0.0f, 0.0f);
    
    const unsigned long startMs = millis();
    bool targetReached = false;
    unsigned long lastPrintMs = startMs;

    Serial.print("Starting positions: m1-pos:0.000, m2-pos:0.000 (calibration locked)");
    Serial.println();
    
    while (millis() - startMs < timeoutMs) {
        // Pump CAN events to receive feedback
        MotorControl::update();
        
        // Get current motor positions and convert to joint angle
        const float motor1Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorLeftNodeId);
        const float motor2Pos = MotorControl::getMotorPosition(AppConfig::Motor::kJointMotorRightNodeId);
        const float motor1RelPos = motor1Pos - calibrationMotor1Pos;
        const float motor2RelPos = motor2Pos - calibrationMotor2Pos;
        
        const float currentJointAngle = AppConfig::Motor::kLegStartupAngleRad +
            ((motor1RelPos) / 
             (AppConfig::Motor::kJointGearRatio * AppConfig::Motor::kJoint1LegDirectionSign));
        
        const float angleError = abs(currentJointAngle - AppConfig::Motor::kStandardJointAngle);
        
        // Print progress every 500ms
        if (millis() - lastPrintMs >= 500) {
            lastPrintMs = millis();
            Serial.print("  [");
            Serial.print(millis() - startMs);
            Serial.print("ms] m1-pos:");
            Serial.print(motor1RelPos, 3);
            Serial.print(", m2-pos:");
            Serial.print(motor2RelPos, 3);
            Serial.print(", Current angle: ");
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
    const float motor1FinalRel = motor1Final - calibrationMotor1Pos;
    const float motor2FinalRel = motor2Final - calibrationMotor2Pos;
    
    const float finalJointAngle = AppConfig::Motor::kLegStartupAngleRad +
        ((motor1FinalRel) / 
         (AppConfig::Motor::kJointGearRatio * AppConfig::Motor::kJoint1LegDirectionSign));
    
    Serial.print("Motor calibration result: ");
    if (targetReached) {
        Serial.println("SUCCESS");
        Serial.print("Final joint angle: ");
        Serial.print(finalJointAngle, 4);
        Serial.println(" rad");
        Serial.print("Motor positions - m1-pos:");
        Serial.print(motor1FinalRel, 3);
        Serial.print(", m2-pos:");
        Serial.println(motor2FinalRel, 3);
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
