#include <Arduino.h>
#include <cstdlib>
#include "xbox_controller.h"

// Static member definitions
XboxControllerData XboxController::controllerData = {};
uint32_t XboxController::lastPacketTime = 0;
uint16_t XboxController::errorCount = 0;
uint8_t XboxController::rxBuffer[XboxController::BUFFER_SIZE] = {};
uint8_t XboxController::bufferIndex = 0;

// Initializes UART receiver and resets controller state to neutral defaults.
void XboxController::begin() {
    // Initialize the board-supported hardware UART.
    // UNO R4 Minima exposes Serial1 in the Arduino core.
    Serial1.begin(UART_BAUDRATE);
    
    // Initialize controller data to neutral state
    controllerData.leftStickX = 128;
    controllerData.leftStickY = 128;
    controllerData.rightStickX = 128;
    controllerData.rightStickY = 128;
    controllerData.leftTrigger = 0;
    controllerData.rightTrigger = 0;
    controllerData.buttons = 0;
    controllerData.timestamp = millis();
    
    // Mark as disconnected until first valid packet is decoded.
    lastPacketTime = 0;
    errorCount = 0;
    bufferIndex = 0;
}

// Pulls bytes from UART and feeds packet parser state machine.
bool XboxController::update() {
    bool packetReceived = false;
    
    // Read all available bytes from Serial1
    while (Serial1.available()) {
        uint8_t byte = Serial1.read();
        processByte(byte);
        packetReceived = true;  // At least some data was received
    }
    
    return packetReceived;
}

// Returns latest decoded controller payload.
const XboxControllerData& XboxController::getData() {
    return controllerData;
}

// Reports connection as active when fresh packets and non-neutral data are present.
bool XboxController::isConnected() {
    // Connection should depend on transport freshness, not input activity.
    // Neutral sticks/buttons are still a valid connected state.
    if (lastPacketTime == 0) {
        return false;
    }

    uint32_t timeSinceLastPacket = millis() - lastPacketTime;
    return timeSinceLastPacket < CONNECTION_TIMEOUT_MS;
}

// Returns packet age in milliseconds.
uint32_t XboxController::getTimeSinceLastPacket() {
    return millis() - lastPacketTime;
}

// Returns cumulative checksum/parse error counter.
uint16_t XboxController::getErrorCount() {
    return errorCount;
}

// Filters out all-neutral packets to avoid false-positive "connected" states.
bool XboxController::isDataActive() {
    // Define "neutral" position with deadband
    // Xbox sticks center around 128, but may vary slightly
    const uint8_t NEUTRAL_CENTER = 128;
    const uint8_t DEADBAND = 20;  // Consider 108-148 as neutral
    
    // Check if sticks are significantly away from center
    if (abs((int)controllerData.leftStickX - NEUTRAL_CENTER) > DEADBAND ||
        abs((int)controllerData.leftStickY - NEUTRAL_CENTER) > DEADBAND ||
        abs((int)controllerData.rightStickX - NEUTRAL_CENTER) > DEADBAND ||
        abs((int)controllerData.rightStickY - NEUTRAL_CENTER) > DEADBAND) {
        return true;
    }
    
    // Check if triggers are pressed (>10 range out of 0-255)
    if (controllerData.leftTrigger > 10 || controllerData.rightTrigger > 10) {
        return true;
    }
    
    // Check if any buttons are pressed (0 = no buttons)
    if (controllerData.buttons != 0) {
        return true;
    }
    
    // All values are neutral - likely no real controller input
    return false;
}

// Incrementally assembles packets and dispatches valid ones to parser.
void XboxController::processByte(uint8_t byte) {
    // Look for packet start marker
    if (bufferIndex == 0) {
        if (byte == PACKET_HEADER) {
            rxBuffer[bufferIndex++] = byte;
        }
        return;
    }
    
    // Add byte to buffer
    rxBuffer[bufferIndex++] = byte;
    
    // Check if we have a complete packet
    if (bufferIndex >= PACKET_SIZE) {
        // Verify checksum
        if (verifyChecksum(rxBuffer)) {
            // Parse the valid packet
            parsePacket(rxBuffer);
            lastPacketTime = millis();
            controllerData.timestamp = millis();
        } else {
            // Bad checksum, increment error counter
            errorCount++;
        }
        
        // Reset buffer for next packet
        bufferIndex = 0;
    }
}

// Validates packet checksum using XOR over payload bytes.
bool XboxController::verifyChecksum(const uint8_t* packet) {
    // Simple XOR checksum: sum all data bytes XORed together
    uint8_t checksum = 0;
    for (int i = 1; i < PACKET_SIZE - 1; i++) {
        checksum ^= packet[i];
    }
    return checksum == packet[PACKET_SIZE - 1];
}

// Decodes packet payload fields into controllerData.
void XboxController::parsePacket(const uint8_t* packet) {
    // Packet format:
    // [0] = 0xAA (header)
    // [1] = leftStickX (0-255)
    // [2] = leftStickY (0-255)
    // [3] = rightStickX (0-255)
    // [4] = rightStickY (0-255)
    // [5] = leftTrigger (0-255)
    // [6] = rightTrigger (0-255)
    // [7] = buttons (bit-packed)
    // [8] = checksum
    
    controllerData.leftStickX = packet[1];
    controllerData.leftStickY = packet[2];
    controllerData.rightStickX = packet[3];
    controllerData.rightStickY = packet[4];
    controllerData.leftTrigger = packet[5];
    controllerData.rightTrigger = packet[6];
    controllerData.buttons = packet[7];
}
