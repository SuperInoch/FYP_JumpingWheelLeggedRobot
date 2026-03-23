#include "xbox_controller.h"

// Static member definitions
XboxControllerData XboxController::controllerData = {};
uint32_t XboxController::lastPacketTime = 0;
uint16_t XboxController::errorCount = 0;
uint8_t XboxController::rxBuffer[XboxController::BUFFER_SIZE] = {};
uint8_t XboxController::bufferIndex = 0;

void XboxController::begin() {
    // Initialize Serial2 (UART2) on pins 16 (RX2) and 17 (TX2)
    Serial2.begin(UART_BAUDRATE);
    
    // Initialize controller data to neutral state
    controllerData.leftStickX = 128;
    controllerData.leftStickY = 128;
    controllerData.rightStickX = 128;
    controllerData.rightStickY = 128;
    controllerData.leftTrigger = 0;
    controllerData.rightTrigger = 0;
    controllerData.buttons = 0;
    controllerData.timestamp = millis();
    
    lastPacketTime = millis();
    errorCount = 0;
    bufferIndex = 0;
}

bool XboxController::update() {
    bool packetReceived = false;
    
    // Read all available bytes from Serial2
    while (Serial2.available()) {
        uint8_t byte = Serial2.read();
        processByte(byte);
        packetReceived = true;  // At least some data was received
    }
    
    return packetReceived;
}

const XboxControllerData& XboxController::getData() {
    return controllerData;
}

bool XboxController::isConnected() {
    uint32_t timeSinceLastPacket = millis() - lastPacketTime;
    return timeSinceLastPacket < CONNECTION_TIMEOUT_MS;
}

uint32_t XboxController::getTimeSinceLastPacket() {
    return millis() - lastPacketTime;
}

uint16_t XboxController::getErrorCount() {
    return errorCount;
}

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

bool XboxController::verifyChecksum(const uint8_t* packet) {
    // Simple XOR checksum: sum all data bytes XORed together
    uint8_t checksum = 0;
    for (int i = 1; i < PACKET_SIZE - 1; i++) {
        checksum ^= packet[i];
    }
    return checksum == packet[PACKET_SIZE - 1];
}

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
