#ifndef XBOX_CONTROLLER_H
#define XBOX_CONTROLLER_H

#include <cstdint>
#include <HardwareSerial.h>

/**
 * Xbox controller data structure received from ESP32 via UART2
 * Format: Binary packet with left stick, right stick, and button states
 */
struct XboxControllerData {
    // Analog stick values: 0-255 (centered at ~128)
    uint8_t leftStickX;    // 0 = left, 255 = right
    uint8_t leftStickY;    // 0 = down, 255 = up
    uint8_t rightStickX;   // 0 = left, 255 = right
    uint8_t rightStickY;   // 0 = down, 255 = up
    
    // Trigger values: 0-255
    uint8_t leftTrigger;   // 0 = not pressed, 255 = fully pressed
    uint8_t rightTrigger;  // 0 = not pressed, 255 = fully pressed
    
    // Button states (packed in one byte)
    uint8_t buttons;       // Bit 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=Menu, 7=View
    
    // Timestamp when data was received (milliseconds)
    uint32_t timestamp;
};

/**
 * Xbox Controller receiver - handles UART communication with ESP32
 * Uses Serial2 (Hardware UART2 on Arduino R4 Minima)
 * 
 * Pin connections:
 *   - RX2: Pin 16 (receives data from ESP32 TX)
 *   - TX2: Pin 17 (not used for this receiver, but available for handshake)
 * 
 * Baudrate: 115200
 */
class XboxController {
public:
    /**
     * Initialize Xbox controller receiver on UART2
     * Call this in setup()
     */
    static void begin();
    
    /**
     * Update controller data - reads incoming UART packets
     * Call this frequently in the main loop (>10ms interval recommended)
     * 
     * @return true if new controller data was received, false otherwise
     */
    static bool update();
    
    /**
     * Get the current controller data
     * 
     * @return reference to the current XboxControllerData structure
     */
    static const XboxControllerData& getData();
    
    /**
     * Check if controller is connected (receiving valid data)
     * 
     * @return true if data has been received recently (within 200ms)
     */
    static bool isConnected();
    
    /**
     * Get time since last valid packet received (milliseconds)
     * 
     * @return milliseconds since last packet
     */
    static uint32_t getTimeSinceLastPacket();
    
    /**
     * Get number of malformed packets received (for debugging)
     * 
     * @return count of bad packets
     */
    static uint16_t getErrorCount();
    
private:
    static constexpr uint32_t UART_BAUDRATE = 115200;
    static constexpr uint32_t CONNECTION_TIMEOUT_MS = 200;
    static constexpr uint8_t PACKET_HEADER = 0xAA;  // Start marker
    static constexpr uint8_t PACKET_SIZE = 9;        // Header(1) + Data(7) + Checksum(1)
    static constexpr uint8_t BUFFER_SIZE = 64;
    
    static XboxControllerData controllerData;
    static uint32_t lastPacketTime;
    static uint16_t errorCount;
    static uint8_t rxBuffer[BUFFER_SIZE];
    static uint8_t bufferIndex;
    
    /**
     * Parse incoming UART byte and update controller state
     */
    static void processByte(uint8_t byte);
    
    /**
     * Verify packet checksum
     */
    static bool verifyChecksum(const uint8_t* packet);
    
    /**
     * Extract controller data from valid packet
     */
    static void parsePacket(const uint8_t* packet);
};

#endif // XBOX_CONTROLLER_H
