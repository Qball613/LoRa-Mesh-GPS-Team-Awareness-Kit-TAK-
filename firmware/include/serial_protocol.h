#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include <functional>

// Forward declarations
class RoutingEngine;
class GPSManager;

/**
 * SLIP (Serial Line Internet Protocol) Framing
 * RFC 1055 - used by Meshtastic and other embedded protocols
 * 
 * Frame format: [END][data with escaping][END]
 * - 0xC0 (END) marks packet boundaries
 * - 0xDB (ESC) followed by 0xDC escapes 0xC0 in data
 * - 0xDB (ESC) followed by 0xDD escapes 0xDB in data
 */

namespace slip {
    constexpr uint8_t END = 0xC0;
    constexpr uint8_t ESC = 0xDB;
    constexpr uint8_t ESC_END = 0xDC;
    constexpr uint8_t ESC_ESC = 0xDD;
    
    constexpr size_t MAX_PACKET_SIZE = 512;  // Max decoded packet size
}

/**
 * SLIP decoder state machine
 */
class SLIPDecoder {
public:
    SLIPDecoder();
    
    /**
     * Feed bytes to the decoder
     * @param byte The byte to process
     * @return true if a complete packet is ready
     */
    bool feed(uint8_t byte);
    
    /**
     * Get the decoded packet data
     * @return Pointer to packet data
     */
    const uint8_t* getPacket() const { return buffer_; }
    
    /**
     * Get the decoded packet length
     * @return Packet length in bytes
     */
    size_t getLength() const { return length_; }
    
    /**
     * Reset the decoder for next packet
     */
    void reset();

private:
    uint8_t buffer_[slip::MAX_PACKET_SIZE];
    size_t length_;
    bool escaping_;
    bool started_;
};

/**
 * SLIP encoder
 */
class SLIPEncoder {
public:
    /**
     * Encode data with SLIP framing
     * @param data Input data
     * @param len Input length
     * @param output Output buffer (must be at least len*2 + 2)
     * @return Number of bytes written to output
     */
    static size_t encode(const uint8_t* data, size_t len, uint8_t* output);
    
    /**
     * Calculate maximum encoded size
     */
    static size_t maxEncodedSize(size_t len) { return len * 2 + 2; }
};

/**
 * Serial Protocol Handler
 * Handles protobuf messages over SLIP-framed serial
 */
class SerialProtocol {
public:
    SerialProtocol(Stream& serial);
    
    /**
     * Set routing engine reference for mesh operations
     */
    void setRoutingEngine(RoutingEngine* engine) { routing_ = engine; }
    
    /**
     * Set GPS manager reference
     */
    void setGPSManager(GPSManager* gps) { gps_ = gps; }
    
    /**
     * Process incoming serial data (call in loop)
     */
    void update();
    
    /**
     * Enable/disable async event notifications
     */
    void setEventsEnabled(bool enabled) { events_enabled_ = enabled; }
    
    // Event notification methods (call from other modules)
    void notifyMessageReceived(const char* from, const char* msg_id, 
                               const char* text, bool is_broadcast);
    void notifyGPSReceived(const char* node_id, double lat, double lon, 
                           double alt, uint64_t timestamp);
    void notifyNeighborJoined(const char* node_id, int rssi);
    void notifyNeighborLeft(const char* node_id);
    void notifyEmergency(const char* from, uint8_t type, const char* desc,
                         double lat, double lon);
    void logMessage(uint8_t level, const char* message);

private:
    Stream& serial_;
    SLIPDecoder decoder_;
    RoutingEngine* routing_;
    GPSManager* gps_;
    bool events_enabled_;
    uint32_t next_packet_id_;
    
    // Process a complete decoded packet
    void processPacket(const uint8_t* data, size_t len);
    
    // Send a SLIP-encoded packet
    void sendPacket(const uint8_t* data, size_t len);
    
    // Handle ToDevice commands
    void handleGetInfo(uint32_t request_id);
    void handleGetGPS(uint32_t request_id);
    void handleGetNeighbors(uint32_t request_id);
    void handleGetRoutes(uint32_t request_id);
    void handleGetRoster(uint32_t request_id);
    void handleGetStats(uint32_t request_id);
    void handleSetGPS(uint32_t request_id, const uint8_t* data, size_t len);
    void handleSendMessage(uint32_t request_id, const uint8_t* data, size_t len);
    void handleSendGPS(uint32_t request_id);
    void handleSendEmergency(uint32_t request_id, const uint8_t* data, size_t len);
    void handleDiscover(uint32_t request_id);
    void handleJoin(uint32_t request_id);
    void handlePing(uint32_t request_id, const uint8_t* data, size_t len);
    
    // Send response helpers
    void sendResult(uint32_t request_id, bool success, const char* error = nullptr);
};

#endif // SERIAL_PROTOCOL_H
