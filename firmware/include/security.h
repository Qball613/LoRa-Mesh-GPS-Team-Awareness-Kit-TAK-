/**
 * @file security.h
 * @brief HMAC-SHA256 message signing and verification
 */

#ifndef SECURITY_H
#define SECURITY_H

#include <Arduino.h>
#include <map>
#include <set>
#include <vector>
#include "protobuf_handler.h"

// Message ID tracking for replay protection
struct SeenMessage {
    String message_id;
    uint64_t seen_at;  // When we first saw this message
};

class Security {
public:
    /**
     * @brief Initialize security system
     * @return true if successful
     */
    static bool init();
    
    /**
     * @brief Set HMAC key (256-bit)
     * @param key 32-byte key
     * @return true if successful
     */
    static bool setKey(const uint8_t* key, size_t key_len);
    
    /**
     * @brief Generate random HMAC key and store in NVS
     * @return true if successful
     */
    static bool generateAndStoreKey();
    
    /**
     * @brief Load HMAC key from NVS
     * @return true if successful
     */
    static bool loadKeyFromNVS();
    
    /**
     * @brief Sign a LoRaMessage
     * @param msg Message to sign (signature field will be populated)
     * @return true if successful
     */
    static bool signMessage(LoRaMessage& msg);
    
    /**
     * @brief Verify a LoRaMessage signature
     * @param msg Message to verify
     * @return true if signature is valid
     */
    static bool verifyMessage(const LoRaMessage& msg);
    
    /**
     * @brief Compute HMAC-SHA256
     * @param data Input data
     * @param data_len Length of data
     * @param signature Output buffer (32 bytes)
     * @return true if successful
     */
    static bool computeHMAC(const uint8_t* data, size_t data_len, uint8_t* signature);
    
    /**
     * @brief Check if message is duplicate/replay
     * Uses message_id + timestamp for deduplication (sequence numbers not used)
     * @param message_id Unique message identifier
     * @param timestamp Message timestamp
     * @return true if message is valid (not duplicate)
     */
    static bool checkReplayProtection(const String& message_id, uint64_t timestamp);
    
    /**
     * @brief Prune old message IDs from tracking (call periodically)
     */
    static void pruneOldMessages();

private:
    static uint8_t hmac_key[32];
    static bool initialized;
    static std::vector<SeenMessage> seen_messages;  // Recently seen message IDs
    static std::map<String, uint64_t> last_seen_timestamp;  // Per-node last timestamp (optional)
    
    static constexpr size_t MAX_SEEN_MESSAGES = 100;  // Limit memory usage
    static constexpr uint64_t MESSAGE_EXPIRY_MS = 300000;  // 5 minutes
};

#endif // SECURITY_H
