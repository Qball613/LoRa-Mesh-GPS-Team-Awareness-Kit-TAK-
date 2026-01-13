/**
 * @file security.h
 * @brief AES-256-GCM encryption with authenticated encryption
 */

#ifndef SECURITY_H
#define SECURITY_H

#include <Arduino.h>
#include <map>
#include <set>
#include <vector>
#include "protobuf_handler.h"

// AES-256-GCM constants
static constexpr size_t AES_GCM_KEY_SIZE = 32;    // 256-bit key
static constexpr size_t AES_GCM_NONCE_SIZE = 12;  // 96-bit nonce (recommended for GCM)
static constexpr size_t AES_GCM_TAG_SIZE = 16;    // 128-bit auth tag

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
     * @brief Set encryption key (256-bit)
     * @param key 32-byte key
     * @return true if successful
     */
    static bool setKey(const uint8_t* key, size_t key_len);

    /**
     * @brief Generate random key and store in NVS
     * @return true if successful
     */
    static bool generateAndStoreKey();

    /**
     * @brief Load key from NVS
     * @return true if successful
     */
    static bool loadKeyFromNVS();

    /**
     * @brief Encrypt entire serialized message (full confidentiality)
     * @param plaintext Serialized LoRaMessage
     * @param plaintext_len Length of serialized message
     * @param ciphertext Output buffer [nonce][ciphertext][tag]
     * @param ciphertext_len Output length
     * @return true if successful
     *
     * Wire format: [12-byte nonce][encrypted protobuf][16-byte GCM tag]
     * GCM tag provides authentication - no separate signature needed
     */
    static bool encryptMessage(const uint8_t* plaintext, size_t plaintext_len,
                               uint8_t* ciphertext, size_t* ciphertext_len);

    /**
     * @brief Decrypt entire message (full confidentiality)
     * @param ciphertext Received encrypted blob [nonce][ciphertext][tag]
     * @param ciphertext_len Length of encrypted data
     * @param plaintext Output buffer for decrypted protobuf
     * @param plaintext_len Output length
     * @return true if successful and GCM authentication passed
     */
    static bool decryptMessage(const uint8_t* ciphertext, size_t ciphertext_len,
                               uint8_t* plaintext, size_t* plaintext_len);

    /**
     * @brief Check if message is duplicate/replay
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
    static uint8_t encryption_key[32];  // AES-256-GCM key
    static bool initialized;
    static std::vector<SeenMessage> seen_messages;  // Recently seen message IDs
    static std::map<String, uint64_t> last_seen_timestamp;  // Per-node last timestamp (optional)

    static constexpr size_t MAX_SEEN_MESSAGES = 100;  // Limit memory usage
    static constexpr uint64_t MESSAGE_EXPIRY_MS = 300000;  // 5 minutes
};

#endif // SECURITY_H
