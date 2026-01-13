/**
 * @file security.cpp
 * @brief AES-256-GCM encryption using ESP32-S3 crypto accelerator
 *
 * Provides full message encryption with authenticated encryption.
 * GCM mode provides both confidentiality and authentication in one operation.
 */

#include "security.h"
#include "config.h"
#include <Preferences.h>
#include <mbedtls/gcm.h>
#include <esp_random.h>
#include <algorithm>

// Default encryption key for development (change for production!)
static const uint8_t DEFAULT_KEY[32] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
    0xDE, 0xAD, 0xC0, 0xDE, 0xFE, 0xED, 0xFA, 0xCE,
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88
};

// NVS key name for encryption key
static const char* NVS_KEY_ENCRYPTION = "enc_key";

uint8_t Security::encryption_key[32] = {0};
bool Security::initialized = false;
std::vector<SeenMessage> Security::seen_messages;
std::map<String, uint64_t> Security::last_seen_timestamp;

bool Security::init() {
    if (initialized) {
        return true;
    }

    // Use default key for development
    LOG_I("Using default encryption key for development");
    memcpy(encryption_key, DEFAULT_KEY, 32);

    // Store to NVS to ensure persistence
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putBytes(NVS_KEY_ENCRYPTION, encryption_key, 32);
        prefs.end();
    }

    initialized = true;
    LOG_I("Security initialized: AES-256-GCM encryption enabled");
    LOG_I("Key (first 4): %02X%02X%02X%02X",
          encryption_key[0], encryption_key[1], encryption_key[2], encryption_key[3]);
    return true;
}

bool Security::setKey(const uint8_t* key, size_t key_len) {
    if (key_len != 32) {
        LOG_E("Encryption key must be 32 bytes");
        return false;
    }

    memcpy(encryption_key, key, 32);

    // Store in NVS
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        LOG_E("Failed to open NVS");
        return false;
    }

    prefs.putBytes(NVS_KEY_ENCRYPTION, encryption_key, 32);
    prefs.end();

    LOG_I("Encryption key updated and stored");
    return true;
}

bool Security::generateAndStoreKey() {
    // Generate random key using ESP32 hardware RNG
    for (int i = 0; i < 32; i++) {
        encryption_key[i] = esp_random() & 0xFF;
    }

    // Store in NVS
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        LOG_E("Failed to open NVS");
        return false;
    }

    prefs.putBytes(NVS_KEY_ENCRYPTION, encryption_key, 32);
    prefs.end();

    LOG_I("Generated and stored new encryption key");
    LOG_W("Key (first 4 bytes): %02X%02X%02X%02X...",
          encryption_key[0], encryption_key[1], encryption_key[2], encryption_key[3]);

    return true;
}

bool Security::loadKeyFromNVS() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        return false;
    }

    size_t key_len = prefs.getBytesLength(NVS_KEY_ENCRYPTION);
    if (key_len != 32) {
        prefs.end();
        return false;
    }

    prefs.getBytes(NVS_KEY_ENCRYPTION, encryption_key, 32);
    prefs.end();

    LOG_I("Loaded encryption key from NVS");
    return true;
}

// ==================== Replay Protection ====================

bool Security::checkReplayProtection(const String& message_id, uint64_t timestamp) {
    uint64_t now = millis();  // Use local time only - clocks are not synchronized!

    // Prune old messages periodically
    pruneOldMessages();

    // Check if we've seen this exact message_id before
    for (const auto& seen : seen_messages) {
        if (seen.message_id == message_id) {
            LOG_D("Duplicate message_id: %s (seen %lu ms ago)",
                  message_id.c_str(), (unsigned long)(now - seen.seen_at));
            return false;  // Duplicate!
        }
    }

    // NOTE: We do NOT check the sender's timestamp because:
    // - ESP32 nodes have no RTC and use uptime as "time"
    // - Nodes restart at different times, so clocks are never synchronized
    // - A node that's been running 10 min will have timestamp ~600000
    // - A node that just restarted will have timestamp ~5000
    // - Comparing these is meaningless and causes false rejections
    //
    // Instead, we rely purely on message_id uniqueness:
    // - message_id is 64-bit random, collision probability is negligible
    // - We track seen message_ids for MESSAGE_EXPIRY_MS (5 minutes)
    // - This prevents replay attacks within that window

    // Track this message_id using OUR local time
    SeenMessage entry;
    entry.message_id = message_id;
    entry.seen_at = now;
    seen_messages.push_back(entry);

    // Limit memory usage
    if (seen_messages.size() > MAX_SEEN_MESSAGES) {
        seen_messages.erase(seen_messages.begin());
    }

    return true;
}

void Security::pruneOldMessages() {
    uint64_t now = millis();  // Use local time

    // Remove messages we saw more than MESSAGE_EXPIRY_MS ago
    seen_messages.erase(
        std::remove_if(seen_messages.begin(), seen_messages.end(),
            [now](const SeenMessage& msg) {
                return (now - msg.seen_at) > MESSAGE_EXPIRY_MS;
            }),
        seen_messages.end()
    );
}

// ==================== AES-256-GCM Encryption ====================

/**
 * @brief Internal GCM encrypt function
 */
static bool gcm_encrypt(const uint8_t* key,
                        const uint8_t* plaintext, size_t plaintext_len,
                        uint8_t* ciphertext, size_t* ciphertext_len) {

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    int ret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, 256);
    if (ret != 0) {
        LOG_E("mbedtls_gcm_setkey failed: %d", ret);
        mbedtls_gcm_free(&gcm);
        return false;
    }

    // Generate random 12-byte nonce using ESP32 hardware RNG
    uint8_t nonce[AES_GCM_NONCE_SIZE];
    for (size_t i = 0; i < AES_GCM_NONCE_SIZE; i++) {
        nonce[i] = esp_random() & 0xFF;
    }

    // Output format: [nonce (12)][ciphertext][tag (16)]
    memcpy(ciphertext, nonce, AES_GCM_NONCE_SIZE);

    uint8_t* encrypted_data = ciphertext + AES_GCM_NONCE_SIZE;
    uint8_t* tag = encrypted_data + plaintext_len;

    ret = mbedtls_gcm_crypt_and_tag(&gcm,
                                     MBEDTLS_GCM_ENCRYPT,
                                     plaintext_len,
                                     nonce, AES_GCM_NONCE_SIZE,
                                     nullptr, 0,  // No AAD - full encryption
                                     plaintext,
                                     encrypted_data,
                                     AES_GCM_TAG_SIZE, tag);

    mbedtls_gcm_free(&gcm);

    if (ret != 0) {
        LOG_E("mbedtls_gcm_crypt_and_tag failed: %d", ret);
        return false;
    }

    *ciphertext_len = AES_GCM_NONCE_SIZE + plaintext_len + AES_GCM_TAG_SIZE;
    return true;
}

/**
 * @brief Internal GCM decrypt function
 */
static bool gcm_decrypt(const uint8_t* key,
                        const uint8_t* ciphertext, size_t ciphertext_len,
                        uint8_t* plaintext, size_t* plaintext_len) {

    // Minimum size: nonce + tag
    if (ciphertext_len < AES_GCM_NONCE_SIZE + AES_GCM_TAG_SIZE) {
        LOG_E("Ciphertext too short: %u", ciphertext_len);
        return false;
    }

    mbedtls_gcm_context gcm;
    mbedtls_gcm_init(&gcm);

    int ret = mbedtls_gcm_setkey(&gcm, MBEDTLS_CIPHER_ID_AES, key, 256);
    if (ret != 0) {
        LOG_E("mbedtls_gcm_setkey failed: %d", ret);
        mbedtls_gcm_free(&gcm);
        return false;
    }

    // Parse: [nonce (12)][encrypted data][tag (16)]
    const uint8_t* nonce = ciphertext;
    size_t encrypted_len = ciphertext_len - AES_GCM_NONCE_SIZE - AES_GCM_TAG_SIZE;
    const uint8_t* encrypted_data = ciphertext + AES_GCM_NONCE_SIZE;
    const uint8_t* tag = encrypted_data + encrypted_len;

    ret = mbedtls_gcm_auth_decrypt(&gcm,
                                    encrypted_len,
                                    nonce, AES_GCM_NONCE_SIZE,
                                    nullptr, 0,  // No AAD
                                    tag, AES_GCM_TAG_SIZE,
                                    encrypted_data,
                                    plaintext);

    mbedtls_gcm_free(&gcm);

    if (ret != 0) {
        if (ret == MBEDTLS_ERR_GCM_AUTH_FAILED) {
            LOG_W("GCM auth failed - likely foreign device or different key");
            LOG_W("  Nonce: %02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                  nonce[0], nonce[1], nonce[2], nonce[3], nonce[4], nonce[5],
                  nonce[6], nonce[7], nonce[8], nonce[9], nonce[10], nonce[11]);
            LOG_W("  Packet size=%u (probably from another LoRa network)", ciphertext_len);
        } else {
            LOG_E("mbedtls_gcm_auth_decrypt failed: %d", ret);
        }
        return false;
    }

    *plaintext_len = encrypted_len;
    return true;
}

// ==================== Public API ====================

bool Security::encryptMessage(const uint8_t* plaintext, size_t plaintext_len,
                               uint8_t* ciphertext, size_t* ciphertext_len) {
    if (!initialized) {
        LOG_E("Security not initialized");
        return false;
    }

    bool result = gcm_encrypt(encryption_key, plaintext, plaintext_len,
                              ciphertext, ciphertext_len);

    if (result) {
        LOG_D("Encrypted %u -> %u bytes", plaintext_len, *ciphertext_len);
    }
    return result;
}

bool Security::decryptMessage(const uint8_t* ciphertext, size_t ciphertext_len,
                               uint8_t* plaintext, size_t* plaintext_len) {
    if (!initialized) {
        LOG_E("Security not initialized");
        return false;
    }

    bool result = gcm_decrypt(encryption_key, ciphertext, ciphertext_len,
                              plaintext, plaintext_len);

    if (result) {
        LOG_D("Decrypted %u -> %u bytes", ciphertext_len, *plaintext_len);
    }
    return result;
}
