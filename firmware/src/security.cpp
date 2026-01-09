/**
 * @file security.cpp
 * @brief Security implementation using ESP32-S3 crypto accelerator
 */

#include "security.h"
#include "config.h"
#include <Preferences.h>
#include <mbedtls/md.h>
#include <esp_random.h>
#include <algorithm>

// Default HMAC key for development (change for production!)
static const uint8_t DEFAULT_HMAC_KEY[32] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
    0xDE, 0xAD, 0xC0, 0xDE, 0xFE, 0xED, 0xFA, 0xCE,
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88
};

uint8_t Security::hmac_key[32] = {0};
bool Security::initialized = false;
std::vector<SeenMessage> Security::seen_messages;
std::map<String, uint64_t> Security::last_seen_timestamp;

bool Security::init() {
    if (initialized) {
        return true;
    }
    
    // Force default key for development (overwrites any existing key)
    LOG_I("Using default HMAC key for development");
    memcpy(hmac_key, DEFAULT_HMAC_KEY, 32);
    
    // Store to NVS to ensure persistence
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.putBytes(NVS_KEY_HMAC_KEY, hmac_key, 32);
        prefs.end();
        LOG_I("Default HMAC key stored to NVS");
    }
    
    initialized = true;
    LOG_I("Security system initialized");
    return true;
}

bool Security::setKey(const uint8_t* key, size_t key_len) {
    if (key_len != 32) {
        LOG_E("HMAC key must be 32 bytes");
        return false;
    }
    
    memcpy(hmac_key, key, 32);
    
    // Store in NVS
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        LOG_E("Failed to open NVS");
        return false;
    }
    
    prefs.putBytes(NVS_KEY_HMAC_KEY, hmac_key, 32);
    prefs.end();
    
    LOG_I("HMAC key updated and stored");
    return true;
}

bool Security::generateAndStoreKey() {
    // Generate random key using ESP32 hardware RNG
    for (int i = 0; i < 32; i++) {
        hmac_key[i] = esp_random() & 0xFF;
    }
    
    // Store in NVS
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        LOG_E("Failed to open NVS");
        return false;
    }
    
    prefs.putBytes(NVS_KEY_HMAC_KEY, hmac_key, 32);
    prefs.end();
    
    LOG_I("Generated and stored new HMAC key");
    
    // Print key for provisioning (remove in production!)
    LOG_W("HMAC Key (HEX): %02X%02X%02X%02X...", hmac_key[0], hmac_key[1], hmac_key[2], hmac_key[3]);
    
    return true;
}

bool Security::loadKeyFromNVS() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) { // Read-only
        return false;
    }
    
    size_t key_len = prefs.getBytesLength(NVS_KEY_HMAC_KEY);
    if (key_len != 32) {
        prefs.end();
        return false;
    }
    
    prefs.getBytes(NVS_KEY_HMAC_KEY, hmac_key, 32);
    prefs.end();
    
    LOG_I("Loaded HMAC key from NVS");
    return true;
}

bool Security::computeHMAC(const uint8_t* data, size_t data_len, uint8_t* signature) {
    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
    
    mbedtls_md_init(&ctx);
    
    int ret = mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1); // 1 = HMAC mode
    if (ret != 0) {
        LOG_E("mbedtls_md_setup failed: %d", ret);
        mbedtls_md_free(&ctx);
        return false;
    }
    
    ret = mbedtls_md_hmac_starts(&ctx, hmac_key, 32);
    if (ret != 0) {
        LOG_E("mbedtls_md_hmac_starts failed: %d", ret);
        mbedtls_md_free(&ctx);
        return false;
    }
    
    ret = mbedtls_md_hmac_update(&ctx, data, data_len);
    if (ret != 0) {
        LOG_E("mbedtls_md_hmac_update failed: %d", ret);
        mbedtls_md_free(&ctx);
        return false;
    }
    
    ret = mbedtls_md_hmac_finish(&ctx, signature);
    if (ret != 0) {
        LOG_E("mbedtls_md_hmac_finish failed: %d", ret);
        mbedtls_md_free(&ctx);
        return false;
    }
    
    mbedtls_md_free(&ctx);
    return true;
}

bool Security::signMessage(LoRaMessage& msg) {
    if (!initialized) {
        LOG_E("Security not initialized");
        return false;
    }
    
    // Serialize message (excluding signature field)
    uint8_t buffer[MAX_LORA_PAYLOAD_BYTES];
    size_t msg_len = ProtobufHandler::encodeLoRaMessage(msg, buffer, sizeof(buffer));
    
    if (msg_len == 0) {
        LOG_E("Failed to encode message for signing");
        return false;
    }
    
    // Compute HMAC
    uint8_t signature[32];
    if (!computeHMAC(buffer, msg_len, signature)) {
        LOG_E("Failed to compute HMAC");
        return false;
    }
    
    // Attach signature to message - signature_data is now PB_BYTES_ARRAY_T(64)
    msg.has_signature = true;
    ProtobufHandler::copyBytes(msg.signature.signature_data, signature, 32);
    
    LOG_D("Message signed: %s", msg.message_id);
    return true;
}

bool Security::verifyMessage(const LoRaMessage& msg) {
    if (!initialized) {
        LOG_E("Security not initialized");
        return false;
    }
    
    // signature_data is now PB_BYTES_ARRAY_T(64) - check size field
    if (!msg.has_signature || msg.signature.signature_data.size != 32) {
        LOG_E("Invalid signature size: %d", msg.signature.signature_data.size);
        return false;
    }
    
    // Create copy without signature for verification
    LoRaMessage msg_copy = msg;
    msg_copy.has_signature = false;
    memset(&msg_copy.signature, 0, sizeof(msg_copy.signature));
    
    // Serialize message
    uint8_t buffer[MAX_LORA_PAYLOAD_BYTES];
    size_t msg_len = ProtobufHandler::encodeLoRaMessage(msg_copy, buffer, sizeof(buffer));
    
    if (msg_len == 0) {
        LOG_E("Failed to encode message for verification");
        return false;
    }
    
    // Compute expected HMAC
    uint8_t expected_signature[32];
    if (!computeHMAC(buffer, msg_len, expected_signature)) {
        LOG_E("Failed to compute HMAC for verification");
        return false;
    }
    
    // Compare signatures (constant-time comparison)
    int match = 0;
    for (int i = 0; i < 32; i++) {
        match |= (expected_signature[i] ^ msg.signature.signature_data.bytes[i]);
    }
    
    if (match != 0) {
        LOG_W("Signature verification failed for message: %s", msg.message_id);
        return false;
    }
    
    // Check for duplicate/replay using message_id + timestamp
    String message_id = msg.message_id;
    if (!checkReplayProtection(message_id, msg.timestamp)) {
        LOG_W("Duplicate message detected: %s", msg.message_id);
        return false;
    }
    
    LOG_D("Message signature verified: %s", msg.message_id);
    return true;
}

bool Security::checkReplayProtection(const String& message_id, uint64_t timestamp) {
    uint64_t now = ProtobufHandler::getCurrentTimestamp();
    
    // Prune old messages periodically
    pruneOldMessages();
    
    // Check if we've seen this exact message_id before
    for (const auto& seen : seen_messages) {
        if (seen.message_id == message_id) {
            LOG_W("Duplicate message_id detected: %s", message_id.c_str());
            return false;  // Duplicate!
        }
    }
    
    // Reject messages claiming to be from far future (clock manipulation)
    if (timestamp > now + TIMESTAMP_TOLERANCE_MS) {
        LOG_W("Future timestamp rejected: %llu > %llu + %lu", timestamp, now, TIMESTAMP_TOLERANCE_MS);
        return false;
    }
    
    // Reject very old messages (likely stale/replay)
    // Allow messages up to MESSAGE_EXPIRY_MS old to handle delayed delivery
    if (now > timestamp && (now - timestamp) > MESSAGE_EXPIRY_MS) {
        LOG_W("Message too old: %llu ms ago", now - timestamp);
        return false;
    }
    
    // Track this message_id
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
    uint64_t now = ProtobufHandler::getCurrentTimestamp();
    
    // Remove messages older than expiry time
    seen_messages.erase(
        std::remove_if(seen_messages.begin(), seen_messages.end(),
            [now](const SeenMessage& msg) {
                return (now - msg.seen_at) > MESSAGE_EXPIRY_MS;
            }),
        seen_messages.end()
    );
}
