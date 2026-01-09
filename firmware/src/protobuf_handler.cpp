/**
 * @file protobuf_handler.cpp
 * @brief Protocol Buffer encoding/decoding using STATIC nanopb types
 * 
 * No callback indirection - direct static array access.
 */

#include "protobuf_handler.h"
#include "config.h"
#include <esp_random.h>
#include <sys/time.h>
#include <cstring>

// ================================================================
// String/Bytes Copy Helpers
// ================================================================

void ProtobufHandler::copyString(char* dest, size_t destSize, const String& src) {
    size_t len = src.length();
    if (len >= destSize) len = destSize - 1;
    memcpy(dest, src.c_str(), len);
    dest[len] = '\0';
}

void ProtobufHandler::copyString(char* dest, size_t destSize, const char* src) {
    if (src == nullptr) {
        dest[0] = '\0';
        return;
    }
    size_t len = strlen(src);
    if (len >= destSize) len = destSize - 1;
    memcpy(dest, src, len);
    dest[len] = '\0';
}

String ProtobufHandler::toString(const char* src) {
    return String(src);
}

// ================================================================
// Utility Functions
// ================================================================

String ProtobufHandler::generateMessageID() {
    uint8_t random_bytes[8];
    for (int i = 0; i < 8; i++) {
        random_bytes[i] = esp_random() & 0xFF;
    }
    
    char id[32];
    snprintf(id, sizeof(id), "%02X%02X%02X%02X%02X%02X%02X%02X",
             random_bytes[0], random_bytes[1], random_bytes[2], random_bytes[3],
             random_bytes[4], random_bytes[5], random_bytes[6], random_bytes[7]);
    
    return String(id);
}

uint64_t ProtobufHandler::getCurrentTimestamp() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)(tv.tv_sec) * 1000 + (tv.tv_usec / 1000);
}

// ================================================================
// Message Builders - Create properly initialized nanopb structs
// ================================================================

LoRaMessage ProtobufHandler::createLoRaMessage() {
    LoRaMessage msg = lora_mesh_v1_LoRaMessage_init_zero;
    msg.timestamp = getCurrentTimestamp();
    msg.sequence_number = 0;  // Not used for replay protection anymore
    msg.ttl = 10;
    return msg;
}

HelloPayload ProtobufHandler::createHelloPayload() {
    HelloPayload hello = lora_mesh_v1_HelloPayload_init_zero;
    hello.has_node_info = true;
    hello.hello_timestamp = getCurrentTimestamp();
    hello.hello_sequence = 0;  // Not used for replay protection anymore
    return hello;
}

TextMessagePayload ProtobufHandler::createTextMessagePayload() {
    TextMessagePayload txt = lora_mesh_v1_TextMessagePayload_init_zero;
    txt.message_sequence = 0;  // Not used for replay protection anymore
    return txt;
}

RouteRequestPayload ProtobufHandler::createRouteRequestPayload() {
    RouteRequestPayload rreq = lora_mesh_v1_RouteRequestPayload_init_zero;
    rreq.rreq_id = esp_random();
    rreq.broadcast_id = esp_random();
    return rreq;
}

RouteReplyPayload ProtobufHandler::createRouteReplyPayload() {
    RouteReplyPayload rrep = lora_mesh_v1_RouteReplyPayload_init_zero;
    return rrep;
}

RouteErrorPayload ProtobufHandler::createRouteErrorPayload() {
    RouteErrorPayload rerr = lora_mesh_v1_RouteErrorPayload_init_zero;
    rerr.error_timestamp = getCurrentTimestamp();
    return rerr;
}

GPSUpdatePayload ProtobufHandler::createGPSUpdatePayload() {
    GPSUpdatePayload gps = lora_mesh_v1_GPSUpdatePayload_init_zero;
    gps.has_position = true;
    gps.timestamp = getCurrentTimestamp();
    return gps;
}

// ================================================================
// LoRaMessage Encoding/Decoding - DIRECT STATIC TYPES
// ================================================================

size_t ProtobufHandler::encodeLoRaMessage(const LoRaMessage& msg, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    
    if (!pb_encode(&stream, lora_mesh_v1_LoRaMessage_fields, &msg)) {
        LOG_E("LoRaMessage encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    
    LOG_D("Encoded LoRaMessage: type=%d, src=%s, size=%d",
          msg.message_type, msg.source_id, stream.bytes_written);
    
    return stream.bytes_written;
}

bool ProtobufHandler::decodeLoRaMessage(const uint8_t* buffer, size_t buffer_size, LoRaMessage& msg) {
    // Initialize to zero before decode
    memset(&msg, 0, sizeof(msg));
    
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    
    if (!pb_decode(&stream, lora_mesh_v1_LoRaMessage_fields, &msg)) {
        LOG_E("LoRaMessage decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    
    LOG_D("Decoded LoRaMessage: type=%d, src=%s, payload=%d bytes",
          msg.message_type, msg.source_id, msg.payload.size);
    
    return true;
}

// ================================================================
// HelloPayload Encoding/Decoding
// ================================================================

size_t ProtobufHandler::encodePayload(const HelloPayload& payload, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, lora_mesh_v1_HelloPayload_fields, &payload)) {
        LOG_E("HelloPayload encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool ProtobufHandler::decodePayload(const uint8_t* buffer, size_t buffer_size, HelloPayload& payload) {
    memset(&payload, 0, sizeof(payload));
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    if (!pb_decode(&stream, lora_mesh_v1_HelloPayload_fields, &payload)) {
        LOG_E("HelloPayload decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    return true;
}

// ================================================================
// TextMessagePayload Encoding/Decoding
// ================================================================

size_t ProtobufHandler::encodePayload(const TextMessagePayload& payload, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, lora_mesh_v1_TextMessagePayload_fields, &payload)) {
        LOG_E("TextMessagePayload encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool ProtobufHandler::decodePayload(const uint8_t* buffer, size_t buffer_size, TextMessagePayload& payload) {
    memset(&payload, 0, sizeof(payload));
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    if (!pb_decode(&stream, lora_mesh_v1_TextMessagePayload_fields, &payload)) {
        LOG_E("TextMessagePayload decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    return true;
}

// ================================================================
// GPSUpdatePayload Encoding/Decoding
// ================================================================

size_t ProtobufHandler::encodePayload(const GPSUpdatePayload& payload, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, lora_mesh_v1_GPSUpdatePayload_fields, &payload)) {
        LOG_E("GPSUpdatePayload encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool ProtobufHandler::decodePayload(const uint8_t* buffer, size_t buffer_size, GPSUpdatePayload& payload) {
    memset(&payload, 0, sizeof(payload));
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    if (!pb_decode(&stream, lora_mesh_v1_GPSUpdatePayload_fields, &payload)) {
        LOG_E("GPSUpdatePayload decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    return true;
}

// ================================================================
// RouteRequestPayload Encoding/Decoding
// ================================================================

size_t ProtobufHandler::encodePayload(const RouteRequestPayload& payload, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, lora_mesh_v1_RouteRequestPayload_fields, &payload)) {
        LOG_E("RouteRequestPayload encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool ProtobufHandler::decodePayload(const uint8_t* buffer, size_t buffer_size, RouteRequestPayload& payload) {
    memset(&payload, 0, sizeof(payload));
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    if (!pb_decode(&stream, lora_mesh_v1_RouteRequestPayload_fields, &payload)) {
        LOG_E("RouteRequestPayload decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    return true;
}

// ================================================================
// RouteReplyPayload Encoding/Decoding
// ================================================================

size_t ProtobufHandler::encodePayload(const RouteReplyPayload& payload, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, lora_mesh_v1_RouteReplyPayload_fields, &payload)) {
        LOG_E("RouteReplyPayload encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool ProtobufHandler::decodePayload(const uint8_t* buffer, size_t buffer_size, RouteReplyPayload& payload) {
    memset(&payload, 0, sizeof(payload));
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    if (!pb_decode(&stream, lora_mesh_v1_RouteReplyPayload_fields, &payload)) {
        LOG_E("RouteReplyPayload decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    return true;
}

// ================================================================
// RouteErrorPayload Encoding/Decoding
// ================================================================

size_t ProtobufHandler::encodePayload(const RouteErrorPayload& payload, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, lora_mesh_v1_RouteErrorPayload_fields, &payload)) {
        LOG_E("RouteErrorPayload encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool ProtobufHandler::decodePayload(const uint8_t* buffer, size_t buffer_size, RouteErrorPayload& payload) {
    memset(&payload, 0, sizeof(payload));
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    if (!pb_decode(&stream, lora_mesh_v1_RouteErrorPayload_fields, &payload)) {
        LOG_E("RouteErrorPayload decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    return true;
}

// ================================================================
// GPSCoordinate Encoding/Decoding
// ================================================================

size_t ProtobufHandler::encodeGPSCoordinate(const GPSCoordinate& coord, uint8_t* buffer, size_t buffer_size) {
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
    if (!pb_encode(&stream, lora_mesh_v1_GPSCoordinate_fields, &coord)) {
        LOG_E("GPSCoordinate encode failed: %s", PB_GET_ERROR(&stream));
        return 0;
    }
    return stream.bytes_written;
}

bool ProtobufHandler::decodeGPSCoordinate(const uint8_t* buffer, size_t buffer_size, GPSCoordinate& coord) {
    memset(&coord, 0, sizeof(coord));
    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_size);
    if (!pb_decode(&stream, lora_mesh_v1_GPSCoordinate_fields, &coord)) {
        LOG_E("GPSCoordinate decode failed: %s", PB_GET_ERROR(&stream));
        return false;
    }
    return true;
}
