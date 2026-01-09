/**
 * @file protobuf_handler.h
 * @brief Protocol Buffer encoding/decoding using nanopb generated types
 * 
 * Uses STATIC nanopb types directly - no callback indirection.
 * Fields are char arrays and PB_BYTES_ARRAY_T, not pb_callback_t.
 */

#ifndef PROTOBUF_HANDLER_H
#define PROTOBUF_HANDLER_H

#include <Arduino.h>
#include <vector>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

// Include generated nanopb protocol buffer headers
#include "v1/common.pb.h"
#include "v1/messages.pb.h"
#include "v1/routing.pb.h"

// ================================================================
// Direct type aliases to nanopb types - NO WRAPPERS
// ================================================================

typedef lora_mesh_v1_GPSCoordinate GPSCoordinate;
typedef lora_mesh_v1_NodeInfo NodeInfo;
typedef lora_mesh_v1_HelloPayload HelloPayload;
typedef lora_mesh_v1_TextMessagePayload TextMessagePayload;
typedef lora_mesh_v1_GPSUpdatePayload GPSUpdatePayload;
typedef lora_mesh_v1_RouteRequestPayload RouteRequestPayload;
typedef lora_mesh_v1_RouteReplyPayload RouteReplyPayload;
typedef lora_mesh_v1_RouteErrorPayload RouteErrorPayload;
typedef lora_mesh_v1_ControlMessagePayload ControlMessagePayload;
typedef lora_mesh_v1_EmergencyPayload EmergencyPayload;
typedef lora_mesh_v1_AckPayload AckPayload;
typedef lora_mesh_v1_LinkQuality LinkQuality;
typedef lora_mesh_v1_Velocity Velocity;
typedef lora_mesh_v1_CryptographicSignature CryptographicSignature;

// LoRaMessage IS the actual nanopb type - no wrapper
typedef lora_mesh_v1_LoRaMessage LoRaMessage;

// Message type enum matching nanopb exactly
typedef lora_mesh_v1_MessageType MessageType;
#define MESSAGE_TYPE_UNKNOWN lora_mesh_v1_MessageType_MESSAGE_TYPE_UNSPECIFIED
#define MESSAGE_TYPE_GPS_UPDATE lora_mesh_v1_MessageType_MESSAGE_TYPE_GPS_UPDATE
#define MESSAGE_TYPE_TEXT_MESSAGE lora_mesh_v1_MessageType_MESSAGE_TYPE_TEXT_MESSAGE
#define MESSAGE_TYPE_HELLO lora_mesh_v1_MessageType_MESSAGE_TYPE_HELLO
#define MESSAGE_TYPE_CONTROL lora_mesh_v1_MessageType_MESSAGE_TYPE_CONTROL_MESSAGE
#define MESSAGE_TYPE_ACK lora_mesh_v1_MessageType_MESSAGE_TYPE_ACKNOWLEDGMENT
#define MESSAGE_TYPE_EMERGENCY lora_mesh_v1_MessageType_MESSAGE_TYPE_EMERGENCY
#define MESSAGE_TYPE_NETWORK_DISCOVERY lora_mesh_v1_MessageType_MESSAGE_TYPE_NETWORK_DISCOVERY
#define MESSAGE_TYPE_ROUTE_REQUEST lora_mesh_v1_MessageType_MESSAGE_TYPE_ROUTE_REQUEST
#define MESSAGE_TYPE_ROUTE_REPLY lora_mesh_v1_MessageType_MESSAGE_TYPE_ROUTE_REPLY
#define MESSAGE_TYPE_ROUTE_ERROR lora_mesh_v1_MessageType_MESSAGE_TYPE_ROUTE_ERROR
#define MESSAGE_TYPE_LINK_STATE_ADVERTISEMENT lora_mesh_v1_MessageType_MESSAGE_TYPE_LINK_STATE_ADVERTISEMENT
#define MESSAGE_TYPE_DATA lora_mesh_v1_MessageType_MESSAGE_TYPE_UNSPECIFIED

// Priority enum matching nanopb
typedef lora_mesh_v1_MessagePriority MessagePriority;
#define PRIORITY_ROUTINE lora_mesh_v1_MessagePriority_MESSAGE_PRIORITY_NORMAL
#define PRIORITY_TACTICAL lora_mesh_v1_MessagePriority_MESSAGE_PRIORITY_HIGH
#define PRIORITY_EMERGENCY lora_mesh_v1_MessagePriority_MESSAGE_PRIORITY_CRITICAL

// ================================================================
// ProtobufHandler class - helpers for static nanopb types
// ================================================================

class ProtobufHandler {
public:
    // ================================================================
    // String/Bytes Copy Helpers for static arrays
    // ================================================================
    
    /**
     * @brief Copy Arduino String to fixed char array
     */
    static void copyString(char* dest, size_t destSize, const String& src);
    
    /**
     * @brief Copy C string to fixed char array
     */
    static void copyString(char* dest, size_t destSize, const char* src);
    
    /**
     * @brief Copy fixed char array to Arduino String
     */
    static String toString(const char* src);
    
    /**
     * @brief Copy data to PB_BYTES_ARRAY_T
     */
    template<typename T>
    static void copyBytes(T& dest, const uint8_t* src, size_t len) {
        size_t maxSize = sizeof(dest.bytes);
        dest.size = (len < maxSize) ? len : maxSize;
        memcpy(dest.bytes, src, dest.size);
    }
    
    /**
     * @brief Copy vector to PB_BYTES_ARRAY_T
     */
    template<typename T>
    static void copyBytes(T& dest, const std::vector<uint8_t>& src) {
        copyBytes(dest, src.data(), src.size());
    }
    
    /**
     * @brief Copy PB_BYTES_ARRAY_T to vector
     */
    template<typename T>
    static std::vector<uint8_t> toVector(const T& src) {
        return std::vector<uint8_t>(src.bytes, src.bytes + src.size);
    }

    // ================================================================
    // Message Builders - Create properly initialized nanopb structs
    // ================================================================
    
    static LoRaMessage createLoRaMessage();
    static HelloPayload createHelloPayload();
    static TextMessagePayload createTextMessagePayload();
    static RouteRequestPayload createRouteRequestPayload();
    static RouteReplyPayload createRouteReplyPayload();
    static RouteErrorPayload createRouteErrorPayload();
    static GPSUpdatePayload createGPSUpdatePayload();
    
    // ================================================================
    // LoRaMessage Encoding/Decoding
    // ================================================================
    
    static size_t encodeLoRaMessage(const LoRaMessage& msg, uint8_t* buffer, size_t buffer_size);
    static bool decodeLoRaMessage(const uint8_t* buffer, size_t buffer_size, LoRaMessage& msg);
    
    // ================================================================
    // Payload Encoding/Decoding
    // ================================================================
    
    static size_t encodePayload(const HelloPayload& payload, uint8_t* buffer, size_t buffer_size);
    static bool decodePayload(const uint8_t* buffer, size_t buffer_size, HelloPayload& payload);
    
    static size_t encodePayload(const TextMessagePayload& payload, uint8_t* buffer, size_t buffer_size);
    static bool decodePayload(const uint8_t* buffer, size_t buffer_size, TextMessagePayload& payload);
    
    static size_t encodePayload(const GPSUpdatePayload& payload, uint8_t* buffer, size_t buffer_size);
    static bool decodePayload(const uint8_t* buffer, size_t buffer_size, GPSUpdatePayload& payload);
    
    static size_t encodePayload(const RouteRequestPayload& payload, uint8_t* buffer, size_t buffer_size);
    static bool decodePayload(const uint8_t* buffer, size_t buffer_size, RouteRequestPayload& payload);
    
    static size_t encodePayload(const RouteReplyPayload& payload, uint8_t* buffer, size_t buffer_size);
    static bool decodePayload(const uint8_t* buffer, size_t buffer_size, RouteReplyPayload& payload);
    
    static size_t encodePayload(const RouteErrorPayload& payload, uint8_t* buffer, size_t buffer_size);
    static bool decodePayload(const uint8_t* buffer, size_t buffer_size, RouteErrorPayload& payload);
    
    // ================================================================
    // Utility Functions
    // ================================================================
    
    static String generateMessageID();
    static uint64_t getCurrentTimestamp();
    
    // GPSCoordinate helpers
    static size_t encodeGPSCoordinate(const GPSCoordinate& coord, uint8_t* buffer, size_t buffer_size);
    static bool decodeGPSCoordinate(const uint8_t* buffer, size_t buffer_size, GPSCoordinate& coord);
};

#endif // PROTOBUF_HANDLER_H
