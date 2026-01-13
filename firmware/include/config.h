/**
 * @file config.h
 * @brief Global configuration and hardware abstraction for LoRa Mesh GPS TAK
 *
 * This file provides compile-time configuration based on build flags
 * and defines hardware capabilities for different board variants.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ================================================================
// Device Type Detection
// ================================================================
#if defined(DEVICE_TYPE_T3S3)
    #define DEVICE_NAME "T3S3"
#elif defined(DEVICE_TYPE_TDECK)
    #define DEVICE_NAME "T-Deck"
#else
    #define DEVICE_NAME "Unknown"
    #error "No device type defined! Set DEVICE_TYPE_T3S3 or DEVICE_TYPE_TDECK"
#endif

// ================================================================
// Capability Flags (set via platformio.ini build_flags)
// ================================================================
#ifndef HAS_LORA
    #define HAS_LORA 0
#endif

#ifndef HAS_BLUETOOTH
    #define HAS_BLUETOOTH 0
#endif

#ifndef HAS_TFT
    #define HAS_TFT 0
#endif

#ifndef HAS_KEYBOARD
    #define HAS_KEYBOARD 0
#endif

#ifndef HAS_GPS
    #define HAS_GPS 0
#endif

#ifndef STANDALONE_MODE
    #define STANDALONE_MODE 0
#endif

#ifndef ENABLE_BLUETOOTH_GATEWAY
    #define ENABLE_BLUETOOTH_GATEWAY 0
#endif

// ================================================================
// LoRa Radio Configuration
// ================================================================
#define LORA_FREQUENCY_MHZ          915.0    // US frequency band
#define LORA_BANDWIDTH_KHZ          125.0
#define LORA_SPREADING_FACTOR       7        // SF7 = 222 byte payload
#define LORA_CODING_RATE            5        // 4/5
#define LORA_TX_POWER_DBM           20       // 20dBm = 100mW
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYNC_WORD              0x12     // Private network

// Payload limits by spreading factor (SF7 = 222 bytes max)
#define MAX_LORA_PAYLOAD_BYTES      222
#define LORA_HEADER_OVERHEAD        13       // RadioLib overhead

// ================================================================
// Routing Protocol Configuration
// ================================================================
#ifndef MAX_ROUTING_TABLE_SIZE
    #define MAX_ROUTING_TABLE_SIZE  100      // Max destinations to track
#endif

#ifndef MAX_NEIGHBOR_TABLE_SIZE
    #define MAX_NEIGHBOR_TABLE_SIZE 50       // Max direct neighbors
#endif

#define MAX_DUPLICATE_CACHE_SIZE    32       // Message ID cache for duplicate detection
#define MAX_MESSAGE_QUEUE_SIZE      20       // Outgoing message queue
#define MAX_PENDING_RREQ_SIZE       10       // Pending route requests

// Multi-path routing
#ifndef MAX_ALTERNATE_ROUTES
    #define MAX_ALTERNATE_ROUTES    3        // Backup routes per destination
#endif

#define MULTIPATH_RREQ_STAGGER_MS   50       // Delay between multi-path RREQs

// Neighbor blacklisting
#define NEIGHBOR_FAILURE_THRESHOLD  3        // Failures before blacklist
#define NEIGHBOR_BLACKLIST_DURATION_MS  300000  // 5 minutes
#define NEIGHBOR_BLACKLIST_MAX_DURATION_MS 900000 // 15 minutes

// AODV timings
#define ROUTE_LIFETIME_MS           1200000  // 20 minutes (match neighbor timeout)
#define RREQ_TIMEOUT_MS             3000     // 3 seconds
#define RREP_TIMEOUT_MS             5000     // 5 seconds
#define MAX_RREQ_RETRIES            3
#define RREQ_BACKOFF_MIN_MS         10
#define RREQ_BACKOFF_MAX_MS         50

// Beacon and maintenance
#ifndef HELLO_BEACON_INTERVAL_MS
    #define HELLO_BEACON_INTERVAL_MS 900000  // 15 minutes (fallback only, any message refreshes neighbors)
#endif

#define NEIGHBOR_TIMEOUT_MS         1200000  // 20 minutes (> beacon interval, any RX refreshes)
#define ROUTE_CLEANUP_INTERVAL_MS   10000    // Clean expired routes

// Message parameters
#define DEFAULT_TTL                 10       // Max hops
#define MAX_HOP_COUNT               15
#define MESSAGE_ID_LENGTH           16       // Bytes

// ================================================================
// GPS Configuration
// ================================================================
#ifndef GPS_UPDATE_INTERVAL_MS
    #define GPS_UPDATE_INTERVAL_MS  60000    // 60 seconds
#endif

#define GPS_BAUD_RATE               9600
#define GPS_TIMEOUT_MS              1000
#define GPS_MIN_SATELLITES          3        // Minimum for valid fix

// GPS source modes
typedef enum {
    GPS_SOURCE_HARDWARE = 0,    // Physical GPS module
    GPS_SOURCE_STATIC = 1,      // Fixed/static coordinates
    GPS_SOURCE_MANUAL = 2       // Set via serial command
} GPSSourceMode;

// ================================================================
// Security Configuration
// ================================================================
#define HMAC_KEY_SIZE_BYTES         32       // 256-bit key
#define HMAC_SIGNATURE_SIZE_BYTES   32       // SHA-256 output
#define NVS_NAMESPACE               "lora_tak"
#define NVS_KEY_HMAC_KEY            "hmac_key"
#define NVS_KEY_NODE_ID             "node_id"
#define NVS_KEY_STATIC_GPS          "static_gps"

// Replay protection
#define MAX_SEQUENCE_NUMBER         UINT32_MAX
#define TIMESTAMP_TOLERANCE_MS      300000   // 5 minute clock drift tolerance (no GPS sync yet)

// ================================================================
// Serial Command Interface Configuration
// ================================================================
#define SERIAL_BAUD_RATE            115200
#define SERIAL_COMMAND_MAX_LENGTH   256
#define SERIAL_RX_BUFFER_SIZE       512

// Meshtastic-style framing
#define SERIAL_FRAME_START_BYTE     0x94
#define SERIAL_FRAME_END_BYTE       0x95
#define SERIAL_MAX_FRAME_SIZE       512

// ================================================================
// Display Configuration
// ================================================================
#if HAS_TFT
    #define DISPLAY_UPDATE_INTERVAL_MS  1000    // 1 Hz refresh
    #define ROSTER_MAX_VISIBLE_NODES    10      // Lines on screen
    #define MESSAGE_INBOX_MAX_SIZE      20      // Stored messages
#endif

// ================================================================
// Bluetooth Configuration
// ================================================================
#if HAS_BLUETOOTH || ENABLE_BLUETOOTH_GATEWAY
    #define BLE_DEVICE_NAME         "LoRa-TAK"
    #define BLE_MTU_SIZE            512
    #define BLE_SERVICE_UUID        "6ba1b218-15a1-461f-9fb8-5557aaa8c9a0"
    #define BLE_TX_CHAR_UUID        "6ba1b218-15a1-461f-9fb8-5557aaa8c9a1"
    #define BLE_RX_CHAR_UUID        "6ba1b218-15a1-461f-9fb8-5557aaa8c9a2"
#endif

// ================================================================
// Memory and Performance
// ================================================================
#define TASK_STACK_SIZE_ROUTING     8192     // FreeRTOS task stack sizes
#define TASK_STACK_SIZE_RADIO       4096
#define TASK_STACK_SIZE_GPS         4096
#define TASK_STACK_SIZE_SERIAL      4096
#define TASK_STACK_SIZE_DISPLAY     4096

#define TASK_PRIORITY_RADIO         5        // Highest (time-critical)
#define TASK_PRIORITY_ROUTING       4
#define TASK_PRIORITY_GPS           3
#define TASK_PRIORITY_SERIAL        2
#define TASK_PRIORITY_DISPLAY       1        // Lowest

// ================================================================
// Debug and Logging
// ================================================================
#define LOG_TAG                     "LoRaTAK"

// Log levels (set CORE_DEBUG_LEVEL in platformio.ini)
#define LOG_LEVEL_NONE              0
#define LOG_LEVEL_ERROR             1
#define LOG_LEVEL_WARN              2
#define LOG_LEVEL_INFO              3
#define LOG_LEVEL_DEBUG             4
#define LOG_LEVEL_VERBOSE           5

// Logging macros
#define LOG_E(format, ...) Serial.printf("[ERROR] " format "\n", ##__VA_ARGS__)
#define LOG_W(format, ...) Serial.printf("[WARN ] " format "\n", ##__VA_ARGS__)
#define LOG_I(format, ...) Serial.printf("[INFO ] " format "\n", ##__VA_ARGS__)
#define LOG_D(format, ...) Serial.printf("[DEBUG] " format "\n", ##__VA_ARGS__)
#define LOG_V(format, ...) Serial.printf("[VERB ] " format "\n", ##__VA_ARGS__)

// ================================================================
// Version Information
// ================================================================
#define FIRMWARE_VERSION_MAJOR      0
#define FIRMWARE_VERSION_MINOR      1
#define FIRMWARE_VERSION_PATCH      0
#define FIRMWARE_VERSION_STRING     "0.1.0-alpha"
#define FIRMWARE_BUILD_DATE         __DATE__
#define FIRMWARE_BUILD_TIME         __TIME__

#endif // CONFIG_H
