/**
 * @file main.cpp
 * @brief Main firmware entry point for LoRa Mesh GPS TAK
 *
 * This firmware implements a complete LoRa mesh networking system with:
 * - AODV routing protocol
 * - HMAC-SHA256 message authentication
 * - GPS position tracking and sharing
 * - Text messaging
 * - Serial command-line interface
 */

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "node_id.h"
#include "security.h"
#include "gps_manager.h"
#include "lora_radio.h"
#include "routing_engine.h"
#include "serial_cli.h"
#include "protobuf_handler.h"
#include "display_manager.h"

// Global objects
NodeID nodeID;
GPSManager gpsManager;
LoRaRadio loraRadio;
RoutingEngine routingEngine;
SerialCLI serialCLI;
DisplayManager displayManager;

// Timing variables
unsigned long last_gps_broadcast = 0;
unsigned long last_maintenance = 0;

// Forward declarations
void onLoRaReceive();
void onApplicationMessage(const LoRaMessage& msg);
bool transmitLoRaMessage(const LoRaMessage& msg);
void broadcastGPSPosition();
void handleReceivedGPS(const LoRaMessage& msg);
void handleReceivedTextMessage(const LoRaMessage& msg);

void setup() {
    // Initialize serial console
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setTxTimeoutMs(0);  // Non-blocking TX - don't hang if USB not connected
    
    // Wait for serial with timeout - don't block if no USB connected
    uint32_t serial_timeout = millis() + 2000;
    while (!Serial && millis() < serial_timeout) {
        delay(10);
    }

    LOG_I("========================================");
    LOG_I("LoRa Mesh GPS Team Awareness Kit");
    LOG_I("Firmware Version: %s", FIRMWARE_VERSION_STRING);
    LOG_I("Device Type: %s", DEVICE_NAME);
    LOG_I("Build Date: %s %s", FIRMWARE_BUILD_DATE, FIRMWARE_BUILD_TIME);
    LOG_I("========================================");

    // Print capabilities
    LOG_I("Capabilities:");
    LOG_I("  LoRa: %s", HAS_LORA ? "YES" : "NO");
    LOG_I("  Bluetooth: %s", HAS_BLUETOOTH ? "YES" : "NO");
    LOG_I("  GPS: %s", HAS_GPS ? "YES" : "NO");
    LOG_I("  TFT Display: %s", HAS_TFT ? "YES" : "NO");
    LOG_I("  Keyboard: %s", HAS_KEYBOARD ? "YES" : "NO");
    LOG_I("  Standalone Mode: %s", STANDALONE_MODE ? "YES" : "NO");
    LOG_I("");

    // ================================================================
    // NVS persistence - mesh state survives reboots
    // Uncomment below to erase NVS for clean testing
    // ================================================================
    // LOG_W("Erasing NVS for clean test state...");
    // Preferences prefs;
    // prefs.begin("mesh", false);
    // prefs.clear();
    // prefs.end();
    // delay(500);
    // LOG_W("NVS erased");

    // ================================================================
    // Initialize Node ID
    // ================================================================
    LOG_I("Initializing Node ID...");
    if (!NodeID::init()) {
        LOG_E("Failed to initialize node ID!");
        while (1) delay(1000);
    }
    LOG_I("Node ID: %s", NodeID::getNodeID().c_str());

    // ================================================================
    // Initialize Security
    // ================================================================
    LOG_I("Initializing security...");
    if (!Security::init()) {
        LOG_E("Failed to initialize security!");
        while (1) delay(1000);
    }
    LOG_I("Security initialized with HMAC-SHA256");

    // ================================================================
    // Initialize GPS Manager
    // ================================================================
    LOG_I("Initializing GPS manager...");
    if (!gpsManager.init()) {
        LOG_E("Failed to initialize GPS manager!");
        while (1) delay(1000);
    }
    LOG_I("GPS manager initialized (mode: %d)", gpsManager.getSourceMode());

    // ================================================================
    // Initialize LoRa Radio
    // ================================================================
#if HAS_LORA
    LOG_I("Initializing LoRa radio...");
    if (!loraRadio.init()) {
        LOG_E("Failed to initialize LoRa radio!");
        while (1) delay(1000);
    }
    LOG_I("LoRa radio ready");
#else
    LOG_W("LoRa hardware not available, running in simulation mode");
#endif

    // ================================================================
    // Initialize Routing Engine
    // ================================================================
    LOG_I("Initializing routing engine...");
    if (!routingEngine.init(NodeID::getNodeID())) {
        LOG_E("Failed to initialize routing engine!");
        while (1) delay(1000);
    }

    // Set callbacks
    routingEngine.setTransmitCallback(transmitLoRaMessage);
    routingEngine.setReceiveCallback(onApplicationMessage);

    LOG_I("Routing engine initialized (AODV protocol)");

    // ================================================================
    // Initialize Serial CLI
    // ================================================================
    LOG_I("Initializing serial CLI...");
    serialCLI.init();
    serialCLI.setRoutingEngine(&routingEngine);
    serialCLI.setGPSManager(&gpsManager);
    serialCLI.printWelcome();

    // ================================================================
    // Initialize Display
    // ================================================================
    LOG_I("Initializing OLED display...");
    if (displayManager.init()) {
        displayManager.setRoutingEngine(&routingEngine);
        LOG_I("Display initialized");
    } else {
        LOG_W("Display initialization failed - continuing without display");
    }

    // ================================================================
    // Startup Complete
    // ================================================================
    LOG_I("");
    LOG_I("========================================");
    LOG_I("SYSTEM READY");
    LOG_I("========================================");
    LOG_I("");
    LOG_I("Type 'help' for available commands");
    LOG_I("");

    // ================================================================
    // Start LoRa in continuous receive mode
    // ================================================================
#if HAS_LORA
    loraRadio.startListening();
    LOG_I("LoRa radio in continuous receive mode");

    // Initiate network join to announce presence and trigger version sync
    routingEngine.initiateNetworkJoin();
    LOG_I("Network join initiated");
#endif

    // Initialize timing
    last_gps_broadcast = millis();
    last_maintenance = millis();
}

void loop() {
    unsigned long now = millis();

    // ================================================================
    // Update GPS
    // ================================================================
    gpsManager.update();

    // Update routing engine with current position for geographic routing
    GPSCoordinate gps_pos;
    if (gpsManager.hasFix() && gpsManager.getPosition(gps_pos)) {
        routingEngine.setMyPosition(gps_pos);
    }

    // ================================================================
    // Check for LoRa messages
    // ================================================================
#if HAS_LORA
    LoRaMessage msg;
    int8_t rssi;
    if (loraRadio.receive(msg, rssi)) {
        // Process received message through routing engine
        routingEngine.processMessage(msg, rssi);
    }
#endif

    // ================================================================
    // Routing Engine Maintenance
    // ================================================================
    if (now - last_maintenance >= 1000) { // Every second
        routingEngine.maintain();
        last_maintenance = now;
    }

    // ================================================================
    // Periodic GPS Broadcast
    // ================================================================
    if (now - last_gps_broadcast >= GPS_UPDATE_INTERVAL_MS) {
        broadcastGPSPosition();
        last_gps_broadcast = now;
    }

    // ================================================================
    // Process Serial Commands
    // ================================================================
    serialCLI.process();

    // ================================================================
    // Update Display
    // ================================================================
    static unsigned long last_display_update = 0;
    if (now - last_display_update >= 500) {  // Update display at 2Hz
        displayManager.update();
        last_display_update = now;
    }

    // ================================================================
    // Powerbank Keep-Alive
    // Many USB powerbanks auto-shutoff when current draw is too low.
    // Periodically pulse the LED or do a brief high-current operation
    // to keep the powerbank active.
    // ================================================================
    static unsigned long last_keepalive = 0;
    if (now - last_keepalive >= 30000) {  // Every 30 seconds
        // Brief LED flash to draw current and keep powerbank alive
        #if defined(LED_PIN) || defined(LED_BUILTIN)
        #ifdef LED_PIN
        const int led = LED_PIN;
        #else
        const int led = LED_BUILTIN;
        #endif
        pinMode(led, OUTPUT);
        for (int i = 0; i < 5; i++) {
            digitalWrite(led, HIGH);
            delay(50);
            digitalWrite(led, LOW);
            delay(50);
        }
        #endif
        last_keepalive = now;
        LOG_D("Powerbank keep-alive pulse");
    }

    // Small delay to prevent watchdog issues
    delay(10);
}

// ================================================================
// Callback Functions
// ================================================================

bool transmitLoRaMessage(const LoRaMessage& msg) {
#if HAS_LORA
    return loraRadio.transmit(msg);
#else
    LOG_W("LoRa not available, cannot transmit");
    return false;
#endif
}

void onApplicationMessage(const LoRaMessage& msg) {
    // Handle different message types
    switch (msg.message_type) {
        case MESSAGE_TYPE_GPS_UPDATE:
            handleReceivedGPS(msg);
            break;

        case MESSAGE_TYPE_TEXT_MESSAGE:
            handleReceivedTextMessage(msg);
            break;

        case MESSAGE_TYPE_EMERGENCY:
            LOG_W("EMERGENCY message received from: %s", msg.source_id);
            // TODO: Handle emergency
            break;

        case MESSAGE_TYPE_DATA:
            LOG_D("Data message received from: %s", msg.source_id);
            // TODO: Handle data
            break;

        default:
            LOG_D("Unhandled message type: %d", msg.message_type);
            break;
    }
}

void broadcastGPSPosition() {
    GPSCoordinate pos;
    if (!gpsManager.getPosition(pos)) {
        LOG_V("No GPS position to broadcast");
        return;
    }

    // Encode GPS coordinate
    uint8_t payload_buf[128];
    size_t payload_len = ProtobufHandler::encodeGPSCoordinate(pos, payload_buf, sizeof(payload_buf));

    if (payload_len > 0) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + payload_len);
        if (routingEngine.sendMessage("", MESSAGE_TYPE_GPS_UPDATE, payload, PRIORITY_ROUTINE)) {
            LOG_V("GPS position broadcast: %.6f, %.6f", pos.latitude, pos.longitude);
        }
    }
}

void handleReceivedGPS(const LoRaMessage& msg) {
    GPSCoordinate gps;
    // payload is now PB_BYTES_ARRAY_T - access via .bytes and .size
    if (!ProtobufHandler::decodeGPSCoordinate(msg.payload.bytes, msg.payload.size, gps)) {
        LOG_E("Failed to decode GPS from: %s", msg.source_id);
        return;
    }

    // Calculate distance and bearing to this node
    GPSCoordinate my_pos;
    double distance = 0.0;
    double bearing = 0.0;

    if (gpsManager.getPosition(my_pos)) {
        distance = GPSManager::calculateDistance(my_pos, gps);
        bearing = GPSManager::calculateBearing(my_pos, gps);
    }

    LOG_I("GPS from %s: %.6f, %.6f (%.0f m, %.0fÃƒâ€šÃ‚Â°)",
          msg.source_id, gps.latitude, gps.longitude, distance, bearing);
}

void handleReceivedTextMessage(const LoRaMessage& msg) {
    // Decode TextMessagePayload using actual nanopb type
    TextMessagePayload txt = lora_mesh_v1_TextMessagePayload_init_zero;

    // payload is now PB_BYTES_ARRAY_T - access via .bytes and .size
    if (!ProtobufHandler::decodePayload(msg.payload.bytes, msg.payload.size, txt)) {
        LOG_E("Failed to decode text message from: %s", msg.source_id);
        return;
    }

    // Use source_id from message if sender_callsign wasn't decoded
    // txt.text and txt.sender_callsign are now char arrays
    String sender = strlen(txt.sender_callsign) > 0 ? String(txt.sender_callsign) : String(msg.source_id);

    LOG_I("");
    LOG_I("========================================");
    LOG_I("TEXT MESSAGE RECEIVED");
    LOG_I("From: %s", sender.c_str());
    LOG_I("To: %s", msg.destination_id);
    LOG_I("Message: %s", txt.text);
    LOG_I("========================================");
    LOG_I("");

    // Add to display (get RSSI from routing engine stats)
    displayManager.addMessage(txt.text, msg.source_id, 0);
    displayManager.setMode(DISPLAY_MODE_MESSAGES);  // Switch to messages view
}
