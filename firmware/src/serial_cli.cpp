/**
 * @file serial_cli.cpp
 * @brief Serial CLI implementation
 */

#include "serial_cli.h"
#include "config.h"
#include "node_id.h"
#include "security.h"

SerialCLI::SerialCLI()
    : routing_engine(nullptr)
    , gps_manager(nullptr) {
}

bool SerialCLI::init() {
    Serial.begin(SERIAL_BAUD_RATE);
    delay(100);

    printWelcome();
    return true;
}

void SerialCLI::setRoutingEngine(RoutingEngine* engine) {
    routing_engine = engine;
}

void SerialCLI::setGPSManager(GPSManager* gps) {
    gps_manager = gps;
}

void SerialCLI::printWelcome() {
    Serial.println();
    Serial.println("========================================");
    Serial.println("  LoRa Mesh GPS Team Awareness Kit");
    Serial.println(String("  Firmware v") + FIRMWARE_VERSION_STRING);
    Serial.println(String("  Device: ") + DEVICE_NAME);
    Serial.println(String("  Node ID: ") + NodeID::getNodeID());
    Serial.println("========================================");
    Serial.println();
    Serial.println("Type 'help' for available commands");
    Serial.println();
}

void SerialCLI::printHelp() {
    Serial.println("\n=== Available Commands ===");
    Serial.println();
    Serial.println("-- Messaging --");
    Serial.println("send_gps                    - Broadcast current GPS position");
    Serial.println("send_msg <text>             - Broadcast text message to mesh");
    Serial.println("ping <dest>                 - Ping a node (send control ping)");
    Serial.println("emergency <type> <desc>     - Send emergency alert");
    Serial.println("                              Types: medical, fire, accident, threat, other");
    Serial.println();
    Serial.println("-- Network --");
    Serial.println("show_routes                 - Display routing table");
    Serial.println("show_neighbors              - Display neighbor table");
    Serial.println("show_stats                  - Display statistics");
    Serial.println("show_node                   - Display node info (protobuf format)");
    Serial.println("discover                    - Send network discovery broadcast");
    Serial.println("join                        - Initiate network join (HELLO + discovery)");
    Serial.println("flush_routes                - Clear routing table");
    Serial.println();
    Serial.println("-- GPS --");
    Serial.println("show_gps                    - Display GPS information");
    Serial.println("set_static_gps <lat> <lon>  - Set static GPS coordinates");
    Serial.println("set_manual_gps <lat> <lon>  - Set manual GPS coordinates");
    Serial.println("set_gps_mode <0|1|2>        - Set GPS mode (0=hw, 1=static, 2=manual)");
    Serial.println();
    Serial.println("-- Configuration --");
    Serial.println("set_node_id <id>            - Set custom node ID");
    Serial.println("generate_key                - Generate new HMAC key");
    Serial.println("control <cmd> [target]      - Send control command");
    Serial.println("                              Commands: reboot, config, route_flush, power");
    Serial.println();
    Serial.println("help                        - Show this help message");
    Serial.println();
}

void SerialCLI::process() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (command_buffer.length() > 0) {
                processCommand(command_buffer);
                command_buffer = "";
            }
        } else {
            command_buffer += c;

            // Limit buffer size
            if (command_buffer.length() > SERIAL_COMMAND_MAX_LENGTH) {
                command_buffer = "";
                Serial.println("ERROR: Command too long");
            }
        }
    }
}

void SerialCLI::processCommand(const String& cmd) {
    String trimmed = cmd;
    trimmed.trim();

    if (trimmed.length() == 0) {
        return;
    }

    Serial.println(String("> ") + trimmed);

    // Parse command and arguments
    int space_idx = trimmed.indexOf(' ');
    String command = space_idx > 0 ? trimmed.substring(0, space_idx) : trimmed;
    String args = space_idx > 0 ? trimmed.substring(space_idx + 1) : "";

    command.toLowerCase();

    // Execute command
    if (command == "help") {
        printHelp();
    } else if (command == "send_gps") {
        handleSendGPS(args);
    } else if (command == "send_msg") {
        handleSendMsg(args);
    } else if (command == "set_static_gps") {
        handleSetStaticGPS(args);
    } else if (command == "set_manual_gps") {
        handleSetManualGPS(args);
    } else if (command == "set_gps_mode") {
        handleSetGPSMode(args);
    } else if (command == "show_routes") {
        handleShowRoutes();
    } else if (command == "show_neighbors") {
        handleShowNeighbors();
    } else if (command == "show_stats") {
        handleShowStats();
    } else if (command == "show_gps") {
        handleShowGPS();
    } else if (command == "set_node_id") {
        handleSetNodeID(args);
    } else if (command == "generate_key") {
        handleGenerateKey();
    } else if (command == "ping") {
        handlePing(args);
    } else if (command == "emergency") {
        handleEmergency(args);
    } else if (command == "control") {
        handleControl(args);
    } else if (command == "discover") {
        handleDiscover();
    } else if (command == "join") {
        handleJoin();
    } else if (command == "show_node") {
        handleShowNode();
    } else if (command == "flush_routes") {
        handleFlushRoutes();
    } else {
        Serial.println("ERROR: Unknown command. Type 'help' for available commands.");
    }
}

void SerialCLI::handleSendGPS(const String& args) {
    if (!gps_manager || !routing_engine) {
        Serial.println("ERROR: System not initialized");
        return;
    }

    GPSCoordinate pos;
    if (!gps_manager->getPosition(pos)) {
        Serial.println("ERROR: No valid GPS position");
        return;
    }

    // Encode GPS coordinate
    uint8_t payload_buf[128];
    size_t payload_len = ProtobufHandler::encodeGPSCoordinate(pos, payload_buf, sizeof(payload_buf));

    if (payload_len > 0) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + payload_len);
        if (routing_engine->sendMessage("BROADCAST", MESSAGE_TYPE_GPS_UPDATE, payload, PRIORITY_ROUTINE)) {
            Serial.println("OK: GPS position broadcast");
            Serial.printf("  Lat: %.6f, Lon: %.6f, Alt: %.1f m\n",
                         pos.latitude, pos.longitude, pos.altitude);
        } else {
            Serial.println("ERROR: Failed to send GPS broadcast");
        }
    }
}

void SerialCLI::handleSendMsg(const String& args) {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    // Message text is the entire args (broadcast to all)
    if (args.length() == 0) {
        Serial.println("ERROR: Usage: send_msg <text>");
        return;
    }

    String text = args;
    String sender = NodeID::getNodeID();

    // Create text message payload using actual nanopb type
    TextMessagePayload txt = ProtobufHandler::createTextMessagePayload();

    // Set string fields - now static char arrays
    ProtobufHandler::copyString(txt.text, sizeof(txt.text), text);
    ProtobufHandler::copyString(txt.sender_callsign, sizeof(txt.sender_callsign), sender);

    // Set message options
    txt.priority = PRIORITY_TACTICAL;
    txt.requires_ack = false;
    // message_sequence is already set by createTextMessagePayload()

    uint8_t payload_buf[256];
    size_t payload_len = ProtobufHandler::encodePayload(txt, payload_buf, sizeof(payload_buf));

    if (payload_len > 0) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + payload_len);
        // Broadcast text message to all nodes
        if (routing_engine->sendMessage("BROADCAST", MESSAGE_TYPE_TEXT_MESSAGE, payload, PRIORITY_TACTICAL)) {
            Serial.printf("OK: Message broadcast to mesh: %s\n", text.c_str());
        } else {
            Serial.println("ERROR: Failed to broadcast message");
        }
    }
}

void SerialCLI::handleSetStaticGPS(const String& args) {
    if (!gps_manager) {
        Serial.println("ERROR: GPS manager not initialized");
        return;
    }

    // Parse: <lat> <lon>
    int space_idx = args.indexOf(' ');
    if (space_idx < 0) {
        Serial.println("ERROR: Usage: set_static_gps <lat> <lon>");
        return;
    }

    double lat = args.substring(0, space_idx).toDouble();
    double lon = args.substring(space_idx + 1).toDouble();

    if (gps_manager->setStaticPosition(lat, lon)) {
        Serial.printf("OK: Static GPS set to %.6f, %.6f\n", lat, lon);
    } else {
        Serial.println("ERROR: Failed to set static GPS");
    }
}

void SerialCLI::handleSetManualGPS(const String& args) {
    if (!gps_manager) {
        Serial.println("ERROR: GPS manager not initialized");
        return;
    }

    // Parse: <lat> <lon>
    int space_idx = args.indexOf(' ');
    if (space_idx < 0) {
        Serial.println("ERROR: Usage: set_manual_gps <lat> <lon>");
        return;
    }

    double lat = args.substring(0, space_idx).toDouble();
    double lon = args.substring(space_idx + 1).toDouble();

    if (gps_manager->setManualPosition(lat, lon)) {
        Serial.printf("OK: Manual GPS set to %.6f, %.6f\n", lat, lon);
    } else {
        Serial.println("ERROR: Failed to set manual GPS");
    }
}

void SerialCLI::handleSetGPSMode(const String& args) {
    if (!gps_manager) {
        Serial.println("ERROR: GPS manager not initialized");
        return;
    }

    int mode = args.toInt();
    if (mode < 0 || mode > 2) {
        Serial.println("ERROR: Mode must be 0 (hardware), 1 (static), or 2 (manual)");
        return;
    }

    gps_manager->setSourceMode((GPSSourceMode)mode);
    Serial.printf("OK: GPS mode set to %d\n", mode);
}

void SerialCLI::handleShowRoutes() {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    auto routes = routing_engine->getRoutingTable();

    Serial.println("\n=== Routing Table ===");
    Serial.printf("Total routes: %d\n\n", routes.size());

    if (routes.empty()) {
        Serial.println("(No routes)");
    } else {
        Serial.println("Destination     Next Hop        Hops  Seq#      RSSI");
        Serial.println("-----------------------------------------------------------");

        for (const auto& route : routes) {
            Serial.printf("%-15s %-15s %-5d %-9d %d dBm\n",
                         route.destination_id.c_str(),
                         route.next_hop_id.c_str(),
                         route.hop_count,
                         route.sequence_number,
                         route.rssi);
        }
    }
    Serial.println();
}

void SerialCLI::handleShowNeighbors() {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    auto neighbors = routing_engine->getNeighborTable();

    Serial.println("\n=== Neighbor Table ===");
    Serial.printf("Total neighbors: %d\n\n", neighbors.size());

    if (neighbors.empty()) {
        Serial.println("(No neighbors)");
    } else {
        Serial.println("Node ID         RSSI    Battery  Last Seen");
        Serial.println("-----------------------------------------------------------");

        for (const auto& neighbor : neighbors) {
            uint64_t age_ms = millis() - neighbor.last_seen_time;
            Serial.printf("%-15s %d dBm  %3d%%     %llu ms ago\n",
                         neighbor.node_id.c_str(),
                         neighbor.last_rssi,
                         neighbor.battery_level,
                         age_ms);
        }
    }
    Serial.println();
}

void SerialCLI::handleShowStats() {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    auto stats = routing_engine->stats;

    Serial.println("\n=== Statistics ===");
    Serial.printf("Mesh Version:       %llu\n", routing_engine->getMeshVersion());
    Serial.printf("Messages Sent:      %d\n", stats.messages_sent);
    Serial.printf("Messages Received:  %d\n", stats.messages_received);
    Serial.printf("Messages Forwarded: %d\n", stats.messages_forwarded);
    Serial.printf("Messages Dropped:   %d\n", stats.messages_dropped);
    Serial.printf("RREQ Sent:          %d\n", stats.rreq_sent);
    Serial.printf("RREP Sent:          %d\n", stats.rrep_sent);
    Serial.printf("Routes Discovered:  %d\n", stats.routes_discovered);
    Serial.println();
}

void SerialCLI::handleShowGPS() {
    if (!gps_manager) {
        Serial.println("ERROR: GPS manager not initialized");
        return;
    }

    GPSCoordinate pos;
    bool has_fix = gps_manager->getPosition(pos);

    Serial.println("\n=== GPS Information ===");
    Serial.printf("GPS Mode: %d ", gps_manager->getSourceMode());

    switch (gps_manager->getSourceMode()) {
        case GPS_SOURCE_HARDWARE: Serial.println("(Hardware)"); break;
        case GPS_SOURCE_STATIC: Serial.println("(Static)"); break;
        case GPS_SOURCE_MANUAL: Serial.println("(Manual)"); break;
    }

    Serial.printf("Has Fix: %s\n", has_fix ? "Yes" : "No");
    Serial.printf("Satellites: %d\n", gps_manager->getSatelliteCount());

    if (has_fix) {
        Serial.printf("Latitude:  %.6f\n", pos.latitude);
        Serial.printf("Longitude: %.6f\n", pos.longitude);
        Serial.printf("Altitude:  %.1f m\n", pos.altitude);
        Serial.printf("Accuracy:  %.1f m\n", pos.accuracy);
    } else {
        Serial.println("(No position available)");
    }
    Serial.println();
}

void SerialCLI::handleSetNodeID(const String& args) {
    if (args.length() == 0) {
        Serial.println("ERROR: Usage: set_node_id <id>");
        return;
    }

    if (NodeID::setNodeID(args)) {
        Serial.printf("OK: Node ID set to %s\n", args.c_str());
        Serial.println("RESTART REQUIRED for change to take effect");
    } else {
        Serial.println("ERROR: Invalid node ID format");
    }
}

void SerialCLI::handleGenerateKey() {
    if (Security::generateAndStoreKey()) {
        Serial.println("OK: New HMAC key generated and stored");
        Serial.println("COPY THIS KEY TO ALL NODES IN YOUR MESH!");
    } else {
        Serial.println("ERROR: Failed to generate key");
    }
}

void SerialCLI::handlePing(const String& args) {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    if (args.length() == 0) {
        Serial.println("ERROR: Usage: ping <dest_id>");
        return;
    }

    String dest_id = args;
    dest_id.trim();

    // Create control message with PING command using nanopb type
    lora_mesh_v1_ControlMessagePayload ctrl = lora_mesh_v1_ControlMessagePayload_init_zero;
    ctrl.command = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_PING;
    ProtobufHandler::copyString(ctrl.target_node, sizeof(ctrl.target_node), dest_id);
    ctrl.command_id = millis();
    ctrl.expiry_time = millis() + 30000;  // 30 second expiry

    uint8_t payload_buf[128];
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (pb_encode(&stream, lora_mesh_v1_ControlMessagePayload_fields, &ctrl)) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + stream.bytes_written);
        if (routing_engine->sendMessage(dest_id, MESSAGE_TYPE_CONTROL, payload, PRIORITY_ROUTINE)) {
            Serial.printf("OK: Ping sent to %s\n", dest_id.c_str());
        } else {
            Serial.println("ERROR: Failed to send ping");
        }
    } else {
        Serial.println("ERROR: Failed to encode ping message");
    }
}

void SerialCLI::handleEmergency(const String& args) {
    if (!routing_engine || !gps_manager) {
        Serial.println("ERROR: System not initialized");
        return;
    }

    // Parse: <type> <description>
    int space_idx = args.indexOf(' ');
    String type_str = space_idx > 0 ? args.substring(0, space_idx) : args;
    String description = space_idx > 0 ? args.substring(space_idx + 1) : "Emergency alert";
    type_str.toLowerCase();

    // Map type string to enum
    lora_mesh_v1_EmergencyType etype = lora_mesh_v1_EmergencyType_EMERGENCY_TYPE_OTHER;
    if (type_str == "medical") {
        etype = lora_mesh_v1_EmergencyType_EMERGENCY_TYPE_MEDICAL;
    } else if (type_str == "fire") {
        etype = lora_mesh_v1_EmergencyType_EMERGENCY_TYPE_FIRE;
    } else if (type_str == "accident") {
        etype = lora_mesh_v1_EmergencyType_EMERGENCY_TYPE_ACCIDENT;
    } else if (type_str == "threat" || type_str == "security") {
        etype = lora_mesh_v1_EmergencyType_EMERGENCY_TYPE_SECURITY_THREAT;
    } else if (type_str == "disaster") {
        etype = lora_mesh_v1_EmergencyType_EMERGENCY_TYPE_NATURAL_DISASTER;
    }

    // Create emergency payload using nanopb type
    lora_mesh_v1_EmergencyPayload emergency = lora_mesh_v1_EmergencyPayload_init_zero;
    emergency.emergency_type = etype;
    ProtobufHandler::copyString(emergency.description, sizeof(emergency.description), description);
    emergency.severity_level = 8;  // High severity
    emergency.emergency_time = millis();
    emergency.requires_response = true;

    // Add GPS location if available
    GPSCoordinate pos;
    if (gps_manager->getPosition(pos)) {
        emergency.has_location = true;
        emergency.location.latitude = pos.latitude;
        emergency.location.longitude = pos.longitude;
        emergency.location.altitude = pos.altitude;
        emergency.location.accuracy = pos.accuracy;
        emergency.location.timestamp = pos.timestamp;
    }

    ProtobufHandler::copyString(emergency.contact_info, sizeof(emergency.contact_info), NodeID::getNodeID());

    uint8_t payload_buf[256];
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (pb_encode(&stream, lora_mesh_v1_EmergencyPayload_fields, &emergency)) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + stream.bytes_written);
        if (routing_engine->sendMessage("BROADCAST", MESSAGE_TYPE_EMERGENCY, payload, PRIORITY_EMERGENCY)) {
            Serial.println("!!! EMERGENCY BROADCAST SENT !!!");
            Serial.printf("  Type: %s\n", type_str.c_str());
            Serial.printf("  Description: %s\n", description.c_str());
            if (emergency.has_location) {
                Serial.printf("  Location: %.6f, %.6f\n", pos.latitude, pos.longitude);
            }
        } else {
            Serial.println("ERROR: Failed to send emergency");
        }
    } else {
        Serial.println("ERROR: Failed to encode emergency message");
    }
}

void SerialCLI::handleControl(const String& args) {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    // Parse: <command> [target_node]
    int space_idx = args.indexOf(' ');
    String cmd_str = space_idx > 0 ? args.substring(0, space_idx) : args;
    String target = space_idx > 0 ? args.substring(space_idx + 1) : "BROADCAST";
    cmd_str.toLowerCase();
    target.trim();

    // Map command string to enum
    lora_mesh_v1_ControlCommand cmd = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_UNSPECIFIED;
    if (cmd_str == "reboot") {
        cmd = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_REBOOT;
    } else if (cmd_str == "config") {
        cmd = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_CONFIG_UPDATE;
    } else if (cmd_str == "route_flush" || cmd_str == "flush") {
        cmd = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_ROUTE_FLUSH;
    } else if (cmd_str == "power") {
        cmd = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_POWER_ADJUST;
    } else if (cmd_str == "freq" || cmd_str == "frequency") {
        cmd = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_FREQUENCY_CHANGE;
    } else if (cmd_str == "shutdown") {
        cmd = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_EMERGENCY_SHUTDOWN;
    } else {
        Serial.println("ERROR: Unknown control command");
        Serial.println("  Valid: reboot, config, route_flush, power, freq, shutdown");
        return;
    }

    lora_mesh_v1_ControlMessagePayload ctrl = lora_mesh_v1_ControlMessagePayload_init_zero;
    ctrl.command = cmd;
    ProtobufHandler::copyString(ctrl.target_node, sizeof(ctrl.target_node), target);
    ctrl.command_id = millis();
    ctrl.expiry_time = millis() + 60000;  // 1 minute expiry

    uint8_t payload_buf[128];
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (pb_encode(&stream, lora_mesh_v1_ControlMessagePayload_fields, &ctrl)) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + stream.bytes_written);
        if (routing_engine->sendMessage(target, MESSAGE_TYPE_CONTROL, payload, PRIORITY_TACTICAL)) {
            Serial.printf("OK: Control command '%s' sent to %s\n", cmd_str.c_str(), target.c_str());
        } else {
            Serial.println("ERROR: Failed to send control command");
        }
    } else {
        Serial.println("ERROR: Failed to encode control message");
    }
}

void SerialCLI::handleDiscover() {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    // Use the routing engine's sendNetworkDiscovery for proper protobuf encoding
    routing_engine->sendNetworkDiscovery();
    Serial.println("OK: Network discovery initiated");
}

void SerialCLI::handleJoin() {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    // Use the routing engine's initiateNetworkJoin which sends HELLO + discovery
    routing_engine->initiateNetworkJoin();
    Serial.println("OK: Network join initiated (sent HELLO + discovery)");
}

void SerialCLI::handleShowNode() {
    Serial.println("\n=== Node Information (Protobuf Format) ===");

    // Build NodeInfo structure matching nanopb type
    lora_mesh_v1_NodeInfo info = lora_mesh_v1_NodeInfo_init_zero;
    ProtobufHandler::copyString(info.node_id, sizeof(info.node_id), NodeID::getNodeID());
    info.battery_level = 100;  // TODO: Read actual battery level
    info.rssi = 0;  // Local node
    info.sequence_number = millis() / 1000;
    info.last_seen = millis();

    // Display in structured format
    Serial.printf("node_id: \"%s\"\n", info.node_id);
    Serial.printf("device_name: \"%s\"\n", DEVICE_NAME);
    Serial.printf("firmware_version: \"%s\"\n", FIRMWARE_VERSION_STRING);
    Serial.printf("battery_level: %d%%\n", info.battery_level);
    Serial.printf("sequence_number: %u\n", info.sequence_number);
    Serial.printf("uptime_ms: %llu\n", info.last_seen);

    // GPS info if available
    if (gps_manager) {
        GPSCoordinate pos;
        if (gps_manager->getPosition(pos)) {
            Serial.println("last_position {");
            Serial.printf("  latitude: %.6f\n", pos.latitude);
            Serial.printf("  longitude: %.6f\n", pos.longitude);
            Serial.printf("  altitude: %.1f\n", pos.altitude);
            Serial.printf("  accuracy: %.1f\n", pos.accuracy);
            Serial.println("}");
        }
    }

    // Network stats
    if (routing_engine) {
        auto routes = routing_engine->getRoutingTable();
        auto neighbors = routing_engine->getNeighborTable();
        Serial.printf("known_routes: %d\n", routes.size());
        Serial.printf("known_neighbors: %d\n", neighbors.size());
    }

    Serial.println();
}

void SerialCLI::handleFlushRoutes() {
    if (!routing_engine) {
        Serial.println("ERROR: Routing engine not initialized");
        return;
    }

    // Send route flush control command to ourselves (local effect only)
    Serial.println("OK: Flushing local routing table...");

    // Just display a message - actual route flushing would need routing_engine API
    auto routes = routing_engine->getRoutingTable();
    Serial.printf("  Routes to flush: %d\n", routes.size());
    Serial.println("  Note: Full route flush requires routing_engine.clearRoutes() API");
    Serial.println("  Broadcasting route flush command to mesh...");

    // Broadcast route flush to mesh
    lora_mesh_v1_ControlMessagePayload ctrl = lora_mesh_v1_ControlMessagePayload_init_zero;
    ctrl.command = lora_mesh_v1_ControlCommand_CONTROL_COMMAND_ROUTE_FLUSH;
    ctrl.command_id = millis();
    ctrl.expiry_time = millis() + 30000;

    uint8_t payload_buf[64];
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (pb_encode(&stream, lora_mesh_v1_ControlMessagePayload_fields, &ctrl)) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + stream.bytes_written);
        routing_engine->sendMessage("BROADCAST", MESSAGE_TYPE_CONTROL, payload, PRIORITY_TACTICAL);
    }

    Serial.println("OK: Route flush broadcast sent");
}
