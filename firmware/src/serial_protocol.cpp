/**
 * @file serial_protocol.cpp
 * @brief SLIP-framed protobuf serial protocol implementation
 * 
 * Implements Meshtastic-style serial protocol using:
 * - SLIP framing (RFC 1055)
 * - Protobuf message encoding
 */

#include "serial_protocol.h"
#include "routing_engine.h"
#include "gps_manager.h"
#include "config.h"
#include "node_id.h"
#include "v1/serial.pb.h"
#include <pb_encode.h>
#include <pb_decode.h>

// ================================================================
// SLIP Decoder
// ================================================================

SLIPDecoder::SLIPDecoder() {
    reset();
}

void SLIPDecoder::reset() {
    length_ = 0;
    escaping_ = false;
    started_ = false;
}

bool SLIPDecoder::feed(uint8_t byte) {
    // Handle SLIP special bytes
    if (byte == slip::END) {
        if (started_ && length_ > 0) {
            // Complete packet ready
            started_ = false;
            return true;
        }
        // Start of new packet
        started_ = true;
        length_ = 0;
        escaping_ = false;
        return false;
    }
    
    if (!started_) {
        return false;  // Ignore bytes outside frame
    }
    
    if (byte == slip::ESC) {
        escaping_ = true;
        return false;
    }
    
    if (escaping_) {
        escaping_ = false;
        if (byte == slip::ESC_END) {
            byte = slip::END;
        } else if (byte == slip::ESC_ESC) {
            byte = slip::ESC;
        }
        // Invalid escape sequence - use byte as-is
    }
    
    // Add byte to buffer if space available
    if (length_ < slip::MAX_PACKET_SIZE) {
        buffer_[length_++] = byte;
    }
    
    return false;
}

// ================================================================
// SLIP Encoder
// ================================================================

size_t SLIPEncoder::encode(const uint8_t* data, size_t len, uint8_t* output) {
    size_t pos = 0;
    
    // Start with END marker
    output[pos++] = slip::END;
    
    // Encode data with escaping
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        
        if (byte == slip::END) {
            output[pos++] = slip::ESC;
            output[pos++] = slip::ESC_END;
        } else if (byte == slip::ESC) {
            output[pos++] = slip::ESC;
            output[pos++] = slip::ESC_ESC;
        } else {
            output[pos++] = byte;
        }
    }
    
    // End with END marker
    output[pos++] = slip::END;
    
    return pos;
}

// ================================================================
// Serial Protocol Handler
// ================================================================

SerialProtocol::SerialProtocol(Stream& serial)
    : serial_(serial)
    , routing_(nullptr)
    , gps_(nullptr)
    , events_enabled_(true)
    , next_packet_id_(1) {
}

void SerialProtocol::update() {
    // Process all available bytes
    while (serial_.available() > 0) {
        uint8_t byte = serial_.read();
        
        if (decoder_.feed(byte)) {
            // Complete packet received
            processPacket(decoder_.getPacket(), decoder_.getLength());
            decoder_.reset();
        }
    }
}

void SerialProtocol::processPacket(const uint8_t* data, size_t len) {
    // Decode SerialPacket protobuf
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(data, len);
    
    if (!pb_decode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        Serial.println("ERROR: Failed to decode serial packet");
        return;
    }
    
    uint32_t request_id = packet.packet_id;
    
    // Handle ToDevice commands
    if (packet.which_payload == lora_mesh_v1_SerialPacket_to_device_tag) {
        lora_mesh_v1_ToDevice* cmd = &packet.payload.to_device;
        
        switch (cmd->which_command) {
            case lora_mesh_v1_ToDevice_get_info_tag:
                handleGetInfo(request_id);
                break;
            case lora_mesh_v1_ToDevice_get_gps_tag:
                handleGetGPS(request_id);
                break;
            case lora_mesh_v1_ToDevice_get_neighbors_tag:
                handleGetNeighbors(request_id);
                break;
            case lora_mesh_v1_ToDevice_get_routes_tag:
                handleGetRoutes(request_id);
                break;
            case lora_mesh_v1_ToDevice_get_roster_tag:
                handleGetRoster(request_id);
                break;
            case lora_mesh_v1_ToDevice_get_stats_tag:
                handleGetStats(request_id);
                break;
            case lora_mesh_v1_ToDevice_send_gps_tag:
                handleSendGPS(request_id);
                break;
            case lora_mesh_v1_ToDevice_send_message_tag:
                handleSendMessage(request_id, &cmd->command.send_message);
                break;
            case lora_mesh_v1_ToDevice_discover_tag:
                handleDiscover(request_id);
                break;
            case lora_mesh_v1_ToDevice_join_tag:
                handleJoin(request_id);
                break;
            // TODO: Add remaining command handlers (send_emergency, ping, set_gps, set_node_id)
            default:
                sendResult(request_id, false, "Unknown command");
                break;
        }
    }
}

void SerialProtocol::sendPacket(const uint8_t* data, size_t len) {
    uint8_t encoded[slip::MAX_PACKET_SIZE * 2 + 2];
    size_t encoded_len = SLIPEncoder::encode(data, len, encoded);
    serial_.write(encoded, encoded_len);
}

void SerialProtocol::sendResult(uint32_t request_id, bool success, const char* error) {
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_result_tag;
    resp->request_id = request_id;
    resp->payload.result.success = success;
    if (error) {
        strncpy(resp->payload.result.error, error, sizeof(resp->payload.result.error) - 1);
    }
    
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

// ================================================================
// Command Handlers
// ================================================================

void SerialProtocol::handleGetInfo(uint32_t request_id) {
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_info_tag;
    resp->request_id = request_id;
    
    lora_mesh_v1_GetInfoResponse* info = &resp->payload.info;
    
    // Fill NodeInfo
    info->has_node_info = true;
    strncpy(info->node_info.node_id, NodeID::getNodeID().c_str(), sizeof(info->node_info.node_id) - 1);
    
    if (gps_) {
        GPSCoordinate pos;
        if (gps_->getPosition(pos)) {
            info->node_info.has_position = true;
            info->node_info.position.latitude = pos.latitude;
            info->node_info.position.longitude = pos.longitude;
            info->node_info.position.altitude = pos.altitude;
            info->node_info.position.timestamp = pos.timestamp;
        }
    }
    
    strncpy(info->firmware_version, FIRMWARE_VERSION_STRING, sizeof(info->firmware_version) - 1);
    strncpy(info->protocol_version, "v1", sizeof(info->protocol_version) - 1);
    
    if (routing_) {
        info->mesh_version = routing_->getMeshVersion();
        info->neighbor_count = routing_->getNeighborTable().size();
        info->route_count = routing_->getRoutingTable().size();
    }
    
    info->uptime_ms = millis();
    
    uint8_t buffer[512];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::handleGetGPS(uint32_t request_id) {
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_gps_tag;
    resp->request_id = request_id;
    
    lora_mesh_v1_GetGPSResponse* gps_resp = &resp->payload.gps;
    
    if (gps_) {
        GPSCoordinate pos;
        if (gps_->getPosition(pos)) {
            gps_resp->has_fix = true;
            gps_resp->has_position = true;
            gps_resp->position.latitude = pos.latitude;
            gps_resp->position.longitude = pos.longitude;
            gps_resp->position.altitude = pos.altitude;
            gps_resp->position.timestamp = pos.timestamp;
            gps_resp->satellites = gps_->getSatelliteCount();
            gps_resp->hdop = pos.accuracy;  // Use accuracy from GPSCoordinate
        }
    }
    
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::handleGetNeighbors(uint32_t request_id) {
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_neighbors_tag;
    resp->request_id = request_id;
    
    lora_mesh_v1_GetNeighborsResponse* neighbors_resp = &resp->payload.neighbors;
    
    if (routing_) {
        std::vector<NeighborEntry> neighbor_table = routing_->getNeighborTable();
        neighbors_resp->neighbors_count = std::min(neighbor_table.size(), (size_t)16);
        
        for (size_t i = 0; i < neighbors_resp->neighbors_count; i++) {
            const NeighborEntry& n = neighbor_table[i];
            lora_mesh_v1_NodeInfo* node = &neighbors_resp->neighbors[i];
            
            strncpy(node->node_id, n.node_id.c_str(), sizeof(node->node_id) - 1);
            node->rssi = n.last_rssi;
            node->battery_level = n.battery_level;
            node->last_seen = n.last_seen_time;
            node->sequence_number = n.sequence_number;
            
            if (n.last_position.timestamp > 0) {
                node->has_position = true;
                node->position.latitude = n.last_position.latitude;
                node->position.longitude = n.last_position.longitude;
                node->position.altitude = n.last_position.altitude;
                node->position.timestamp = n.last_position.timestamp;
            }
        }
    }
    
    uint8_t buffer[1024];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::handleGetRoutes(uint32_t request_id) {
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_routes_tag;
    resp->request_id = request_id;
    
    lora_mesh_v1_GetRoutesResponse* routes_resp = &resp->payload.routes;
    
    if (routing_) {
        std::vector<RouteEntry> route_table = routing_->getRoutingTable();
        routes_resp->routes_count = std::min(route_table.size(), (size_t)32);
        
        for (size_t i = 0; i < routes_resp->routes_count; i++) {
            const RouteEntry& r = route_table[i];
            lora_mesh_v1_RouteEntry* route = &routes_resp->routes[i];
            
            strncpy(route->destination, r.destination_id.c_str(), sizeof(route->destination) - 1);
            strncpy(route->next_hop, r.next_hop_id.c_str(), sizeof(route->next_hop) - 1);
            route->hop_count = r.hop_count;
            route->rssi = r.rssi;
            route->last_update = r.last_update_time;
        }
    }
    
    uint8_t buffer[1024];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::handleGetRoster(uint32_t request_id) {
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_roster_tag;
    resp->request_id = request_id;
    
    lora_mesh_v1_GetRosterResponse* roster_resp = &resp->payload.roster;
    
    // Add self first
    roster_resp->roster_count = 0;
    
    lora_mesh_v1_RosterEntry* self_entry = &roster_resp->roster[roster_resp->roster_count++];
    self_entry->has_node = true;
    strncpy(self_entry->node.node_id, NodeID::getNodeID().c_str(), sizeof(self_entry->node.node_id) - 1);
    self_entry->is_self = true;
    self_entry->is_active = true;
    
    if (gps_) {
        GPSCoordinate pos;
        if (gps_->getPosition(pos)) {
            self_entry->node.has_position = true;
            self_entry->node.position.latitude = pos.latitude;
            self_entry->node.position.longitude = pos.longitude;
            self_entry->node.position.altitude = pos.altitude;
            self_entry->node.position.timestamp = pos.timestamp;
        }
    }
    
    // Add neighbors
    if (routing_) {
        std::vector<NeighborEntry> neighbors = routing_->getNeighborTable();
        for (const auto& n : neighbors) {
            if (roster_resp->roster_count >= 32) break;
            
            lora_mesh_v1_RosterEntry* entry = &roster_resp->roster[roster_resp->roster_count++];
            entry->has_node = true;
            strncpy(entry->node.node_id, n.node_id.c_str(), sizeof(entry->node.node_id) - 1);
            entry->node.rssi = n.last_rssi;
            entry->node.battery_level = n.battery_level;
            entry->node.last_seen = n.last_seen_time;
            entry->is_self = false;
            entry->is_active = n.is_active;
            
            if (n.last_position.timestamp > 0) {
                entry->node.has_position = true;
                entry->node.position.latitude = n.last_position.latitude;
                entry->node.position.longitude = n.last_position.longitude;
                entry->node.position.altitude = n.last_position.altitude;
                entry->node.position.timestamp = n.last_position.timestamp;
            }
        }
    }
    
    uint8_t buffer[2048];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::handleGetStats(uint32_t request_id) {
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_stats_tag;
    resp->request_id = request_id;
    
    lora_mesh_v1_GetStatsResponse* stats_resp = &resp->payload.stats;
    
    if (routing_) {
        stats_resp->messages_sent = routing_->stats.messages_sent;
        stats_resp->messages_received = routing_->stats.messages_received;
        stats_resp->messages_forwarded = routing_->stats.messages_forwarded;
        stats_resp->messages_dropped = routing_->stats.messages_dropped;
        stats_resp->route_discoveries = routing_->stats.routes_discovered;
        stats_resp->route_errors = routing_->stats.rerr_sent;
        stats_resp->mesh_version = routing_->getMeshVersion();
    }
    stats_resp->uptime_ms = millis();
    
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::handleSendGPS(uint32_t request_id) {
    if (!gps_ || !routing_) {
        sendResult(request_id, false, "GPS or routing not available");
        return;
    }
    
    GPSCoordinate pos;
    if (!gps_->getPosition(pos)) {
        sendResult(request_id, false, "No valid GPS position");
        return;
    }
    
    // Encode GPS coordinate
    uint8_t payload_buf[128];
    size_t payload_len = ProtobufHandler::encodeGPSCoordinate(pos, payload_buf, sizeof(payload_buf));
    
    if (payload_len > 0) {
        std::vector<uint8_t> payload(payload_buf, payload_buf + payload_len);
        if (routing_->sendMessage("BROADCAST", MESSAGE_TYPE_GPS_UPDATE, payload, PRIORITY_ROUTINE)) {
            sendResult(request_id, true);
        } else {
            sendResult(request_id, false, "Failed to send GPS broadcast");
        }
    } else {
        sendResult(request_id, false, "Failed to encode GPS");
    }
}

void SerialProtocol::handleDiscover(uint32_t request_id) {
    if (!routing_) {
        sendResult(request_id, false, "Routing not available");
        return;
    }
    
    routing_->sendNetworkDiscovery();
    sendResult(request_id, true);
}

void SerialProtocol::handleJoin(uint32_t request_id) {
    if (!routing_) {
        sendResult(request_id, false, "Routing not available");
        return;
    }
    
    routing_->initiateNetworkJoin();
    sendResult(request_id, true);
}

void SerialProtocol::handleSetGPS(uint32_t request_id, const uint8_t* data, size_t len) {
    // TODO: Decode SetGPSRequest and apply
    sendResult(request_id, false, "Not implemented");
}

void SerialProtocol::handleSendMessage(uint32_t request_id, const lora_mesh_v1_SendMessageRequest* req) {
    if (!routing_) {
        sendResult(request_id, false, "Routing engine not available");
        return;
    }
    
    // Extract destination and text from the request
    String destination = String(req->destination);
    String text = String(req->text);
    
    if (text.length() == 0) {
        sendResult(request_id, false, "Empty message");
        return;
    }
    
    // Create TextMessagePayload
    lora_mesh_v1_TextMessagePayload txt = lora_mesh_v1_TextMessagePayload_init_zero;
    ProtobufHandler::copyString(txt.text, sizeof(txt.text), text);
    ProtobufHandler::copyString(txt.sender_callsign, sizeof(txt.sender_callsign), NodeID::getNodeID());
    txt.priority = PRIORITY_ROUTINE;
    
    // Encode payload
    uint8_t payload_buf[256];
    size_t payload_len = ProtobufHandler::encodePayload(txt, payload_buf, sizeof(payload_buf));
    
    if (payload_len == 0) {
        sendResult(request_id, false, "Failed to encode message");
        return;
    }
    
    // Send via routing engine
    std::vector<uint8_t> payload(payload_buf, payload_buf + payload_len);
    
    // Use BROADCAST if destination is empty or "BROADCAST"
    String dest = destination;
    if (dest.length() == 0 || dest.equalsIgnoreCase("BROADCAST")) {
        dest = "BROADCAST";
    }
    
    if (routing_->sendMessage(dest, MESSAGE_TYPE_TEXT_MESSAGE, payload, PRIORITY_ROUTINE)) {
        sendResult(request_id, true);
    } else {
        sendResult(request_id, false, "Failed to send message");
    }
}

void SerialProtocol::handleSendEmergency(uint32_t request_id, const uint8_t* data, size_t len) {
    // TODO: Decode SendEmergencyRequest and send
    sendResult(request_id, false, "Not implemented");
}

void SerialProtocol::handlePing(uint32_t request_id, const uint8_t* data, size_t len) {
    // TODO: Decode PingRequest and send
    sendResult(request_id, false, "Not implemented");
}

// ================================================================
// Event Notifications
// ================================================================

void SerialProtocol::notifyMessageReceived(const char* from, const char* msg_id,
                                           const char* text, bool is_broadcast) {
    if (!events_enabled_) return;
    
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_message_received_tag;
    resp->request_id = 0;  // Async event
    
    lora_mesh_v1_MessageReceivedEvent* event = &resp->payload.message_received;
    strncpy(event->from, from, sizeof(event->from) - 1);
    strncpy(event->message_id, msg_id, sizeof(event->message_id) - 1);
    strncpy(event->text, text, sizeof(event->text) - 1);
    event->timestamp = millis();
    event->is_broadcast = is_broadcast;
    
    uint8_t buffer[384];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::notifyGPSReceived(const char* node_id, double lat, double lon,
                                       double alt, uint64_t timestamp) {
    if (!events_enabled_) return;
    
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_gps_received_tag;
    resp->request_id = 0;
    
    lora_mesh_v1_GPSReceivedEvent* event = &resp->payload.gps_received;
    strncpy(event->node_id, node_id, sizeof(event->node_id) - 1);
    event->has_position = true;
    event->position.latitude = lat;
    event->position.longitude = lon;
    event->position.altitude = alt;
    event->position.timestamp = timestamp;
    
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::notifyNeighborJoined(const char* node_id, int rssi) {
    if (!events_enabled_) return;
    
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_neighbor_changed_tag;
    resp->request_id = 0;
    
    lora_mesh_v1_NeighborChangedEvent* event = &resp->payload.neighbor_changed;
    event->change_type = lora_mesh_v1_NeighborChangedEvent_ChangeType_JOINED;
    event->has_neighbor = true;
    strncpy(event->neighbor.node_id, node_id, sizeof(event->neighbor.node_id) - 1);
    event->neighbor.rssi = rssi;
    
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::notifyNeighborLeft(const char* node_id) {
    if (!events_enabled_) return;
    
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_neighbor_changed_tag;
    resp->request_id = 0;
    
    lora_mesh_v1_NeighborChangedEvent* event = &resp->payload.neighbor_changed;
    event->change_type = lora_mesh_v1_NeighborChangedEvent_ChangeType_LEFT;
    event->has_neighbor = true;
    strncpy(event->neighbor.node_id, node_id, sizeof(event->neighbor.node_id) - 1);
    
    uint8_t buffer[128];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::notifyEmergency(const char* from, uint8_t type, const char* desc,
                                     double lat, double lon) {
    if (!events_enabled_) return;
    
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_emergency_received_tag;
    resp->request_id = 0;
    
    lora_mesh_v1_EmergencyReceivedEvent* event = &resp->payload.emergency_received;
    strncpy(event->from, from, sizeof(event->from) - 1);
    event->emergency_type = static_cast<lora_mesh_v1_EmergencyType>(type);
    strncpy(event->description, desc, sizeof(event->description) - 1);
    event->has_position = true;
    event->position.latitude = lat;
    event->position.longitude = lon;
    event->position.timestamp = millis();
    
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}

void SerialProtocol::logMessage(uint8_t level, const char* message) {
    if (!events_enabled_) return;
    
    lora_mesh_v1_SerialPacket packet = lora_mesh_v1_SerialPacket_init_zero;
    packet.packet_id = next_packet_id_++;
    packet.which_payload = lora_mesh_v1_SerialPacket_from_device_tag;
    
    lora_mesh_v1_FromDevice* resp = &packet.payload.from_device;
    resp->which_payload = lora_mesh_v1_FromDevice_log_tag;
    resp->request_id = 0;
    
    lora_mesh_v1_LogEvent* event = &resp->payload.log;
    event->level = static_cast<lora_mesh_v1_LogEvent_LogLevel>(level);
    strncpy(event->message, message, sizeof(event->message) - 1);
    event->timestamp = millis();
    
    uint8_t buffer[384];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if (pb_encode(&stream, lora_mesh_v1_SerialPacket_fields, &packet)) {
        sendPacket(buffer, stream.bytes_written);
    }
}
