/**
 * @file routing_engine.cpp
 * @brief AODV routing protocol implementation
 */

#include "routing_engine.h"
#include "config.h"
#include "gps_manager.h"
#include <cstring>
#include <algorithm>
#include <set>

// Rate limiting for content sync pushes (to avoid flooding on every HELLO)
static std::map<String, uint64_t> last_content_sync_push;
static const uint64_t CONTENT_SYNC_PUSH_INTERVAL_MS = 30000;  // Min 30 seconds between pushes to same node

RoutingEngine::RoutingEngine()
    : my_sequence_number(0)
    , mesh_version(0)
    , next_stream_id(1)
    , state_dirty(false)
    , last_save_time(0)
    , last_hello_time(0)
    , last_cleanup_time(0) {

    memset(&stats, 0, sizeof(stats));
    memset(&my_position, 0, sizeof(my_position));
}

bool RoutingEngine::init(const String& node_id) {
    my_node_id = node_id;
    memset(&my_position, 0, sizeof(my_position));

    routing_table.clear();
    neighbor_table.clear();
    pending_routes.clear();
    duplicate_cache.clear();
    message_history.clear();

    state_dirty = false;
    last_save_time = millis();

    // Load all persistent state from NVS
    loadState();

    LOG_I("Routing engine initialized for node: %s (mesh_version=%llu, seq=%u, neighbors=%d, routes=%d, messages=%d)",
          my_node_id.c_str(), mesh_version, my_sequence_number,
          neighbor_table.size(), routing_table.size(), message_history.size());
    return true;
}

void RoutingEngine::setMyPosition(const GPSCoordinate& position) {
    my_position = position;
    my_position.timestamp = getCurrentTime();
    LOG_D("Position updated: lat=%.6f, lon=%.6f", position.latitude, position.longitude);
}

// ================================================================
// Mesh State Version (Synchronization)
// ================================================================

uint64_t RoutingEngine::incrementMeshVersion() {
    mesh_version++;
    state_dirty = true;  // Will be saved periodically by maintain()

    LOG_D("Mesh version incremented to: %llu", mesh_version);
    return mesh_version;
}

bool RoutingEngine::updateMeshVersionIfNewer(uint64_t received_version) {
    if (received_version > mesh_version) {
        uint64_t old_version = mesh_version;
        mesh_version = received_version;
        state_dirty = true;

        LOG_I("Mesh version updated: %llu -> %llu (we were behind by %llu)",
              old_version, mesh_version, mesh_version - old_version);

        return true;
    }
    return false;
}

void RoutingEngine::requestStateSync(const String& target_node) {
    LOG_I("Requesting state sync from %s", target_node.c_str());

    // Build a targeted Network Discovery to request their state
    lora_mesh_v1_NetworkDiscoveryPayload discovery = lora_mesh_v1_NetworkDiscoveryPayload_init_zero;

    // Mark ourselves as originator
    discovery.has_originator = true;
    ProtobufHandler::copyString(discovery.originator.node_id,
                                sizeof(discovery.originator.node_id),
                                my_node_id);
    discovery.originator.sequence_number = getNextSequenceNumber();

    discovery.discovery_start_time = getCurrentTime();
    discovery.include_routing_info = true;  // Request routing info
    discovery.discovery_radius = 1;  // Direct request only

    // Encode and send
    uint8_t payload_buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buffer, sizeof(payload_buffer));

    if (!pb_encode(&stream, lora_mesh_v1_NetworkDiscoveryPayload_fields, &discovery)) {
        LOG_E("Failed to encode sync request");
        return;
    }

    // Send directly to the target node (unicast)
    std::vector<uint8_t> payload(payload_buffer, payload_buffer + stream.bytes_written);

    if (transmit_callback) {
        LoRaMessage msg = createMessage(target_node, MESSAGE_TYPE_NETWORK_DISCOVERY,
                                        payload.data(), payload.size(), PRIORITY_ROUTINE);
        stats.messages_sent++;
        transmit_callback(msg);
        LOG_I("State sync request sent to %s", target_node.c_str());
    }
}

// ================================================================
// Persistent Storage (DISABLED - RAM only for now)
// ================================================================

void RoutingEngine::loadState() {
    // NVS persistence disabled for simplicity
    // Mesh state is RAM-only and resets on boot
    mesh_version = 0;
    my_sequence_number = 0;
    LOG_I("Starting with fresh mesh state (NVS disabled)");
}

void RoutingEngine::saveState() {
    // NVS persistence disabled for simplicity
    // State remains in RAM only
    state_dirty = false;
}

void RoutingEngine::storeMessage(const LoRaMessage& msg) {
    // Add to front (most recent first)
    message_history.insert(message_history.begin(), msg);

    // Limit size
    while (message_history.size() > MAX_MESSAGE_HISTORY) {
        message_history.pop_back();
    }

    state_dirty = true;
}

void RoutingEngine::setTransmitCallback(std::function<bool(const LoRaMessage&)> callback) {
    transmit_callback = callback;
}

void RoutingEngine::setReceiveCallback(std::function<void(const LoRaMessage&)> callback) {
    receive_callback = callback;
}

// ================================================================
// Message Processing
// ================================================================

bool RoutingEngine::processMessage(const LoRaMessage& msg, int8_t rssi) {
    stats.messages_received++;

    // Ignore our own messages (radio echo/multipath)
    String source_id = msg.source_id;
    if (source_id == my_node_id) {
        LOG_D("Ignoring own message echo");
        return false;
    }

    // Update neighbor info from ANY message (even duplicates)
    // Receiving ANY message proves the node is alive and reachable
    updateNeighborFromMessage(msg, rssi);

    // Check for duplicate - message_id is now char[32]
    String msg_id = msg.message_id;
    if (isDuplicate(msg_id)) {
        LOG_D("Duplicate message dropped: %s", msg.message_id);
        stats.messages_dropped++;
        return false;
    }

    addToDuplicateCache(msg_id);

    // Check TTL
    if (msg.ttl == 0 || msg.hop_count >= MAX_HOP_COUNT) {
        LOG_D("Message TTL expired: %s", msg.message_id);
        stats.messages_dropped++;
        return false;
    }

    // Convert char arrays to String for comparison
    String dest_id = msg.destination_id;
    String dest_upper = dest_id;
    dest_upper.toUpperCase();

    // Process based on message type
    switch (msg.message_type) {
        case MESSAGE_TYPE_HELLO:
            processHello(msg, rssi);
            break;

        case MESSAGE_TYPE_ROUTE_REQUEST:
            processRouteRequest(msg, rssi);
            break;

        case MESSAGE_TYPE_ROUTE_REPLY:
            processRouteReply(msg, rssi);
            break;

        case MESSAGE_TYPE_ROUTE_ERROR:
            processRouteError(msg);
            break;

        case MESSAGE_TYPE_NETWORK_DISCOVERY:
            processNetworkDiscovery(msg, rssi);
            break;

        case MESSAGE_TYPE_LINK_STATE_ADVERTISEMENT:
            processLinkStateAdvertisement(msg, rssi);
            break;

        case MESSAGE_TYPE_FRAGMENT:
            // Fragment of a larger message - buffer and reassemble
            processFragment(msg, rssi);
            break;

        case lora_mesh_v1_MessageType_MESSAGE_TYPE_CONTENT_SYNC:
            // Bundled state sync from another node
            processContentSync(msg, rssi);
            break;

        case MESSAGE_TYPE_GPS_UPDATE:
        case MESSAGE_TYPE_TEXT_MESSAGE:
        case MESSAGE_TYPE_DATA:
        case MESSAGE_TYPE_EMERGENCY:
            // Broadcast messages: deliver locally
            // Rebroadcast only if we have neighbors the sender can't reach directly
            if (dest_upper == "BROADCAST" || dest_id.isEmpty()) {
                // Always deliver to local application
                deliverToApplication(msg);

                // Smart rebroadcast: check if we're a bridge to other nodes
                // Use routing table to see if we have neighbors the sender doesn't
                String sender_id = msg.source_id;
                if (shouldRebroadcast(sender_id) && msg.ttl > 1 && msg.hop_count < MAX_HOP_COUNT) {
                    LoRaMessage fwd_msg = msg;
                    fwd_msg.ttl--;
                    fwd_msg.hop_count++;
                    if (transmit_callback && transmit_callback(fwd_msg)) {
                        stats.messages_forwarded++;
                        LOG_D("Rebroadcast as bridge node for %s", sender_id.c_str());
                    }
                }
            }
            // Unicast to us: deliver locally
            else if (dest_id == my_node_id) {
                deliverToApplication(msg);
            }
            // Unicast to someone else: forward via routing table
            else {
                LoRaMessage fwd_msg = msg;
                if (!forwardMessage(fwd_msg)) {
                    LOG_W("Failed to forward message to: %s", msg.destination_id);
                    stats.messages_dropped++;
                }
            }
            break;

        default:
            LOG_W("Unknown message type: %d", msg.message_type);
            stats.messages_dropped++;
            return false;
    }

    return true;
}

bool RoutingEngine::sendMessage(const String& destination_id, MessageType msg_type,
                                const std::vector<uint8_t>& payload, MessagePriority priority) {

    // Check if payload needs fragmentation
    if (needsFragmentation(payload.size())) {
        LOG_I("Payload size %d exceeds limit, using fragmentation", payload.size());
        return sendFragmented(destination_id, msg_type, payload, priority);
    }

    // Increment mesh version for state-changing messages
    // GPS updates and text messages represent new mesh state that nodes should sync
    if (msg_type == MESSAGE_TYPE_GPS_UPDATE ||
        msg_type == MESSAGE_TYPE_TEXT_MESSAGE ||
        msg_type == MESSAGE_TYPE_EMERGENCY) {
        incrementMeshVersion();
    }

    // Create message - convert vector to buffer+length
    LoRaMessage msg = createMessage(destination_id, msg_type, payload.data(), payload.size(), priority);

    // Store OUR sent messages in history (for text, GPS, emergency - not routing control)
    if (msg_type == MESSAGE_TYPE_TEXT_MESSAGE ||
        msg_type == MESSAGE_TYPE_GPS_UPDATE ||
        msg_type == MESSAGE_TYPE_EMERGENCY) {
        storeMessage(msg);
        LOG_D("Stored sent message in history: %s", msg.message_id);
    }

    // If broadcast or we have a route, send immediately
    // Use case-insensitive comparison for "BROADCAST"
    String dest_upper = destination_id;
    dest_upper.toUpperCase();
    if (dest_upper == "BROADCAST") {
        if (transmit_callback) {
            stats.messages_sent++;
            return transmit_callback(msg);
        }
        return false;
    }

    // Check for existing route
    RouteEntry* route = findRoute(destination_id);
    if (route && route->is_valid) {
        if (transmit_callback) {
            stats.messages_sent++;
            return transmit_callback(msg);
        }
        return false;
    }

    // No route - initiate discovery and queue message
    LOG_I("No route to %s, initiating discovery", destination_id.c_str());

    // Add to pending routes
    if (pending_routes.find(destination_id) == pending_routes.end()) {
        PendingRoute pending;
        pending.destination_id = destination_id;
        pending.request_time = getCurrentTime();
        pending.retry_count = 0;
        pending_routes[destination_id] = pending;

        initiateRouteDiscovery(destination_id);
    }

    // Queue message
    pending_routes[destination_id].queued_messages.push_back(msg);

    return true;
}

// ================================================================
// HELLO Beacon Processing
// ================================================================

void RoutingEngine::sendHelloBeacon() {
    // Create HelloPayload using the actual nanopb type
    HelloPayload hello = ProtobufHandler::createHelloPayload();

    // Set up node_info (nested struct) - now static arrays
    hello.has_node_info = true;
    ProtobufHandler::copyString(hello.node_info.node_id, sizeof(hello.node_info.node_id), my_node_id);
    hello.node_info.battery_level = 100; // TODO: Read actual battery level 0-100
    hello.node_info.sequence_number = getNextSequenceNumber();
    hello.node_info.last_seen = ProtobufHandler::getCurrentTimestamp();

    // Include GPS position for geographic routing (per pseudocode)
    if (hasValidPosition()) {
        hello.node_info.has_position = true;
        hello.node_info.position = my_position;
    }

    // Set hello-specific fields
    hello.hello_sequence = hello.node_info.sequence_number;
    hello.battery_level = 1.0f; // 0.0-1.0 float (TODO: read actual)
    hello.neighbor_count = neighbor_table.size();
    hello.mesh_version = mesh_version;  // Include mesh state version for sync

    uint8_t payload_buf[256];
    size_t payload_len = ProtobufHandler::encodePayload(hello, payload_buf, sizeof(payload_buf));

    if (payload_len > 0) {
        LoRaMessage msg = createMessage("", MESSAGE_TYPE_HELLO, payload_buf, payload_len, PRIORITY_ROUTINE);

        if (transmit_callback) {
            stats.messages_sent++;
            transmit_callback(msg);
            LOG_D("HELLO beacon sent (seq=%u, size=%u, hasGPS=%d)",
                  hello.hello_sequence, payload_len, hello.node_info.has_position);
        }
    }
}

void RoutingEngine::initiateNetworkJoin() {
    // Per pseudocode: New node joining network sends discovery broadcast
    // and optionally RREQ with JOIN flag
    LOG_I("Initiating network join - sending discovery and HELLO");

    // First, send a HELLO beacon so others know about us
    sendHelloBeacon();

    // Small delay to let HELLO propagate
    delay(100);

    // Then send network discovery to learn topology
    sendNetworkDiscovery();
}

void RoutingEngine::sendNetworkDiscovery() {
    // Create NetworkDiscoveryPayload per pseudocode patterns
    lora_mesh_v1_NetworkDiscoveryPayload discovery = lora_mesh_v1_NetworkDiscoveryPayload_init_zero;

    // Set ourselves as originator
    discovery.has_originator = true;
    ProtobufHandler::copyString(discovery.originator.node_id,
                                sizeof(discovery.originator.node_id),
                                my_node_id);
    discovery.originator.battery_level = 100;  // TODO: actual battery
    discovery.originator.sequence_number = getNextSequenceNumber();
    discovery.originator.last_seen = getCurrentTime();

    discovery.discovery_start_time = getCurrentTime();
    discovery.include_routing_info = true;

    // Include any neighbors we already know about (may be empty on first boot)
    discovery.discovered_nodes_count = 0;
    for (const auto& pair : neighbor_table) {
        if (discovery.discovered_nodes_count >= 16) break;
        if (pair.second.is_active) {
            lora_mesh_v1_NodeInfo& node = discovery.discovered_nodes[discovery.discovered_nodes_count];
            ProtobufHandler::copyString(node.node_id, sizeof(node.node_id), pair.first);
            node.rssi = pair.second.last_rssi;
            node.last_seen = pair.second.last_seen_time;
            node.battery_level = pair.second.battery_level;
            if (pair.second.last_position.timestamp > 0) {
                node.has_position = true;
                node.position = pair.second.last_position;
            }
            discovery.discovered_nodes_count++;
        }
    }

    // Encode and broadcast
    uint8_t payload_buf[512];
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (pb_encode(&stream, lora_mesh_v1_NetworkDiscoveryPayload_fields, &discovery)) {
        LoRaMessage msg = createMessage("", MESSAGE_TYPE_NETWORK_DISCOVERY,
                                        payload_buf, stream.bytes_written, PRIORITY_TACTICAL);

        if (transmit_callback) {
            stats.messages_sent++;
            transmit_callback(msg);
            LOG_I("Network discovery sent (%d known neighbors)", discovery.discovered_nodes_count);
        }
    } else {
        LOG_E("Failed to encode network discovery");
    }
}

void RoutingEngine::processHello(const LoRaMessage& msg, int8_t rssi) {
    // Decode using actual nanopb type
    HelloPayload hello = lora_mesh_v1_HelloPayload_init_zero;

    // payload is now PB_BYTES_ARRAY_T - access via .bytes and .size
    if (!ProtobufHandler::decodePayload(msg.payload.bytes, msg.payload.size, hello)) {
        LOG_E("Failed to decode HELLO from: %s", msg.source_id);
        return;
    }

    // node_id is now char[16] - convert to String
    String node_id = strlen(hello.node_info.node_id) > 0 ?
                     String(hello.node_info.node_id) : String(msg.source_id);

    // Helper lambda to check if we should push content (rate limited)
    auto shouldPushContent = [&]() -> bool {
        uint64_t now = getCurrentTime();
        auto it = last_content_sync_push.find(node_id);
        if (it == last_content_sync_push.end()) {
            return true;  // Never pushed to this node
        }
        return (now - it->second) >= CONTENT_SYNC_PUSH_INTERVAL_MS;
    };

    // Helper lambda to record push time
    auto recordPush = [&]() {
        last_content_sync_push[node_id] = getCurrentTime();
    };

    // Check mesh version for sync
    if (hello.mesh_version > 0) {
        if (hello.mesh_version > mesh_version) {
            // Sender has newer version - we're behind, request sync
            updateMeshVersionIfNewer(hello.mesh_version);
            LOG_I("Detected newer mesh version from %s (%llu > %llu) - requesting state sync",
                  node_id.c_str(), hello.mesh_version, mesh_version);
            requestStateSync(node_id);
        } else if (hello.mesh_version < mesh_version) {
            // We have newer version - push our content to the neighbor (rate limited)
            if (shouldPushContent()) {
                LOG_I("Detected older mesh version on %s (%llu < %llu) - pushing content sync",
                      node_id.c_str(), hello.mesh_version, mesh_version);
                // Small delay to avoid collision with their potential transmissions
                delay(100 + (esp_random() % 150));

                // Send a HELLO first so they update their routing table with our info
                sendHelloBeacon();
                delay(100);

                // Then push the content they're missing
                sendContentSync(node_id, hello.mesh_version);  // Differential sync based on their version
                recordPush();
            } else {
                LOG_D("Skipping content push to %s (rate limited)", node_id.c_str());
            }
        }
    } else {
        // Node sent mesh_version=0, likely just booted - push our content (rate limited)
        if (shouldPushContent()) {
            LOG_I("Node %s has mesh_version=0 (fresh boot) - pushing content sync", node_id.c_str());
            delay(100 + (esp_random() % 150));

            // Send a HELLO first so they update their routing table with our info
            sendHelloBeacon();
            delay(100);

            // Then push the content they're missing
            sendContentSync(node_id, 0);  // Full sync for fresh boot
            recordPush();
        } else {
            LOG_D("Skipping content push to %s (rate limited)", node_id.c_str());
        }
    }

    // Check if this is a NEW neighbor (not just an update)
    auto neighbor_it = neighbor_table.find(node_id);
    bool is_new_neighbor = (neighbor_it == neighbor_table.end() || !neighbor_it->second.is_active);

    // Update neighbor table
    NeighborEntry& neighbor = neighbor_table[node_id];
    neighbor.node_id = node_id;
    neighbor.last_seen_time = getCurrentTime();
    neighbor.last_rssi = rssi;
    neighbor.battery_level = (uint8_t)(hello.battery_level * 100); // Convert 0.0-1.0 to 0-100
    neighbor.sequence_number = hello.hello_sequence;
    neighbor.is_active = true;
    state_dirty = true;  // Mark for persistence

    // NOTE: We do NOT increment mesh_version when discovering a neighbor via HELLO
    // Reason: If 5 nodes hear a new node's HELLO, all 5 would increment and flood the network
    // Version only increments when we generate NEW content (GPS, messages) or discover multi-hop routes
    // The designated responders (best RSSI + edge) will handle onboarding the new node
    if (is_new_neighbor) {
        LOG_I("New neighbor discovered: %s (their v=%llu, our v=%llu)",
              node_id.c_str(), hello.mesh_version, mesh_version);
    }

    // Copy position if present
    if (hello.node_info.has_position) {
        neighbor.last_position = hello.node_info.position;
    }

    // Update route (direct neighbor, 1 hop)
    updateRoute(node_id, node_id, 1, hello.hello_sequence, rssi);

    LOG_D("HELLO from %s (RSSI=%d, battery=%.0f%%, mesh_v=%llu)",
          node_id.c_str(), rssi, hello.battery_level * 100, hello.mesh_version);
}

// ================================================================
// Network Discovery - Dual Responder Strategy
// Best RSSI neighbor + Farthest reachable neighbor both respond
// ================================================================

void RoutingEngine::processNetworkDiscovery(const LoRaMessage& msg, int8_t rssi) {
    // Decode the discovery payload
    lora_mesh_v1_NetworkDiscoveryPayload discovery = lora_mesh_v1_NetworkDiscoveryPayload_init_zero;

    pb_istream_t stream = pb_istream_from_buffer(msg.payload.bytes, msg.payload.size);
    if (!pb_decode(&stream, lora_mesh_v1_NetworkDiscoveryPayload_fields, &discovery)) {
        LOG_E("Failed to decode NetworkDiscovery from: %s", msg.source_id);
        return;
    }

    String originator_id = msg.source_id;
    LOG_I("Network discovery from %s (RSSI=%d)", originator_id.c_str(), rssi);

    // Update neighbor table from the discovery message
    NeighborEntry& neighbor = neighbor_table[originator_id];
    neighbor.node_id = originator_id;
    neighbor.last_seen_time = getCurrentTime();
    neighbor.last_rssi = rssi;
    neighbor.is_active = true;

    // Update route to originator (direct neighbor, 1 hop)
    updateRoute(originator_id, originator_id, 1, msg.sequence_number, rssi);

    // Learn from discovered_nodes in the message
    for (pb_size_t i = 0; i < discovery.discovered_nodes_count; i++) {
        String node_id = discovery.discovered_nodes[i].node_id;
        if (node_id.length() > 0 && node_id != my_node_id) {
            // Add 2-hop route through originator
            updateRoute(node_id, originator_id, 2,
                       discovery.discovered_nodes[i].sequence_number,
                       discovery.discovered_nodes[i].rssi);
            LOG_D("Learned route to %s via %s (2 hops)", node_id.c_str(), originator_id.c_str());
        }
    }

    // Dual responder strategy: only respond if we're best or worst RSSI
    int response_type = shouldRespondToDiscovery(rssi);
    if (response_type > 0) {
        // Add random backoff to avoid collisions
        // Best RSSI (type 1) gets short delay, worst RSSI (type 2) gets longer delay
        uint32_t backoff = (response_type == 1) ?
                           (50 + esp_random() % 100) :   // 50-150ms for best
                           (200 + esp_random() % 200);   // 200-400ms for farthest
        delay(backoff);

        sendNeighborUpdate(originator_id, response_type == 2);
        LOG_I("Responded to discovery as %s responder",
              response_type == 1 ? "best-RSSI" : "edge-node");

        // If this is a sync request (include_routing_info=true), also send content
        // This shares missed text messages and GPS positions with the requesting node
        if (discovery.include_routing_info) {
            delay(100);  // Brief delay before content sync
            sendContentSync(originator_id);
        }
    }
}

int RoutingEngine::shouldRespondToDiscovery(int8_t incoming_rssi) {
    // We need at least 2 neighbors to make best/worst decision meaningful
    if (neighbor_table.size() < 1) {
        return 1;  // We're the only one, respond as best
    }

    // Find best and worst RSSI among active neighbors
    int8_t best_rssi = -128;
    int8_t worst_rssi = 0;

    for (const auto& pair : neighbor_table) {
        const NeighborEntry& n = pair.second;
        if (n.is_active && !isNeighborBlacklisted(n.node_id)) {
            if (n.last_rssi > best_rssi) best_rssi = n.last_rssi;
            if (n.last_rssi < worst_rssi) worst_rssi = n.last_rssi;
        }
    }

    // Check if our received RSSI is best (strongest signal = closest)
    // We use a tolerance of 6dB
    if (incoming_rssi >= best_rssi - 6) {
        return 1;  // Best RSSI responder
    }

    // Check if our received RSSI is worst (weakest signal = farthest)
    // Must still be above usable threshold (-110 dBm typical for LoRa)
    if (incoming_rssi <= worst_rssi + 6 && incoming_rssi > -110) {
        return 2;  // Farthest/edge responder
    }

    return 0;  // Middle of the pack, don't respond
}

void RoutingEngine::sendNeighborUpdate(const String& target_id, bool is_edge_responder) {
    // Build NetworkDiscoveryPayload with our neighbor/routing info
    lora_mesh_v1_NetworkDiscoveryPayload response = lora_mesh_v1_NetworkDiscoveryPayload_init_zero;

    // Set ourselves as originator
    response.has_originator = true;
    ProtobufHandler::copyString(response.originator.node_id,
                                sizeof(response.originator.node_id),
                                my_node_id);
    response.originator.battery_level = 100;  // TODO: actual battery
    response.originator.sequence_number = getNextSequenceNumber();
    response.originator.last_seen = getCurrentTime();

    response.discovery_start_time = getCurrentTime();
    response.include_routing_info = true;

    // Pack our known nodes (neighbors + routes) into discovered_nodes
    // Prioritize differently based on responder type:
    // - Best RSSI: prioritize close/strong neighbors (local topology)
    // - Edge responder: prioritize distant/weak routes (network extent)

    std::vector<std::pair<String, int8_t>> nodes_to_share;

    // Gather all known nodes with their RSSI
    for (const auto& pair : neighbor_table) {
        if (pair.second.is_active && pair.first != target_id) {
            nodes_to_share.push_back({pair.first, pair.second.last_rssi});
        }
    }

    // Also add routes (2+ hop destinations)
    for (const auto& pair : routing_table) {
        if (pair.second.is_valid && pair.first != target_id) {
            // Avoid duplicates from neighbor table
            bool found = false;
            for (const auto& n : nodes_to_share) {
                if (n.first == pair.first) { found = true; break; }
            }
            if (!found) {
                nodes_to_share.push_back({pair.first, pair.second.rssi});
            }
        }
    }

    // Sort based on responder type
    // Define the pair type explicitly for C++11 compatibility
    typedef std::pair<String, int8_t> NodeRssiPair;

    if (is_edge_responder) {
        // Edge responder: sort by WORST RSSI first (farthest nodes)
        std::sort(nodes_to_share.begin(), nodes_to_share.end(),
                  [](const NodeRssiPair& a, const NodeRssiPair& b) { return a.second < b.second; });
    } else {
        // Best RSSI responder: sort by BEST RSSI first (closest nodes)
        std::sort(nodes_to_share.begin(), nodes_to_share.end(),
                  [](const NodeRssiPair& a, const NodeRssiPair& b) { return a.second > b.second; });
    }

    // Pack up to 16 nodes
    response.discovered_nodes_count = 0;
    for (size_t i = 0; i < nodes_to_share.size() && response.discovered_nodes_count < 16; i++) {
        lora_mesh_v1_NodeInfo& node = response.discovered_nodes[response.discovered_nodes_count];
        ProtobufHandler::copyString(node.node_id, sizeof(node.node_id),
                                    nodes_to_share[i].first);
        node.rssi = nodes_to_share[i].second;
        node.last_seen = getCurrentTime();

        // Add position if we have it from neighbor table
        auto it = neighbor_table.find(nodes_to_share[i].first);
        if (it != neighbor_table.end() && it->second.last_position.timestamp > 0) {
            node.has_position = true;
            node.position = it->second.last_position;
        }

        response.discovered_nodes_count++;
    }

    // Encode and send
    uint8_t payload_buf[512];  // NetworkDiscoveryPayload can be large
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (pb_encode(&stream, lora_mesh_v1_NetworkDiscoveryPayload_fields, &response)) {
        LoRaMessage msg = createMessage(target_id, MESSAGE_TYPE_NETWORK_DISCOVERY,
                                        payload_buf, stream.bytes_written, PRIORITY_ROUTINE);
        if (transmit_callback) {
            stats.messages_sent++;
            transmit_callback(msg);
            LOG_I("Sent neighbor update to %s (%d nodes, %s)",
                  target_id.c_str(), response.discovered_nodes_count,
                  is_edge_responder ? "edge" : "local");
        }
    } else {
        LOG_E("Failed to encode neighbor update");
    }

    // Edge responder: Also notify nodes deeper in the mesh about the newcomer
    // This propagates awareness of the joining node to multi-hop distant nodes
    if (is_edge_responder) {
        broadcastNewcomerAnnouncement(target_id);
    }
}

void RoutingEngine::sendContentSync(const String& target_id, uint64_t neighbor_mesh_version) {
    // BUNDLED SYNC: Send positions + messages in a single fragmented stream
    // This replaces individual packets with delays, reducing overhead and collisions

    if (!transmit_callback) {
        LOG_E("No transmit callback set");
        return;
    }

    // Calculate how much we're ahead
    uint64_t version_diff = (neighbor_mesh_version > 0) ? (mesh_version - neighbor_mesh_version) : mesh_version;
    bool full_sync = (neighbor_mesh_version == 0) || (version_diff > 20);

    LOG_I("Content sync to %s (their v=%llu, ours v=%llu, diff=%llu, %s)",
          target_id.c_str(), neighbor_mesh_version, mesh_version, version_diff,
          full_sync ? "full" : "differential");

    // Build ContentSyncPayload
    lora_mesh_v1_ContentSyncPayload sync = lora_mesh_v1_ContentSyncPayload_init_zero;
    sync.mesh_version = mesh_version;

    // 1. Add our own position first (highest priority)
    if (hasValidPosition()) {
        lora_mesh_v1_SyncPosition& pos = sync.positions[sync.positions_count++];
        ProtobufHandler::copyString(pos.node_id, sizeof(pos.node_id), my_node_id);
        pos.has_position = true;
        pos.position = my_position;
        pos.timestamp = my_position.timestamp;
    }

    // 2. Add neighbor positions (up to limit, most recent first)
    std::vector<std::pair<String, const NeighborEntry*>> nodes_with_gps;
    for (const auto& pair : neighbor_table) {
        if (pair.second.is_active && pair.second.last_position.timestamp > 0) {
            nodes_with_gps.push_back({pair.first, &pair.second});
        }
    }
    std::sort(nodes_with_gps.begin(), nodes_with_gps.end(),
              [](const std::pair<String, const NeighborEntry*>& a,
                 const std::pair<String, const NeighborEntry*>& b) {
                  return a.second->last_position.timestamp > b.second->last_position.timestamp;
              });

    size_t max_positions = full_sync ? 9 : std::min((size_t)version_diff + 2, (size_t)5);
    for (size_t i = 0; i < nodes_with_gps.size() && sync.positions_count < max_positions; i++) {
        lora_mesh_v1_SyncPosition& pos = sync.positions[sync.positions_count++];
        ProtobufHandler::copyString(pos.node_id, sizeof(pos.node_id), nodes_with_gps[i].first);
        pos.has_position = true;
        pos.position = nodes_with_gps[i].second->last_position;
        pos.timestamp = nodes_with_gps[i].second->last_position.timestamp;
    }

    // 3. Add text messages (decode from stored messages)
    size_t max_messages = full_sync ? 5 : std::min((size_t)version_diff, (size_t)3);
    for (const auto& stored_msg : message_history) {
        if (sync.messages_count >= max_messages) break;

        if (stored_msg.message_type == MESSAGE_TYPE_TEXT_MESSAGE ||
            stored_msg.message_type == MESSAGE_TYPE_EMERGENCY) {

            // Decode text from payload
            lora_mesh_v1_TextMessagePayload text_payload = lora_mesh_v1_TextMessagePayload_init_zero;
            pb_istream_t stream = pb_istream_from_buffer(stored_msg.payload.bytes, stored_msg.payload.size);
            if (pb_decode(&stream, lora_mesh_v1_TextMessagePayload_fields, &text_payload)) {
                lora_mesh_v1_SyncTextMessage& msg = sync.messages[sync.messages_count++];
                ProtobufHandler::copyString(msg.message_id, sizeof(msg.message_id), stored_msg.message_id);
                ProtobufHandler::copyString(msg.source_id, sizeof(msg.source_id), stored_msg.source_id);
                ProtobufHandler::copyString(msg.text, sizeof(msg.text), text_payload.text);
                msg.timestamp = stored_msg.timestamp;
            }
        }
    }

    // 4. Add roster - active nodes that DON'T have GPS positions (already included above)
    // This ensures GPS-less nodes still appear in team roster
    std::set<String> nodes_with_positions;
    nodes_with_positions.insert(my_node_id);  // We're in positions if we have GPS
    for (size_t i = 0; i < sync.positions_count; i++) {
        nodes_with_positions.insert(sync.positions[i].node_id);
    }

    // Add active neighbors without GPS to roster
    for (const auto& pair : neighbor_table) {
        if (sync.roster_count >= 16) break;
        if (pair.second.is_active && nodes_with_positions.find(pair.first) == nodes_with_positions.end()) {
            ProtobufHandler::copyString(sync.roster[sync.roster_count], 16, pair.first);
            sync.roster_count++;
        }
    }

    // Add ourselves to roster if we don't have GPS
    if (!hasValidPosition() && sync.roster_count < 16) {
        ProtobufHandler::copyString(sync.roster[sync.roster_count], 16, my_node_id);
        sync.roster_count++;
    }

    LOG_I("ContentSync payload: %d positions, %d messages, %d roster entries",
          sync.positions_count, sync.messages_count, sync.roster_count);

    // Encode the sync payload
    uint8_t payload_buf[1024];  // Large enough for full sync
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (!pb_encode(&stream, lora_mesh_v1_ContentSyncPayload_fields, &sync)) {
        LOG_E("Failed to encode ContentSyncPayload");
        return;
    }

    // Send (fragmented if needed)
    std::vector<uint8_t> payload(payload_buf, payload_buf + stream.bytes_written);

    if (needsFragmentation(payload.size())) {
        LOG_I("ContentSync %d bytes - fragmenting", payload.size());
        sendFragmented(target_id, lora_mesh_v1_MessageType_MESSAGE_TYPE_CONTENT_SYNC, payload, PRIORITY_ROUTINE);
    } else {
        LOG_I("ContentSync %d bytes - single packet", payload.size());
        LoRaMessage msg = createMessage(target_id, lora_mesh_v1_MessageType_MESSAGE_TYPE_CONTENT_SYNC,
                                        payload_buf, stream.bytes_written, PRIORITY_ROUTINE);
        stats.messages_sent++;
        transmit_callback(msg);
    }
}

void RoutingEngine::processContentSync(const LoRaMessage& msg, int8_t rssi) {
    // Decode the bundled sync payload
    lora_mesh_v1_ContentSyncPayload sync = lora_mesh_v1_ContentSyncPayload_init_zero;

    pb_istream_t stream = pb_istream_from_buffer(msg.payload.bytes, msg.payload.size);
    if (!pb_decode(&stream, lora_mesh_v1_ContentSyncPayload_fields, &sync)) {
        LOG_E("Failed to decode ContentSyncPayload from %s", msg.source_id);
        return;
    }

    LOG_I("ContentSync from %s: %d positions, %d messages, %d roster (mesh_v=%llu)",
          msg.source_id, sync.positions_count, sync.messages_count, sync.roster_count, sync.mesh_version);

    // Process positions - update neighbor table with received GPS data
    for (size_t i = 0; i < sync.positions_count; i++) {
        const lora_mesh_v1_SyncPosition& pos = sync.positions[i];
        String node_id = pos.node_id;

        if (!pos.has_position) continue;

        // Update our neighbor/position table
        if (neighbor_table.find(node_id) != neighbor_table.end()) {
            NeighborEntry& neighbor = neighbor_table[node_id];
            // Only update if newer
            if (pos.timestamp > neighbor.last_position.timestamp) {
                neighbor.last_position = pos.position;
                LOG_D("Updated position for %s from sync", node_id.c_str());
            }
        } else {
            // New node we didn't know about - add as inactive (no direct link)
            NeighborEntry new_neighbor;
            new_neighbor.is_active = false;  // We didn't hear them directly
            new_neighbor.last_position = pos.position;
            new_neighbor.last_rssi = -100;  // Unknown
            new_neighbor.last_seen_time = getCurrentTime();
            neighbor_table[node_id] = new_neighbor;
            LOG_D("Learned about %s from sync (GPS only)", node_id.c_str());
        }
    }

    // Process text messages - deliver to local application
    for (size_t i = 0; i < sync.messages_count; i++) {
        const lora_mesh_v1_SyncTextMessage& text = sync.messages[i];
        String message_id = text.message_id;

        // Check duplicate cache
        if (isDuplicate(message_id)) {
            LOG_D("Skipping duplicate synced message %s", message_id.c_str());
            continue;
        }

        // Create a LoRaMessage to deliver to application
        LoRaMessage synced_msg = lora_mesh_v1_LoRaMessage_init_zero;
        ProtobufHandler::copyString(synced_msg.message_id, sizeof(synced_msg.message_id), message_id);
        ProtobufHandler::copyString(synced_msg.source_id, sizeof(synced_msg.source_id), text.source_id);
        synced_msg.message_type = MESSAGE_TYPE_TEXT_MESSAGE;
        synced_msg.timestamp = text.timestamp;

        // Encode text into payload
        lora_mesh_v1_TextMessagePayload text_payload = lora_mesh_v1_TextMessagePayload_init_zero;
        ProtobufHandler::copyString(text_payload.text, sizeof(text_payload.text), text.text);

        pb_ostream_t out_stream = pb_ostream_from_buffer(synced_msg.payload.bytes, sizeof(synced_msg.payload.bytes));
        if (pb_encode(&out_stream, lora_mesh_v1_TextMessagePayload_fields, &text_payload)) {
            synced_msg.payload.size = out_stream.bytes_written;
            deliverToApplication(synced_msg);
            addToDuplicateCache(message_id);
            LOG_D("Delivered synced message from %s", text.source_id);
        }
    }

    // Process roster - add GPS-less nodes to our known roster
    for (size_t i = 0; i < sync.roster_count; i++) {
        String node_id = sync.roster[i];

        // Add to neighbor table as inactive if we don't know them
        if (neighbor_table.find(node_id) == neighbor_table.end()) {
            NeighborEntry new_neighbor;
            new_neighbor.node_id = node_id;
            new_neighbor.is_active = false;  // We didn't hear them directly
            new_neighbor.last_rssi = -100;   // Unknown
            new_neighbor.last_seen_time = getCurrentTime();
            memset(&new_neighbor.last_position, 0, sizeof(new_neighbor.last_position));  // No GPS
            neighbor_table[node_id] = new_neighbor;
            LOG_D("Learned about GPS-less node %s from roster sync", node_id.c_str());
        }
    }

    LOG_I("ContentSync processed: %d positions, %d messages, %d roster",
          sync.positions_count, sync.messages_count, sync.roster_count);

    // Update mesh version if sender is ahead
    if (sync.mesh_version > mesh_version) {
        LOG_I("Mesh version updated from sync: %llu -> %llu", mesh_version, sync.mesh_version);
        mesh_version = sync.mesh_version;
    }
}

// ================================================================
// Link State Advertisement - Proactive Topology Maintenance
// Per pseudocode: LSA floods to maintain network-wide topology awareness
// ================================================================

void RoutingEngine::broadcastNewcomerAnnouncement(const String& newcomer_id) {
    // Create a Link State Advertisement to propagate newcomer info
    // This allows nodes multiple hops away to learn about the new node
    lora_mesh_v1_LinkStateAdvertisementPayload lsa = lora_mesh_v1_LinkStateAdvertisementPayload_init_zero;

    // Set ourselves as originator (we are the edge node relaying this info)
    ProtobufHandler::copyString(lsa.originator_id, sizeof(lsa.originator_id), my_node_id);
    lsa.sequence_number = getNextSequenceNumber();
    lsa.lsa_timestamp = getCurrentTime();
    lsa.lsa_lifetime_ms = ROUTE_LIFETIME_MS;

    // Include our GPS position for geographic routing (per pseudocode)
    if (hasValidPosition()) {
        lsa.has_gps_position = true;
        lsa.gps_position = my_position;
    }

    // Add the newcomer as our first neighbor entry
    lsa.neighbors_count = 0;
    lora_mesh_v1_NeighborInfo& newcomer_info = lsa.neighbors[lsa.neighbors_count++];
    ProtobufHandler::copyString(newcomer_info.neighbor_id, sizeof(newcomer_info.neighbor_id), newcomer_id);
    newcomer_info.rssi = -50;  // We just received discovery from them, good signal
    newcomer_info.distance_meters = 0;  // Unknown

    // Include our other neighbors so distant nodes learn full local topology
    for (const auto& pair : neighbor_table) {
        if (lsa.neighbors_count >= 16) break;  // Max 16 neighbors per LSA
        if (pair.second.is_active && pair.first != newcomer_id && !isNeighborBlacklisted(pair.first)) {
            lora_mesh_v1_NeighborInfo& ni = lsa.neighbors[lsa.neighbors_count++];
            ProtobufHandler::copyString(ni.neighbor_id, sizeof(ni.neighbor_id), pair.first);
            ni.rssi = pair.second.last_rssi;
            // Calculate distance if we have both positions
            if (hasValidPosition() && pair.second.last_position.timestamp > 0) {
                ni.distance_meters = (float)GPSManager::calculateDistance(my_position, pair.second.last_position);
            } else {
                ni.distance_meters = 0;
            }
        }
    }

    // Encode and broadcast the LSA
    uint8_t payload_buf[512];
    pb_ostream_t stream = pb_ostream_from_buffer(payload_buf, sizeof(payload_buf));
    if (pb_encode(&stream, lora_mesh_v1_LinkStateAdvertisementPayload_fields, &lsa)) {
        // Broadcast with reduced TTL to prevent flooding too far
        // Use TTL of 2-3 hops to propagate to nearby multi-hop nodes
        LoRaMessage msg = createMessage("", MESSAGE_TYPE_LINK_STATE_ADVERTISEMENT,
                                        payload_buf, stream.bytes_written, PRIORITY_TACTICAL);
        msg.ttl = 3;  // Limit propagation depth

        if (transmit_callback) {
            stats.messages_sent++;
            transmit_callback(msg);
            LOG_I("Broadcasted LSA with newcomer %s (%d neighbors, TTL=3)",
                  newcomer_id.c_str(), lsa.neighbors_count);
        }
    } else {
        LOG_E("Failed to encode LSA for newcomer announcement");
    }
}

void RoutingEngine::processLinkStateAdvertisement(const LoRaMessage& msg, int8_t rssi) {
    // Decode the LSA payload
    lora_mesh_v1_LinkStateAdvertisementPayload lsa = lora_mesh_v1_LinkStateAdvertisementPayload_init_zero;

    pb_istream_t stream = pb_istream_from_buffer(msg.payload.bytes, msg.payload.size);
    if (!pb_decode(&stream, lora_mesh_v1_LinkStateAdvertisementPayload_fields, &lsa)) {
        LOG_E("Failed to decode LSA from: %s", msg.source_id);
        return;
    }

    String originator_id = lsa.originator_id;
    LOG_I("LSA from %s (seq=%u, %d neighbors)",
          originator_id.c_str(), lsa.sequence_number, lsa.neighbors_count);

    // Update route to the LSA originator (direct if hop_count=0, otherwise multi-hop)
    if (msg.hop_count == 0) {
        // Direct neighbor - 1 hop
        updateRoute(originator_id, originator_id, 1, lsa.sequence_number, rssi);

        // Update neighbor table
        NeighborEntry& neighbor = neighbor_table[originator_id];
        neighbor.node_id = originator_id;
        neighbor.last_seen_time = getCurrentTime();
        neighbor.last_rssi = rssi;
        neighbor.sequence_number = lsa.sequence_number;
        neighbor.is_active = true;

        if (lsa.has_gps_position) {
            neighbor.last_position = lsa.gps_position;
        }
    } else {
        // Multi-hop - route via whoever forwarded this to us
        String forwarder = msg.source_id;
        updateRoute(originator_id, forwarder, msg.hop_count + 1, lsa.sequence_number, rssi);
    }

    // Learn 2-hop topology from the LSA neighbor list
    // Per pseudocode: Build network graph from LSA neighbor information
    for (pb_size_t i = 0; i < lsa.neighbors_count; i++) {
        String neighbor_id = lsa.neighbors[i].neighbor_id;
        if (neighbor_id.length() > 0 && neighbor_id != my_node_id) {
            // Check if we already have a direct route to this node
            RouteEntry* existing = findRoute(neighbor_id);
            if (!existing || !existing->is_valid || existing->hop_count > msg.hop_count + 2) {
                // Add/update 2-hop route through LSA originator
                updateRoute(neighbor_id, originator_id, msg.hop_count + 2,
                           lsa.sequence_number, lsa.neighbors[i].rssi);
                LOG_D("Learned 2-hop route to %s via %s from LSA",
                      neighbor_id.c_str(), originator_id.c_str());
            }
        }
    }

    // Forward LSA if TTL allows (controlled flooding for link state)
    // Per pseudocode: LSAs are flooded to maintain topology awareness
    if (msg.ttl > 1 && msg.hop_count < MAX_HOP_COUNT) {
        LoRaMessage fwd_msg = msg;
        fwd_msg.hop_count++;
        fwd_msg.ttl--;

        // Add small jitter to avoid collisions per pseudocode collision avoidance
        delay(10 + esp_random() % 40);  // 10-50ms jitter

        if (transmit_callback) {
            transmit_callback(fwd_msg);
            stats.messages_forwarded++;
            LOG_D("Forwarded LSA from %s (TTL=%u)", originator_id.c_str(), fwd_msg.ttl);
        }
    }
}

void RoutingEngine::updateNeighborFromMessage(const LoRaMessage& msg, int8_t rssi) {
    // Any message from a node proves it's alive and reachable
    // This keeps neighbors fresh without needing frequent HELLO beacons

    // Only update if this is from a direct neighbor (hop_count == 0 or source == last hop)
    if (msg.hop_count == 0) {
        // source_id is now char[16] - convert to String for map key
        String source_id = msg.source_id;
        NeighborEntry& neighbor = neighbor_table[source_id];
        neighbor.node_id = source_id;
        neighbor.last_seen_time = getCurrentTime();
        neighbor.last_rssi = rssi;
        neighbor.sequence_number = msg.sequence_number;
        neighbor.is_active = true;
        // Note: We don't update battery/position from non-HELLO messages

        // Update route (direct neighbor, 1 hop)
        updateRoute(source_id, source_id, 1, msg.sequence_number, rssi);

        LOG_V("Neighbor %s refreshed from message (RSSI=%d)", msg.source_id, rssi);
    }
}

// ================================================================
// Route Discovery (RREQ)
// ================================================================

void RoutingEngine::initiateRouteDiscovery(const String& destination_id) {
    sendRouteRequest(destination_id);
    stats.rreq_sent++;
}

void RoutingEngine::sendRouteRequest(const String& destination_id) {
    // Create RouteRequestPayload using actual nanopb type
    RouteRequestPayload rreq = ProtobufHandler::createRouteRequestPayload();
    rreq.rreq_id = esp_random();
    rreq.originator_sequence_number = getNextSequenceNumber();
    rreq.hop_count = 0;
    rreq.broadcast_id = esp_random();

    // Set string fields - now static char arrays
    ProtobufHandler::copyString(rreq.originator_id, sizeof(rreq.originator_id), my_node_id);
    ProtobufHandler::copyString(rreq.destination_id, sizeof(rreq.destination_id), destination_id);

    uint8_t payload_buf[256];
    size_t payload_len = ProtobufHandler::encodePayload(rreq, payload_buf, sizeof(payload_buf));

    if (payload_len > 0) {
        LoRaMessage msg = createMessage("", MESSAGE_TYPE_ROUTE_REQUEST, payload_buf, payload_len, PRIORITY_TACTICAL);

        if (transmit_callback) {
            stats.messages_sent++;
            transmit_callback(msg);
            LOG_I("RREQ sent for destination: %s", destination_id.c_str());
        }
    }
}

void RoutingEngine::processRouteRequest(const LoRaMessage& msg, int8_t rssi) {
    // Decode RouteRequestPayload using actual nanopb type
    RouteRequestPayload rreq = lora_mesh_v1_RouteRequestPayload_init_zero;

    // payload is now PB_BYTES_ARRAY_T - access via .bytes and .size
    if (!ProtobufHandler::decodePayload(msg.payload.bytes, msg.payload.size, rreq)) {
        LOG_E("Failed to decode RREQ");
        return;
    }

    // String fields are now char arrays - convert to String for operations
    String originator_id = rreq.originator_id;
    String destination_id = rreq.destination_id;
    String source_id = msg.source_id;

    LOG_D("RREQ from %s to %s (seq=%u, hop=%u)",
          originator_id.c_str(), destination_id.c_str(),
          rreq.originator_sequence_number, rreq.hop_count);

    // Create reverse route to originator
    createReverseRoute(originator_id, source_id, rreq.hop_count + 1,
                      rreq.originator_sequence_number);

    // Am I the destination?
    if (destination_id == my_node_id) {
        LOG_I("RREQ destination reached, sending RREP to: %s", originator_id.c_str());
        sendRouteReply(originator_id, my_node_id, 0, my_sequence_number);
        stats.rrep_sent++;
        return;
    }

    // Do I have a fresh route to destination?
    RouteEntry* route = findRoute(destination_id);
    if (route && route->is_valid) {
        LOG_I("Intermediate RREP for %s via %s", destination_id.c_str(), route->next_hop_id.c_str());
        sendRouteReply(originator_id, destination_id,
                      route->hop_count, route->sequence_number);
        stats.rrep_sent++;
        return;
    }

    // Forward RREQ
    if (msg.hop_count < DEFAULT_TTL) {
        LoRaMessage fwd_msg = msg;
        fwd_msg.hop_count++;
        fwd_msg.ttl--;

        // Update payload with new hop count - need to re-encode with updated hop_count
        RouteRequestPayload fwd_rreq = ProtobufHandler::createRouteRequestPayload();
        fwd_rreq.rreq_id = rreq.rreq_id;
        fwd_rreq.originator_sequence_number = rreq.originator_sequence_number;
        fwd_rreq.hop_count = fwd_msg.hop_count;
        fwd_rreq.broadcast_id = rreq.broadcast_id;
        ProtobufHandler::copyString(fwd_rreq.originator_id, sizeof(fwd_rreq.originator_id), originator_id);
        ProtobufHandler::copyString(fwd_rreq.destination_id, sizeof(fwd_rreq.destination_id), destination_id);

        uint8_t payload_buf[256];
        size_t payload_len = ProtobufHandler::encodePayload(fwd_rreq, payload_buf, sizeof(payload_buf));
        ProtobufHandler::copyBytes(fwd_msg.payload, payload_buf, payload_len);

        // Random backoff to avoid collisions
        delay(random(RREQ_BACKOFF_MIN_MS, RREQ_BACKOFF_MAX_MS));

        if (transmit_callback) {
            transmit_callback(fwd_msg);
            stats.messages_forwarded++;
            LOG_D("RREQ forwarded (hop=%u)", fwd_msg.hop_count);
        }
    }
}

// ================================================================
// Route Reply (RREP)
// ================================================================

void RoutingEngine::sendRouteReply(const String& originator_id, const String& destination_id,
                                  uint32_t hop_count, uint32_t dest_seq) {
    // Create RouteReplyPayload using actual nanopb type
    RouteReplyPayload rrep = ProtobufHandler::createRouteReplyPayload();
    rrep.destination_sequence_number = dest_seq;
    rrep.hop_count = hop_count;
    rrep.lifetime_ms = ROUTE_LIFETIME_MS;
    rrep.reply_timestamp = ProtobufHandler::getCurrentTimestamp();

    // Set string fields - now static char arrays
    ProtobufHandler::copyString(rrep.destination_id, sizeof(rrep.destination_id), destination_id);
    ProtobufHandler::copyString(rrep.originator_id, sizeof(rrep.originator_id), originator_id);

    uint8_t payload_buf[256];
    size_t payload_len = ProtobufHandler::encodePayload(rrep, payload_buf, sizeof(payload_buf));

    if (payload_len > 0) {
        LoRaMessage msg = createMessage(originator_id, MESSAGE_TYPE_ROUTE_REPLY, payload_buf, payload_len, PRIORITY_TACTICAL);

        if (transmit_callback) {
            stats.messages_sent++;
            transmit_callback(msg);
            LOG_I("RREP sent to %s for dest %s", originator_id.c_str(), destination_id.c_str());
        }
    }
}

void RoutingEngine::processRouteReply(const LoRaMessage& msg, int8_t rssi) {
    // Decode RouteReplyPayload using actual nanopb type
    RouteReplyPayload rrep = lora_mesh_v1_RouteReplyPayload_init_zero;

    // payload is now PB_BYTES_ARRAY_T - access via .bytes and .size
    if (!ProtobufHandler::decodePayload(msg.payload.bytes, msg.payload.size, rrep)) {
        LOG_E("Failed to decode RREP");
        return;
    }

    // String fields are now char arrays - convert to String for operations
    String decoded_dest_id = rrep.destination_id;
    String decoded_orig_id = rrep.originator_id;
    String source_id = msg.source_id;
    String msg_dest_id = msg.destination_id;

    LOG_I("RREP received for dest: %s (hop=%u, seq=%u)",
          decoded_dest_id.c_str(), rrep.hop_count, rrep.destination_sequence_number);

    // Check if we already have a route to this destination
    RouteEntry* existing = findRoute(decoded_dest_id);

    if (existing && existing->is_valid) {
        // We have an existing route - this might be an alternate path
        // Add as alternate if it's different and not worse than existing
        if (existing->next_hop_id != source_id) {
            addAlternateRoute(decoded_dest_id, source_id,
                            rrep.hop_count + 1, rssi);
        }
    } else {
        // Create new primary route to destination via source of this RREP
        updateRoute(decoded_dest_id, source_id, rrep.hop_count + 1,
                   rrep.destination_sequence_number, rssi);
        stats.routes_discovered++;
    }

    // Am I the originator?
    if (msg_dest_id == my_node_id) {
        LOG_I("Route discovered to: %s", decoded_dest_id.c_str());

        // Send any queued messages
        auto it = pending_routes.find(decoded_dest_id);
        if (it != pending_routes.end()) {
            for (auto& queued_msg : it->second.queued_messages) {
                if (transmit_callback) {
                    transmit_callback(queued_msg);
                    stats.messages_sent++;
                }
            }
            pending_routes.erase(it);
        }
    } else {
        // Forward RREP
        if (msg.hop_count < DEFAULT_TTL) {
            LoRaMessage fwd_msg = msg;
            fwd_msg.hop_count++;
            fwd_msg.ttl--;

            // Update payload with new hop count - need to re-encode
            RouteReplyPayload fwd_rrep = ProtobufHandler::createRouteReplyPayload();
            fwd_rrep.destination_sequence_number = rrep.destination_sequence_number;
            fwd_rrep.hop_count = fwd_msg.hop_count;
            fwd_rrep.lifetime_ms = rrep.lifetime_ms;
            fwd_rrep.reply_timestamp = rrep.reply_timestamp;
            ProtobufHandler::copyString(fwd_rrep.destination_id, sizeof(fwd_rrep.destination_id), decoded_dest_id);
            ProtobufHandler::copyString(fwd_rrep.originator_id, sizeof(fwd_rrep.originator_id), decoded_orig_id);

            uint8_t payload_buf[256];
            size_t payload_len = ProtobufHandler::encodePayload(fwd_rrep, payload_buf, sizeof(payload_buf));
            ProtobufHandler::copyBytes(fwd_msg.payload, payload_buf, payload_len);

            if (transmit_callback) {
                transmit_callback(fwd_msg);
                stats.messages_forwarded++;
            }
        }
    }
}

// ================================================================
// Route Error (RERR)
// ================================================================

void RoutingEngine::sendRouteError(const String& unreachable_dest) {
    LOG_W("Sending RERR for unreachable destination: %s", unreachable_dest.c_str());

    // Create Route Error payload using actual nanopb type
    RouteErrorPayload rerr = ProtobufHandler::createRouteErrorPayload();
    rerr.error_cause = lora_mesh_v1_RouteErrorCause_ROUTE_ERROR_CAUSE_LINK_FAILURE;
    rerr.error_timestamp = ProtobufHandler::getCurrentTimestamp();
    rerr.affected_message_count = 1;

    // Set string fields - now static char arrays
    ProtobufHandler::copyString(rerr.reporting_node, sizeof(rerr.reporting_node), my_node_id);
    // Note: unreachable_destinations is a repeated field - for simplicity, we just encode the dest
    // A full implementation would add UnreachableDestination structs

    uint8_t payload_buf[256];
    size_t payload_len = ProtobufHandler::encodePayload(rerr, payload_buf, sizeof(payload_buf));

    if (payload_len > 0) {
        LoRaMessage msg = createMessage("", MESSAGE_TYPE_ROUTE_ERROR, payload_buf, payload_len, PRIORITY_TACTICAL);

        // Broadcast RERR
        if (transmit_callback) {
            stats.rerr_sent++;
            if (transmit_callback(msg)) {
                LOG_I("RERR broadcasted for: %s", unreachable_dest.c_str());
            } else {
                LOG_E("Failed to broadcast RERR");
            }
        }
    }
}

void RoutingEngine::processRouteError(const LoRaMessage& msg) {
    // source_id is now char[16] - convert to String for logging
    String source_id = msg.source_id;
    LOG_W("Processing RERR from: %s", source_id.c_str());

    // Decode RERR payload using actual nanopb type
    RouteErrorPayload rerr = lora_mesh_v1_RouteErrorPayload_init_zero;

    // payload is now PB_BYTES_ARRAY_T - access via .bytes and .size
    if (!ProtobufHandler::decodePayload(msg.payload.bytes, msg.payload.size, rerr)) {
        LOG_E("Failed to decode RERR");
        return;
    }

    // reporting_node is now char[16] - convert to String
    String reporting_node = strlen(rerr.reporting_node) > 0 ? String(rerr.reporting_node) : source_id;

    // For this simplified implementation, we treat the reporting node as the unreachable destination
    // A full implementation would iterate through unreachable_destinations
    String unreachable_dest = reporting_node;

    // Find and invalidate route to unreachable destination
    auto it = routing_table.find(unreachable_dest);
    if (it != routing_table.end()) {
        RouteEntry& route = it->second;

        // Check if this route goes through the reporting node
        if (route.next_hop_id == source_id ||
            route.next_hop_id == reporting_node) {

            LOG_W("Invalidating route to %s via %s",
                  unreachable_dest.c_str(), route.next_hop_id.c_str());

            // Record failure for next hop
            recordNeighborFailure(route.next_hop_id);

            // Try to promote an alternate route
            if (!route.backup_routes.empty()) {
                promoteAlternateRoute(unreachable_dest);
                LOG_I("Promoted alternate route to: %s", unreachable_dest.c_str());
            } else {
                // No alternates, mark route as invalid
                route.is_valid = false;
                LOG_W("No alternate routes available for: %s", unreachable_dest.c_str());

                // Trigger route rediscovery if we have pending messages
                auto pending_it = pending_routes.find(unreachable_dest);
                if (pending_it == pending_routes.end() &&
                    !routing_table.find(unreachable_dest)->second.is_valid) {
                    initiateRouteDiscovery(unreachable_dest);
                }
            }
        }
    }
}

// ================================================================
// Message Forwarding
// ================================================================

bool RoutingEngine::forwardMessage(LoRaMessage& msg) {
    // destination_id is now char[16] - convert to String for map operations
    String dest_id = msg.destination_id;
    RouteEntry* route = findRoute(dest_id);

    if (!route || !route->is_valid) {
        LOG_W("No valid route to: %s", msg.destination_id);
        return false;
    }

    // Skip blacklisted next hops
    if (isNeighborBlacklisted(route->next_hop_id)) {
        LOG_W("Next hop %s is blacklisted, trying alternates", route->next_hop_id.c_str());

        // Try alternate routes
        if (tryAlternateRoutes(dest_id, msg)) {
            return true;
        }

        LOG_E("All routes to %s are blacklisted", msg.destination_id);
        return false;
    }

    msg.hop_count++;
    msg.ttl--;

    if (msg.ttl == 0 || msg.hop_count >= MAX_HOP_COUNT) {
        LOG_W("Message TTL expired during forward");
        return false;
    }

    // Attempt to forward via primary route
    if (transmit_callback && transmit_callback(msg)) {
        stats.messages_forwarded++;
        route->packets_forwarded++;
        route->last_used_time = getCurrentTime();

        // Record success for neighbor
        recordNeighborSuccess(route->next_hop_id);

        return true;
    }

    // Primary forward failed, record failure
    LOG_W("Forward failed via primary route: %s", route->next_hop_id.c_str());
    route->packets_dropped++;
    recordNeighborFailure(route->next_hop_id);

    // Try alternate routes
    if (tryAlternateRoutes(dest_id, msg)) {
        return true;
    }

    // All routes failed, send RERR
    LOG_E("All routes to %s failed", msg.destination_id);
    sendRouteError(dest_id);

    return false;
}

void RoutingEngine::deliverToApplication(const LoRaMessage& msg) {
    // Store message in history (persisted to NVS)
    storeMessage(msg);

    if (receive_callback) {
        receive_callback(msg);
    }
    LOG_D("Message delivered to application: type=%d from=%s",
          msg.message_id, msg.source_id);
}

// ================================================================
// Maintenance
// ================================================================

void RoutingEngine::maintain() {
    uint64_t now = getCurrentTime();

    // Send periodic HELLO beacon
    if (now - last_hello_time >= HELLO_BEACON_INTERVAL_MS) {
        sendHelloBeacon();
        last_hello_time = now;
    }

    // Cleanup expired routes and neighbors
    if (now - last_cleanup_time >= ROUTE_CLEANUP_INTERVAL_MS) {
        cleanupRoutes();
        cleanupNeighbors();
        clearExpiredBlacklists();  // Clear expired blacklists
        cleanupFragmentBuffers();  // Cleanup timed out fragment buffers
        last_cleanup_time = now;
    }

    // Periodically save state to NVS (if dirty)
    if (state_dirty && (millis() - last_save_time >= SAVE_INTERVAL_MS)) {
        saveState();
    }
}

// ================================================================
// Helper Functions
// ================================================================

bool RoutingEngine::shouldRebroadcast(const String& sender_id) {
    // Check if sender is a known neighbor
    auto sender_it = neighbor_table.find(sender_id);
    if (sender_it == neighbor_table.end()) {
        // Sender not in our neighbor table - we should rebroadcast
        // since we likely have nodes that can't hear the sender
        return true;
    }

    // Get sender's known neighbors from their HELLO beacons
    // If sender has fewer neighbors than us, we might be a bridge
    const NeighborEntry& sender = sender_it->second;

    // Check if we have any neighbors that the sender doesn't have
    // The sender's neighbor list is built from their HELLO beacons
    for (const auto& my_neighbor : neighbor_table) {
        if (my_neighbor.first == sender_id) {
            continue;  // Skip the sender itself
        }

        // Check if sender knows this neighbor
        // We infer this from routes: if sender has a 1-hop route to this node,
        // they can reach it directly
        bool sender_knows_neighbor = false;

        // Check if our neighbor has the sender as a neighbor
        // (symmetric link check - if A heard B, B probably heard A)
        auto route_it = routing_table.find(my_neighbor.first);
        if (route_it != routing_table.end()) {
            // If we have a route to this neighbor, check if it's also reachable from sender
            // Simple heuristic: if hop_count == 1, it's a direct neighbor
            if (route_it->second.hop_count == 1) {
                // This is our direct neighbor - check if sender can also reach them
                // We can only know this if we've seen the sender's LSA/HELLO
                // For now, assume sender can reach them if sender has good RSSI
                if (sender.last_rssi > -80) {  // Good signal = likely shared neighbors
                    sender_knows_neighbor = true;
                }
            }
        }

        if (!sender_knows_neighbor) {
            // We have a neighbor the sender might not reach - rebroadcast
            LOG_D("Bridge node: rebroadcast to reach %s", my_neighbor.first.c_str());
            return true;
        }
    }

    // All our neighbors are likely reachable by sender - no need to rebroadcast
    return false;
}

bool RoutingEngine::isDuplicate(const String& message_id) {
    for (const auto& entry : duplicate_cache) {
        if (entry.message_id == message_id) {
            return true;
        }
    }
    return false;
}

void RoutingEngine::addToDuplicateCache(const String& message_id) {
    DuplicateEntry entry;
    entry.message_id = message_id;
    entry.timestamp = getCurrentTime();

    duplicate_cache.push_back(entry);

    // Limit cache size
    if (duplicate_cache.size() > MAX_DUPLICATE_CACHE_SIZE) {
        duplicate_cache.erase(duplicate_cache.begin());
    }
}

RouteEntry* RoutingEngine::findRoute(const String& destination_id) {
    auto it = routing_table.find(destination_id);
    if (it != routing_table.end()) {
        return &it->second;
    }
    return nullptr;
}

void RoutingEngine::updateRoute(const String& destination_id, const String& next_hop,
                               uint32_t hop_count, uint32_t seq_num, int8_t rssi) {
    // Check if this is a NEW route (not just an update)
    auto it = routing_table.find(destination_id);
    bool is_new_route = (it == routing_table.end() || !it->second.is_valid);
    bool is_better_route = false;

    if (!is_new_route && it != routing_table.end()) {
        // Check if this is a better route (fewer hops or fresher sequence)
        is_better_route = (hop_count < it->second.hop_count) ||
                          (seq_num > it->second.sequence_number);
    }

    RouteEntry& route = routing_table[destination_id];
    route.destination_id = destination_id;
    route.next_hop_id = next_hop;
    route.hop_count = hop_count;
    route.sequence_number = seq_num;
    route.lifetime_ms = ROUTE_LIFETIME_MS;
    route.last_update_time = getCurrentTime();
    route.rssi = rssi;
    route.is_valid = true;

    // Only increment mesh version for MULTI-HOP routes (new topology we discovered)
    // 1-hop routes are just direct neighbors - if we incremented for those,
    // every node hearing a new neighbor would increment and flood the network
    if ((is_new_route || is_better_route) && hop_count > 1) {
        incrementMeshVersion();
        state_dirty = true;
        LOG_I("Topology changed: %s route to %s via %s (%d hops)",
              is_new_route ? "new" : "better", destination_id.c_str(),
              next_hop.c_str(), hop_count);
    } else if (is_new_route || is_better_route) {
        state_dirty = true;
        LOG_D("Direct route added: %s via %s (1 hop, no version increment)",
              destination_id.c_str(), next_hop.c_str());
    } else {
        LOG_D("Route refreshed: dest=%s, next_hop=%s, hops=%d",
              destination_id.c_str(), next_hop.c_str(), hop_count);
    }
}

void RoutingEngine::createReverseRoute(const String& originator_id, const String& previous_hop,
                                      uint32_t hop_count, uint32_t seq_num) {
    updateRoute(originator_id, previous_hop, hop_count, seq_num, 0);
}

void RoutingEngine::cleanupRoutes() {
    uint64_t now = getCurrentTime();

    for (auto it = routing_table.begin(); it != routing_table.end();) {
        if (now - it->second.last_update_time > it->second.lifetime_ms) {
            LOG_D("Route expired: %s", it->second.destination_id.c_str());
            it = routing_table.erase(it);
        } else {
            ++it;
        }
    }
}

void RoutingEngine::cleanupNeighbors() {
    uint64_t now = getCurrentTime();

    for (auto it = neighbor_table.begin(); it != neighbor_table.end();) {
        if (now - it->second.last_seen_time > NEIGHBOR_TIMEOUT_MS) {
            LOG_D("Neighbor timeout: %s", it->second.node_id.c_str());
            it->second.is_active = false;
            ++it;
        } else {
            ++it;
        }
    }
}

uint32_t RoutingEngine::getNextSequenceNumber() {
    return ++my_sequence_number;
}

uint64_t RoutingEngine::getCurrentTime() const {
    return millis();
}

LoRaMessage RoutingEngine::createMessage(const String& dest_id, MessageType type,
                                        const uint8_t* payload_data, size_t payload_len,
                                        MessagePriority priority) {
    LoRaMessage msg = lora_mesh_v1_LoRaMessage_init_zero;

    // Copy strings to static char arrays
    String msg_id = ProtobufHandler::generateMessageID();
    ProtobufHandler::copyString(msg.message_id, sizeof(msg.message_id), msg_id);
    ProtobufHandler::copyString(msg.source_id, sizeof(msg.source_id), my_node_id);
    ProtobufHandler::copyString(msg.destination_id, sizeof(msg.destination_id), dest_id);

    msg.message_type = type;

    // Copy payload to PB_BYTES_ARRAY_T
    if (payload_data && payload_len > 0) {
        ProtobufHandler::copyBytes(msg.payload, payload_data, payload_len);
    }

    msg.timestamp = ProtobufHandler::getCurrentTimestamp();
    msg.ttl = DEFAULT_TTL;
    msg.hop_count = 0;
    msg.sequence_number = getNextSequenceNumber();
    msg.priority = priority;

    return msg;
}

std::vector<RouteEntry> RoutingEngine::getRoutingTable() const {
    std::vector<RouteEntry> routes;
    for (const auto& pair : routing_table) {
        routes.push_back(pair.second);
    }
    return routes;
}

std::vector<NeighborEntry> RoutingEngine::getNeighborTable() const {
    std::vector<NeighborEntry> neighbors;
    for (const auto& pair : neighbor_table) {
        // Only return active neighbors
        if (pair.second.is_active) {
            neighbors.push_back(pair.second);
        }
    }
    return neighbors;
}

// ================================================================
// Multi-Path Routing Implementation
// ================================================================

void RoutingEngine::discoverMultiplePaths(const String& destination_id) {
    LOG_I("Discovering multiple paths to: %s", destination_id.c_str());

    // Launch 3 staggered RREQs with different optimization goals
    // RREQ 1: Minimize hop count (immediate)
    sendRouteRequest(destination_id);

    // RREQ 2: Optimize for RSSI/quality (50ms delay)
    // RREQ 3: Energy-aware path (100ms delay)
    // Note: Delays would need timer implementation in main loop
    // For now, just send immediate RREQ and let RREP processing handle alternates
}

void RoutingEngine::addAlternateRoute(const String& destination_id, const String& next_hop,
                                     uint32_t hop_count, int8_t rssi) {
    RouteEntry* primary = findRoute(destination_id);
    if (!primary) {
        return;  // No primary route exists
    }

    // Don't add if same as primary
    if (primary->next_hop_id == next_hop) {
        return;
    }

    // Check if alternate already exists
    for (auto& alt : primary->backup_routes) {
        if (alt.next_hop_id == next_hop) {
            // Update existing alternate
            alt.hop_count = hop_count;
            alt.rssi = rssi;
            alt.last_used = getCurrentTime();
            LOG_D("Updated alternate route to %s via %s", destination_id.c_str(), next_hop.c_str());
            return;
        }
    }

    // Add new alternate if space available
    if (primary->backup_routes.size() < MAX_ALTERNATE_ROUTES) {
        AlternateRoute alt;
        alt.next_hop_id = next_hop;
        alt.hop_count = hop_count;
        alt.rssi = rssi;
        alt.quality_score = 0.0;  // Will be calculated
        alt.last_used = getCurrentTime();

        primary->backup_routes.push_back(alt);
        LOG_I("Added alternate route to %s via %s (total: %d)",
              destination_id.c_str(), next_hop.c_str(), primary->backup_routes.size());
    }
}

bool RoutingEngine::tryAlternateRoutes(const String& destination_id, LoRaMessage& msg) {
    RouteEntry* route = findRoute(destination_id);
    if (!route || route->backup_routes.empty()) {
        return false;
    }

    LOG_I("Trying alternate routes to: %s", destination_id.c_str());

    // Try each alternate route
    for (size_t i = 0; i < route->backup_routes.size(); i++) {
        AlternateRoute& alt = route->backup_routes[i];

        // Skip blacklisted neighbors
        if (isNeighborBlacklisted(alt.next_hop_id)) {
            LOG_D("Alternate %s is blacklisted, skipping", alt.next_hop_id.c_str());
            continue;
        }

        // Try to forward via this alternate
        String old_next_hop = route->next_hop_id;
        route->next_hop_id = alt.next_hop_id;
        route->hop_count = alt.hop_count;
        route->rssi = alt.rssi;

        if (transmit_callback && transmit_callback(msg)) {
            LOG_I("Successfully forwarded via alternate: %s", alt.next_hop_id.c_str());
            stats.alternate_routes_used++;
            alt.last_used = getCurrentTime();
            return true;
        }

        // Restore primary
        route->next_hop_id = old_next_hop;
    }

    return false;
}

float RoutingEngine::calculateRouteQuality(const RouteEntry& route) {
    // Quality formula: RSSI(40%) + HopCount(30%) + Stability(20%) + Energy(10%)

    // RSSI factor: normalize -120 to -40 dBm to 0.0-1.0
    float rssi_factor = (route.rssi + 120.0f) / 80.0f;
    rssi_factor = constrain(rssi_factor, 0.0f, 1.0f);

    // Hop count factor: fewer hops is better
    float hop_factor = 1.0f / (1.0f + route.hop_count);

    // Stability factor: based on success rate
    float stability_factor = 0.5f;  // Default
    if (route.quality.success_count + route.quality.failure_count > 0) {
        stability_factor = (float)route.quality.success_count /
                          (route.quality.success_count + route.quality.failure_count);
    }

    // Energy factor: would need neighbor battery info, use 1.0 for now
    float energy_factor = 1.0f;

    return (rssi_factor * 0.4f) + (hop_factor * 0.3f) +
           (stability_factor * 0.2f) + (energy_factor * 0.1f);
}

void RoutingEngine::promoteAlternateRoute(const String& destination_id) {
    RouteEntry* route = findRoute(destination_id);
    if (!route || route->backup_routes.empty()) {
        return;
    }

    // Find best alternate based on quality score
    size_t best_idx = 0;
    float best_quality = 0.0f;

    for (size_t i = 0; i < route->backup_routes.size(); i++) {
        // Calculate quality for this alternate
        RouteEntry temp_route;
        temp_route.rssi = route->backup_routes[i].rssi;
        temp_route.hop_count = route->backup_routes[i].hop_count;
        temp_route.quality.success_count = 10;  // Assume good
        temp_route.quality.failure_count = 0;

        float quality = calculateRouteQuality(temp_route);

        if (quality > best_quality && !isNeighborBlacklisted(route->backup_routes[i].next_hop_id)) {
            best_quality = quality;
            best_idx = i;
        }
    }

    // Promote best alternate to primary
    AlternateRoute& best_alt = route->backup_routes[best_idx];

    // Save old primary as alternate
    AlternateRoute old_primary;
    old_primary.next_hop_id = route->next_hop_id;
    old_primary.hop_count = route->hop_count;
    old_primary.rssi = route->rssi;
    old_primary.quality_score = route->quality_score;
    old_primary.last_used = route->last_used_time;

    // Promote alternate to primary
    route->next_hop_id = best_alt.next_hop_id;
    route->hop_count = best_alt.hop_count;
    route->rssi = best_alt.rssi;
    route->quality_score = best_quality;
    route->last_update_time = getCurrentTime();

    // Remove promoted alternate and add old primary
    route->backup_routes.erase(route->backup_routes.begin() + best_idx);
    if (route->backup_routes.size() < MAX_ALTERNATE_ROUTES) {
        route->backup_routes.push_back(old_primary);
    }

    LOG_I("Promoted alternate route to %s via %s (quality: %.2f)",
          destination_id.c_str(), route->next_hop_id.c_str(), best_quality);
}

// ================================================================
// Neighbor Blacklisting Implementation
// ================================================================

bool RoutingEngine::recordNeighborFailure(const String& neighbor_id) {
    auto it = neighbor_table.find(neighbor_id);
    if (it == neighbor_table.end()) {
        return false;
    }

    NeighborEntry& neighbor = it->second;
    neighbor.failure_count++;
    neighbor.total_failures++;

    // Check if should be blacklisted
    if (neighbor.failure_count >= NEIGHBOR_FAILURE_THRESHOLD) {
        // Calculate blacklist duration with exponential backoff
        uint32_t duration = NEIGHBOR_BLACKLIST_DURATION_MS;
        if (neighbor.total_failures > 6) {
            duration = NEIGHBOR_BLACKLIST_MAX_DURATION_MS;  // Cap at 15 minutes
        } else if (neighbor.total_failures > 3) {
            duration = NEIGHBOR_BLACKLIST_DURATION_MS * 2;  // 10 minutes
        }

        neighbor.blacklist_until = getCurrentTime() + duration;

        LOG_W("Neighbor %s blacklisted for %d ms (failures: %d/%d)",
              neighbor_id.c_str(), duration, neighbor.failure_count, neighbor.total_failures);

        return true;
    }

    return false;
}

bool RoutingEngine::isNeighborBlacklisted(const String& neighbor_id) {
    auto it = neighbor_table.find(neighbor_id);
    if (it == neighbor_table.end()) {
        return false;
    }

    const NeighborEntry& neighbor = it->second;

    if (neighbor.blacklist_until > 0 && getCurrentTime() < neighbor.blacklist_until) {
        return true;
    }

    return false;
}

void RoutingEngine::clearExpiredBlacklists() {
    uint64_t now = getCurrentTime();

    for (auto& pair : neighbor_table) {
        NeighborEntry& neighbor = pair.second;

        if (neighbor.blacklist_until > 0 && now >= neighbor.blacklist_until) {
            LOG_I("Blacklist expired for neighbor: %s", pair.first.c_str());
            neighbor.blacklist_until = 0;
            neighbor.failure_count = 0;  // Reset consecutive failures
        }
    }
}

void RoutingEngine::recordNeighborSuccess(const String& neighbor_id) {
    auto it = neighbor_table.find(neighbor_id);
    if (it == neighbor_table.end()) {
        return;
    }

    NeighborEntry& neighbor = it->second;

    // Reset consecutive failure count on success
    if (neighbor.failure_count > 0) {
        LOG_D("Neighbor %s recovered (failures reset: %d)",
              neighbor_id.c_str(), neighbor.failure_count);
        neighbor.failure_count = 0;
    }
}

// ================================================================
// Fragmentation Implementation
// ================================================================

bool RoutingEngine::needsFragmentation(size_t payload_size) const {
    return payload_size > MAX_FRAGMENT_PAYLOAD;
}

bool RoutingEngine::sendFragmented(const String& dest_id, MessageType msg_type,
                                   const std::vector<uint8_t>& payload, MessagePriority priority) {
    if (payload.empty()) {
        LOG_E("Cannot fragment empty payload");
        return false;
    }

    // Calculate number of fragments needed
    size_t total_fragments = (payload.size() + MAX_FRAGMENT_PAYLOAD - 1) / MAX_FRAGMENT_PAYLOAD;

    if (total_fragments > 32) {
        LOG_E("Payload too large: %d bytes would need %d fragments (max 32)",
              payload.size(), total_fragments);
        return false;
    }

    // Generate stream ID
    uint32_t stream_id = next_stream_id++;

    LOG_I("Fragmenting %d bytes into %d fragments (stream %u)",
          payload.size(), total_fragments, stream_id);

    // Create and send each fragment
    for (size_t i = 0; i < total_fragments; i++) {
        // Calculate fragment data bounds
        size_t offset = i * MAX_FRAGMENT_PAYLOAD;
        size_t chunk_size = std::min(MAX_FRAGMENT_PAYLOAD, payload.size() - offset);

        // Build fragment payload
        lora_mesh_v1_FragmentPayload frag = lora_mesh_v1_FragmentPayload_init_zero;
        frag.stream_id = stream_id;
        frag.fragment_index = i;
        frag.total_fragments = total_fragments;
        frag.original_type = msg_type;

        // Set flags
        uint32_t flags = 0;
        if (i == 0) {
            flags |= lora_mesh_v1_FragmentFlags_FRAGMENT_FLAG_FIRST;
        }
        if (i == total_fragments - 1) {
            flags |= lora_mesh_v1_FragmentFlags_FRAGMENT_FLAG_LAST;
        } else {
            flags |= lora_mesh_v1_FragmentFlags_FRAGMENT_FLAG_MORE;
        }
        frag.flags = flags;

        // Copy fragment data
        frag.data.size = chunk_size;
        memcpy(frag.data.bytes, payload.data() + offset, chunk_size);

        // Encode fragment payload
        uint8_t frag_buffer[256];
        pb_ostream_t stream = pb_ostream_from_buffer(frag_buffer, sizeof(frag_buffer));
        if (!pb_encode(&stream, lora_mesh_v1_FragmentPayload_fields, &frag)) {
            LOG_E("Failed to encode fragment %d", i);
            return false;
        }

        // Create wrapper message with MESSAGE_TYPE_FRAGMENT
        LoRaMessage msg = createMessage(dest_id, lora_mesh_v1_MessageType_MESSAGE_TYPE_FRAGMENT,
                                        frag_buffer, stream.bytes_written, priority);

        // Queue for transmission
        if (transmit_callback) {
            if (!transmit_callback(msg)) {
                LOG_E("Failed to transmit fragment %d/%d", i + 1, total_fragments);
                return false;  // Abort on failure - partial streams are useless
            }
            stats.messages_sent++;
        }
    }

    LOG_I("Sent %d fragments for stream %u", total_fragments, stream_id);
    return true;
}

bool RoutingEngine::processFragment(const LoRaMessage& msg, int8_t rssi) {
    // Decode fragment payload
    lora_mesh_v1_FragmentPayload frag = lora_mesh_v1_FragmentPayload_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(msg.payload.bytes, msg.payload.size);

    if (!pb_decode(&stream, lora_mesh_v1_FragmentPayload_fields, &frag)) {
        LOG_E("Failed to decode fragment payload");
        return false;
    }

    uint32_t stream_id = frag.stream_id;
    String source_id = msg.source_id;

    LOG_D("Received fragment %d/%d for stream %u from %s",
          frag.fragment_index + 1, frag.total_fragments, stream_id, source_id.c_str());

    // Get or create fragment buffer
    auto it = fragment_buffers.find(stream_id);
    if (it == fragment_buffers.end()) {
        // New stream - create buffer
        if (fragment_buffers.size() >= MAX_FRAGMENT_BUFFERS) {
            // Cleanup oldest incomplete buffer
            cleanupFragmentBuffers();

            if (fragment_buffers.size() >= MAX_FRAGMENT_BUFFERS) {
                LOG_W("Too many fragment buffers, dropping stream %u", stream_id);
                return false;
            }
        }

        FragmentBuffer& buffer = fragment_buffers[stream_id];
        buffer.stream_id = stream_id;
        buffer.source_id = source_id;
        buffer.original_type = frag.original_type;
        buffer.total_fragments = frag.total_fragments;
        buffer.first_received_time = millis();
        buffer.rssi = rssi;
        buffer.fragments.resize(frag.total_fragments);
        buffer.received_mask.resize(frag.total_fragments, false);

        it = fragment_buffers.find(stream_id);
    }

    FragmentBuffer& buffer = it->second;

    // Validate fragment
    if (frag.fragment_index >= buffer.total_fragments) {
        LOG_E("Fragment index %d out of range (total %d)",
              frag.fragment_index, buffer.total_fragments);
        return false;
    }

    // Check for duplicate fragment
    if (buffer.received_mask[frag.fragment_index]) {
        LOG_D("Duplicate fragment %d for stream %u", frag.fragment_index, stream_id);
        return false;
    }

    // Store fragment
    buffer.fragments[frag.fragment_index].assign(
        frag.data.bytes, frag.data.bytes + frag.data.size);
    buffer.received_mask[frag.fragment_index] = true;
    buffer.received_count++;

    LOG_D("Stored fragment %d/%d for stream %u (%d received)",
          frag.fragment_index + 1, buffer.total_fragments, stream_id, buffer.received_count);

    // Check if complete
    if (buffer.received_count == buffer.total_fragments) {
        buffer.complete = true;

        LOG_I("Stream %u complete - reassembling %d fragments", stream_id, buffer.total_fragments);

        // Reassemble payload
        std::vector<uint8_t> reassembled = reassembleFragments(buffer);

        if (!reassembled.empty()) {
            // Create reassembled message with original type
            LoRaMessage reassembled_msg = lora_mesh_v1_LoRaMessage_init_zero;
            ProtobufHandler::copyString(reassembled_msg.message_id, sizeof(reassembled_msg.message_id),
                                        msg.message_id);  // Keep original message ID
            ProtobufHandler::copyString(reassembled_msg.source_id, sizeof(reassembled_msg.source_id),
                                        buffer.source_id);
            ProtobufHandler::copyString(reassembled_msg.destination_id, sizeof(reassembled_msg.destination_id),
                                        msg.destination_id);
            reassembled_msg.message_type = buffer.original_type;
            ProtobufHandler::copyBytes(reassembled_msg.payload, reassembled.data(), reassembled.size());
            reassembled_msg.timestamp = msg.timestamp;
            reassembled_msg.ttl = msg.ttl;
            reassembled_msg.hop_count = msg.hop_count;
            reassembled_msg.sequence_number = msg.sequence_number;
            reassembled_msg.priority = msg.priority;

            // Deliver to application or forward
            deliverToApplication(reassembled_msg);

            // Rebroadcast the burst if we should
            if (shouldRebroadcast(buffer.source_id)) {
                rebroadcastFragmentBurst(buffer);
            }
        }

        // Remove completed buffer
        fragment_buffers.erase(stream_id);
        return true;
    }

    return false;  // Not complete yet
}

std::vector<uint8_t> RoutingEngine::reassembleFragments(const FragmentBuffer& buffer) {
    std::vector<uint8_t> result;

    if (!buffer.complete) {
        LOG_E("Cannot reassemble incomplete buffer");
        return result;
    }

    // Calculate total size
    size_t total_size = 0;
    for (const auto& frag : buffer.fragments) {
        total_size += frag.size();
    }

    result.reserve(total_size);

    // Concatenate fragments in order
    for (const auto& frag : buffer.fragments) {
        result.insert(result.end(), frag.begin(), frag.end());
    }

    LOG_D("Reassembled %d bytes from %d fragments", result.size(), buffer.total_fragments);
    return result;
}

void RoutingEngine::rebroadcastFragmentBurst(const FragmentBuffer& buffer) {
    LOG_I("Rebroadcasting fragment burst (stream %u, %d fragments)",
          buffer.stream_id, buffer.total_fragments);

    // Rebuild and transmit each fragment
    for (uint32_t i = 0; i < buffer.total_fragments; i++) {
        lora_mesh_v1_FragmentPayload frag = lora_mesh_v1_FragmentPayload_init_zero;
        frag.stream_id = buffer.stream_id;
        frag.fragment_index = i;
        frag.total_fragments = buffer.total_fragments;
        frag.original_type = buffer.original_type;

        // Set flags
        uint32_t flags = 0;
        if (i == 0) {
            flags |= lora_mesh_v1_FragmentFlags_FRAGMENT_FLAG_FIRST;
        }
        if (i == buffer.total_fragments - 1) {
            flags |= lora_mesh_v1_FragmentFlags_FRAGMENT_FLAG_LAST;
        } else {
            flags |= lora_mesh_v1_FragmentFlags_FRAGMENT_FLAG_MORE;
        }
        frag.flags = flags;

        // Copy fragment data
        const std::vector<uint8_t>& data = buffer.fragments[i];
        frag.data.size = data.size();
        memcpy(frag.data.bytes, data.data(), data.size());

        // Encode
        uint8_t frag_buffer[256];
        pb_ostream_t stream = pb_ostream_from_buffer(frag_buffer, sizeof(frag_buffer));
        if (!pb_encode(&stream, lora_mesh_v1_FragmentPayload_fields, &frag)) {
            LOG_E("Failed to encode rebroadcast fragment %d", i);
            continue;
        }

        // Create message - mark as forwarded (hop_count > 0)
        LoRaMessage msg = createMessage("BROADCAST", lora_mesh_v1_MessageType_MESSAGE_TYPE_FRAGMENT,
                                        frag_buffer, stream.bytes_written,
                                        lora_mesh_v1_MessagePriority_MESSAGE_PRIORITY_NORMAL);
        msg.hop_count = 1;  // Mark as forwarded
        ProtobufHandler::copyString(msg.source_id, sizeof(msg.source_id), buffer.source_id);

        if (transmit_callback) {
            transmit_callback(msg);
        }

        stats.messages_forwarded++;
    }
}

void RoutingEngine::cleanupFragmentBuffers() {
    uint64_t now = millis();
    uint64_t oldest_time = now;
    uint32_t oldest_stream = 0;

    for (auto it = fragment_buffers.begin(); it != fragment_buffers.end(); ) {
        FragmentBuffer& buffer = it->second;

        // Remove timed out buffers
        if (now - buffer.first_received_time > FRAGMENT_TIMEOUT_MS) {
            LOG_W("Fragment stream %u timed out (%d/%d received)",
                  buffer.stream_id, buffer.received_count, buffer.total_fragments);
            it = fragment_buffers.erase(it);
        } else {
            // Track oldest for eviction if needed
            if (buffer.first_received_time < oldest_time) {
                oldest_time = buffer.first_received_time;
                oldest_stream = buffer.stream_id;
            }
            ++it;
        }
    }

    // If still too many, evict oldest
    if (fragment_buffers.size() >= MAX_FRAGMENT_BUFFERS && oldest_stream > 0) {
        LOG_W("Evicting oldest fragment stream %u to make room", oldest_stream);
        fragment_buffers.erase(oldest_stream);
    }
}
