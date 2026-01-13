/**
 * @file routing_engine.h
 * @brief AODV routing protocol implementation for LoRa mesh
 */

#ifndef ROUTING_ENGINE_H
#define ROUTING_ENGINE_H

#include <Arduino.h>
#include <map>
#include <vector>
#include <queue>
#include "config.h"
#include "protobuf_handler.h"

// ================================================================
// Data Structures
// ================================================================

// Alternate route information for multi-path routing
struct AlternateRoute {
    String next_hop_id;
    uint32_t hop_count;
    float quality_score;  // Composite quality metric [0.0-1.0]
    uint64_t last_used;
    int8_t rssi;
};

// Route quality metrics
struct RouteQualityMetrics {
    float average_rssi;         // Mean RSSI over time
    float packet_loss_rate;     // [0.0-1.0]
    float stability_factor;     // How stable the route is [0.0-1.0]
    uint32_t success_count;     // Successful transmissions
    uint32_t failure_count;     // Failed transmissions
};

struct RouteEntry {
    String destination_id;
    String next_hop_id;
    uint32_t hop_count;
    uint32_t sequence_number;
    uint32_t lifetime_ms;
    uint64_t last_update_time;
    int8_t rssi;  // Link quality
    bool is_valid;

    // Multi-path support
    std::vector<AlternateRoute> backup_routes;  // Alternate paths

    // Quality metrics
    RouteQualityMetrics quality;
    float quality_score;  // Overall quality [0.0-1.0]

    // Usage statistics
    uint32_t packets_forwarded;
    uint32_t packets_dropped;
    uint64_t last_used_time;

    // Route flags
    uint32_t route_flags;  // Bit flags for route state
};

struct NeighborEntry {
    String node_id;
    GPSCoordinate last_position;
    uint64_t last_seen_time;
    int8_t last_rssi;
    uint8_t battery_level;
    uint32_t sequence_number;
    bool is_active;

    // Blacklisting support
    uint8_t failure_count;      // Consecutive transmission failures
    uint64_t blacklist_until;   // Time when blacklist expires (0 = not blacklisted)
    uint32_t total_failures;    // Total lifetime failures
};

struct PendingRoute {
    String destination_id;
    uint64_t request_time;
    uint8_t retry_count;
    std::vector<LoRaMessage> queued_messages;
};

struct DuplicateEntry {
    String message_id;
    uint64_t timestamp;
};

// Fragment reassembly buffer for multi-packet messages
struct FragmentBuffer {
    uint32_t stream_id;               // Unique stream identifier
    String source_id;                 // Original sender
    MessageType original_type;        // Type of message being reassembled
    uint32_t total_fragments;         // Expected fragment count
    uint32_t received_count;          // How many fragments received
    uint64_t first_received_time;     // When first fragment arrived (for timeout)
    int8_t rssi;                      // RSSI of first fragment (for rebroadcast)
    std::vector<std::vector<uint8_t>> fragments;  // Indexed by fragment_index
    std::vector<bool> received_mask;  // Which fragments we have
    bool complete;                    // All fragments received

    FragmentBuffer() : stream_id(0), original_type(lora_mesh_v1_MessageType_MESSAGE_TYPE_UNSPECIFIED),
                       total_fragments(0), received_count(0), first_received_time(0),
                       rssi(0), complete(false) {}
};

// Fragmentation constants
// Max fragment data must fit in: MAX_LORA_PAYLOAD(222) - LoRaMessage envelope(~80) - FragmentPayload overhead(~20)
static constexpr size_t MAX_FRAGMENT_PAYLOAD = 120;    // Conservative limit to fit in single LoRa packet
static constexpr uint32_t FRAGMENT_TIMEOUT_MS = 5000;  // Wait 5s for complete stream
static constexpr size_t MAX_FRAGMENT_BUFFERS = 8;      // Max concurrent streams being reassembled

// ================================================================
// Routing Engine Class
// ================================================================

class RoutingEngine {
public:
    RoutingEngine();

    /**
     * @brief Initialize routing engine
     * @param node_id This node's ID
     * @return true if successful
     */
    bool init(const String& node_id);

    /**
     * @brief Process incoming LoRa message
     * @param msg Received message
     * @param rssi Signal strength
     * @return true if message was processed
     */
    bool processMessage(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Send a message to a destination
     * @param destination_id Target node ID ("BROADCAST" for broadcast)
     * @param msg_type Message type
     * @param payload Serialized payload
     * @param priority Message priority
     * @return true if queued successfully
     */
    bool sendMessage(const String& destination_id, MessageType msg_type,
                    const std::vector<uint8_t>& payload, MessagePriority priority);

    /**
     * @brief Periodic maintenance tasks (call from main loop)
     */
    void maintain();

    /**
     * @brief Get routing table for display/debugging
     */
    std::vector<RouteEntry> getRoutingTable() const;

    /**
     * @brief Get neighbor table
     */
    std::vector<NeighborEntry> getNeighborTable() const;

    /**
     * @brief Get statistics
     */
    struct {
        uint32_t messages_sent;
        uint32_t messages_received;
        uint32_t messages_forwarded;
        uint32_t messages_dropped;
        uint32_t rreq_sent;
        uint32_t rrep_sent;
        uint32_t rerr_sent;
        uint32_t routes_discovered;
        uint32_t alternate_routes_used;  // Times failover to alternate route
        uint32_t collisions_avoided;     // Transmission suppressions
    } stats;

    /**
     * @brief Set callback for outgoing LoRa transmissions
     */
    void setTransmitCallback(std::function<bool(const LoRaMessage&)> callback);

    /**
     * @brief Set callback for received application messages
     */
    void setReceiveCallback(std::function<void(const LoRaMessage&)> callback);

    /**
     * @brief Broadcast HELLO beacon (public for initial startup beacon)
     */
    void sendHelloBeacon();

    /**
     * @brief Initiate network join - broadcast discovery to learn about existing mesh
     * Call this when a node first boots up to discover and join the network
     */
    void initiateNetworkJoin();

    /**
     * @brief Send network discovery broadcast to learn topology
     * Triggers dual-responder pattern: best + worst RSSI neighbors respond
     */
    void sendNetworkDiscovery();

    /**
     * @brief Update this node's GPS position for geographic routing
     * @param position Current GPS coordinates
     */
    void setMyPosition(const GPSCoordinate& position);

    /**
     * @brief Get this node's current position
     * @return Current GPS coordinates (or default if not set)
     */
    GPSCoordinate getMyPosition() const { return my_position; }

    /**
     * @brief Check if this node has valid GPS position
     */
    bool hasValidPosition() const { return my_position.timestamp > 0; }

    // ================================================================
    // Mesh State Version (for synchronization)
    // ================================================================

    /**
     * @brief Get current mesh state version
     * This is a monotonic counter that increments on any state change
     * (GPS updates, text messages, node joins, etc.)
     * Nodes compare versions to detect if they're out of sync
     */
    uint64_t getMeshVersion() const { return mesh_version; }

    /**
     * @brief Increment mesh version (call when state changes)
     * Persists to NVS so version survives reboots
     * @return The new version number
     */
    uint64_t incrementMeshVersion();

    /**
     * @brief Update mesh version if received version is higher
     * Called when receiving messages with a higher mesh version
     * @param received_version Version from received message
     * @return true if our version was updated (we were behind)
     */
    bool updateMeshVersionIfNewer(uint64_t received_version);

    /**
     * @brief Request state synchronization from a node with newer data
     * Called when we detect our mesh version is behind
     * @param target_node Node ID to request state from
     */
    void requestStateSync(const String& target_node);

    // ================================================================
    // Persistent Storage
    // ================================================================

    /**
     * @brief Save all mesh state to NVS (routing table, neighbors, messages)
     * Call periodically or on significant state changes
     */
    void saveState();

    /**
     * @brief Load mesh state from NVS
     * Called automatically during init()
     */
    void loadState();

    /**
     * @brief Add a message to persistent history
     * @param msg Message to store
     */
    void storeMessage(const LoRaMessage& msg);

    /**
     * @brief Get stored message history
     * @return Vector of stored messages (most recent first)
     */
    std::vector<LoRaMessage> getMessageHistory() const { return message_history; }

private:
    // Node state
    String my_node_id;
    uint32_t my_sequence_number;  // Per-node sequence for AODV
    uint64_t mesh_version;        // Mesh-wide state version (persisted to NVS)
    GPSCoordinate my_position;    // This node's GPS position for geographic routing

    // Routing tables
    std::map<String, RouteEntry> routing_table;
    std::map<String, NeighborEntry> neighbor_table;
    std::map<String, PendingRoute> pending_routes;
    std::vector<DuplicateEntry> duplicate_cache;

    // Message history (persisted)
    std::vector<LoRaMessage> message_history;
    static constexpr size_t MAX_MESSAGE_HISTORY = 50;  // Limit stored messages

    // Persistence state
    bool state_dirty;              // True if state has changed since last save
    uint64_t last_save_time;       // Last time state was saved to NVS
    static constexpr uint64_t SAVE_INTERVAL_MS = 30000;  // Save every 30s if dirty

    // Message queue
    std::queue<LoRaMessage> outgoing_queue;

    // Fragment reassembly buffers (keyed by stream_id)
    std::map<uint32_t, FragmentBuffer> fragment_buffers;
    uint32_t next_stream_id;  // Counter for outgoing streams

    // Callbacks
    std::function<bool(const LoRaMessage&)> transmit_callback;
    std::function<void(const LoRaMessage&)> receive_callback;

    // Timing
    uint64_t last_hello_time;
    uint64_t last_cleanup_time;

    // ================================================================
    // AODV Protocol Functions
    // ================================================================

    /**
     * @brief Process HELLO beacon
     */
    void processHello(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Process network discovery request
     * Implements dual-responder strategy: best RSSI + worst RSSI neighbors respond
     */
    void processNetworkDiscovery(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Send neighbor/routing info to a joining node
     * @param target_id Node to send the info to
     * @param is_edge_responder True if we're the farthest reachable node
     */
    void sendNeighborUpdate(const String& target_id, bool is_edge_responder);

    /**
     * @brief Send content sync to a node (missed messages + GPS positions)
     * Called when responding to a state sync request to share actual content
     * Uses differential sync - only sends data newer than neighbor's mesh_version
     * @param target_id Node to send the content to
     * @param neighbor_mesh_version The neighbor's current mesh version (0 = send all)
     */
    void sendContentSync(const String& target_id, uint64_t neighbor_mesh_version = 0);

    /**
     * @brief Process incoming content sync payload
     * Updates neighbor positions and delivers synced text messages
     * @param msg The ContentSync message
     * @param rssi Signal strength
     */
    void processContentSync(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Check if we should respond to a discovery (best or worst RSSI)
     * @param rssi Signal strength of incoming discovery request
     * @return 0 = don't respond, 1 = best RSSI responder, 2 = worst RSSI responder
     */
    int shouldRespondToDiscovery(int8_t rssi);

    /**
     * @brief Broadcast newcomer announcement to nodes deeper in the mesh
     * Called by edge responder to propagate awareness of joining node
     * @param newcomer_id The node that just joined
     */
    void broadcastNewcomerAnnouncement(const String& newcomer_id);

    /**
     * @brief Process Link State Advertisement for proactive topology maintenance
     * Learns network topology from flooded LSAs and establishes multi-hop routes
     */
    void processLinkStateAdvertisement(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Update neighbor info from any received message
     */
    void updateNeighborFromMessage(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Initiate route discovery
     */
    void initiateRouteDiscovery(const String& destination_id);

    /**
     * @brief Create and broadcast RREQ
     */
    void sendRouteRequest(const String& destination_id);

    /**
     * @brief Process incoming RREQ
     */
    void processRouteRequest(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Send RREP back to originator
     */
    void sendRouteReply(const String& originator_id, const String& destination_id,
                       uint32_t hop_count, uint32_t dest_seq);

    /**
     * @brief Process incoming RREP
     */
    void processRouteReply(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Send RERR for broken route
     */
    void sendRouteError(const String& unreachable_dest);

    /**
     * @brief Process incoming RERR
     */
    void processRouteError(const LoRaMessage& msg);

    /**
     * @brief Forward a data message
     */
    bool forwardMessage(LoRaMessage& msg);

    /**
     * @brief Deliver message to application layer
     */
    void deliverToApplication(const LoRaMessage& msg);

    // ================================================================
    // Multi-Path Routing Functions
    // ================================================================

    /**
     * @brief Discover multiple paths to destination
     * Launches 3 staggered RREQs with different optimization goals
     */
    void discoverMultiplePaths(const String& destination_id);

    /**
     * @brief Add alternate route to routing table entry
     */
    void addAlternateRoute(const String& destination_id, const String& next_hop,
                          uint32_t hop_count, int8_t rssi);

    /**
     * @brief Try alternate routes when primary fails
     */
    bool tryAlternateRoutes(const String& destination_id, LoRaMessage& msg);

    /**
     * @brief Calculate route quality score
     * Combines RSSI (40%), hop count (30%), stability (20%), energy (10%)
     */
    float calculateRouteQuality(const RouteEntry& route);

    /**
     * @brief Promote best alternate route to primary
     */
    void promoteAlternateRoute(const String& destination_id);

    // ================================================================
    // Neighbor Blacklisting Functions
    // ================================================================

    /**
     * @brief Record transmission failure for neighbor
     * @return true if neighbor should be blacklisted
     */
    bool recordNeighborFailure(const String& neighbor_id);

    /**
     * @brief Check if neighbor is currently blacklisted
     */
    bool isNeighborBlacklisted(const String& neighbor_id);

    /**
     * @brief Clear expired blacklist entries
     */
    void clearExpiredBlacklists();

    /**
     * @brief Record successful transmission
     */
    void recordNeighborSuccess(const String& neighbor_id);

    // ================================================================
    // Fragmentation Functions
    // ================================================================

    /**
     * @brief Check if payload needs fragmentation
     * @param payload_size Size of the payload in bytes
     * @return true if fragmentation is required
     */
    bool needsFragmentation(size_t payload_size) const;

    /**
     * @brief Fragment and send a large message
     * Splits payload into fragments and sends as a burst
     * @param dest_id Destination node
     * @param msg_type Original message type
     * @param payload Large payload to fragment
     * @param priority Message priority
     * @return true if all fragments queued successfully
     */
    bool sendFragmented(const String& dest_id, MessageType msg_type,
                       const std::vector<uint8_t>& payload, MessagePriority priority);

    /**
     * @brief Process incoming fragment
     * Buffers fragment and reassembles when complete
     * @param msg Fragment message
     * @param rssi Signal strength
     * @return true if complete message was reassembled and delivered
     */
    bool processFragment(const LoRaMessage& msg, int8_t rssi);

    /**
     * @brief Reassemble fragments into original payload
     * @param buffer Fragment buffer with all fragments
     * @return Reassembled payload (empty if incomplete)
     */
    std::vector<uint8_t> reassembleFragments(const FragmentBuffer& buffer);

    /**
     * @brief Rebroadcast a complete fragment stream as a burst
     * @param buffer Fragment buffer to rebroadcast
     */
    void rebroadcastFragmentBurst(const FragmentBuffer& buffer);

    /**
     * @brief Cleanup incomplete/timed out fragment buffers
     */
    void cleanupFragmentBuffers();

    // ================================================================
    // Helper Functions
    // ================================================================

    /**
     * @brief Check if we should rebroadcast a message from sender
     * Returns true if we have neighbors that sender doesn't have
     * (i.e., we're a "bridge" node that extends the sender's reach)
     */
    bool shouldRebroadcast(const String& sender_id);

    /**
     * @brief Check if message is duplicate
     */
    bool isDuplicate(const String& message_id);

    /**
     * @brief Add message to duplicate cache
     */
    void addToDuplicateCache(const String& message_id);

    /**
     * @brief Find route to destination
     */
    RouteEntry* findRoute(const String& destination_id);

    /**
     * @brief Add or update route
     */
    void updateRoute(const String& destination_id, const String& next_hop,
                    uint32_t hop_count, uint32_t seq_num, int8_t rssi);

    /**
     * @brief Remove expired routes
     */
    void cleanupRoutes();

    /**
     * @brief Remove inactive neighbors
     */
    void cleanupNeighbors();

    /**
     * @brief Create reverse route from RREQ
     */
    void createReverseRoute(const String& originator_id, const String& previous_hop,
                           uint32_t hop_count, uint32_t seq_num);

    /**
     * @brief Increment and get sequence number
     */
    uint32_t getNextSequenceNumber();

    /**
     * @brief Get current timestamp
     */
    uint64_t getCurrentTime() const;

    /**
     * @brief Create LoRaMessage wrapper
     */
    LoRaMessage createMessage(const String& dest_id, MessageType type,
                             const uint8_t* payload_data, size_t payload_len,
                             MessagePriority priority);
};

#endif // ROUTING_ENGINE_H
