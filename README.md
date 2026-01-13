# LoRa Mesh GPS Team Awareness Kit (TAK)

---

## ðŸš€ Start Here: What Is This?

A simple, secure, and robust way for your team to share live GPS locationsâ€”no cell towers, no Wi-Fi, just small radios and a map! Designed for field teams (airsoft, SAR, outdoor evenThis section explains our comprehensive routing system that combines multiple protocols for optimal performance in diverse LoRa mesh scenarios. The system has been designed to handle high mobility, varying link quality, energy constraints, and network partitions.s, etc.) who need to know where everyone is, even in remote areas.

---

## ðŸ—ºï¸ How It Works (In Plain English)

- Each team member carries a small device (ESP32 + LoRa radio + GPS module).
- The device gets your GPS position and sends it out over the air.
- All devices form a mesh network, relaying each otherâ€™s locations so everyone stays in sync.
- You can see your teammatesâ€™ positions on a map display, phone, or even a simple screen.

---

## ðŸŒŸ Key Features

- **Live Team Map:** See where everyone is, in real time.
- **No Infrastructure Needed:** Works anywhereâ€”forests, mountains, urban, or rural.
- **Secure:** All messages are signed, so only your team can join and see locations.
- **Scalable:** Supports 100+ users with smart routing to avoid radio overload.
- **Flexible:** Use for games, search and rescue, events, or any team activity.

---

## ðŸ Quick Start: Building Your Mesh

1. **Assemble Hardware:** ESP32 + LoRa + GPS for each teammate.
2. **Review Protocol Definitions:** Check the `/lora_mesh/v1/` folder for Protocol Buffer message definitions.
3. **Flash Starter Firmware:** Blink an LED, read GPS, send/receive LoRa packets.
4. **Test Range & GPS:** Make sure radios and GPS work in your area.
5. **Follow the Actionable Plan Below!**

---

## ðŸ› ï¸ Actionable Implementation Plan

Follow these steps to build your LoRa Mesh GPS Team Awareness Kit in a logical, testable sequence. Each phase builds on the last, ensuring a robust and maintainable system.

### Phase 1: Hardware Bring-Up & Basic Communication

- Assemble ESP32 + LoRa + GPS hardware for each node.
- Flash basic firmware: blink LED, read GPS, send/receive raw LoRa packets.
- Validate radio range and GPS accuracy in your environment.

### Phase 2: Message Serialization with Protobufs âœ… IMPLEMENTED

- **DONE**: Protocol Buffer definitions complete in `/lora_mesh/v1/` directory
  - `messages.proto`: Core LoRaMessage wrapper and routing control
  - `routing.proto`: AODV route request/reply payloads
  - `geographic.proto`: GPS coordinates and geographic routing
  - `common.proto`: Node info, cryptographic signatures, QoS
- **DONE**: nanopb integration configured with proper .options files for ESP32 memory constraints
- **READY**: Generated C code compiles cleanly for ESP32 using nanopb 0.4.9.1
- **NEXT**: Integrate protobuf encode/decode into your ESP32 firmware

### Phase 3: Cryptographic Signing & Verification

- Implement HMAC or ECC signing for all messages (see 'Cryptography' section).
- Add signature verification on receive; drop invalid messages.
- Test with mismatched keys to ensure security enforcement.

### Phase 4: Node Discovery & Routing Table

- Implement signed HELLO beacons for neighbor discovery.
- Build and maintain a routing table (see 'Routing Table Structure').
- Test neighbor addition/removal and table updates.

### Phase 5: On-Demand Routing (AODV)

- Implement RREQ, RREP, and RERR message handling using the protobuf message definitions.
- Enable route discovery and maintenance (see 'Routing Protocol').
- Test multi-hop message delivery and route repair.

### Phase 6: Smart Relay/Selective Rebroadcast

- Implement relay election and random backoff (see 'Smart Relays').
- Tune relay selection parameters (RSSI, random, role).
- Test with multiple nodes to ensure efficient message propagation.

### Phase 7: Application Layer (GPS, POI, Text Messaging)

- Integrate GPS data collection and periodic broadcast.
- Add POI marking, versioning, and sync via HELLO packets.
- Implement secure text messaging (unicast, group, broadcast).
- Test all application features in the mesh.

### Phase 8: Security Hardening & Replay Protection

- Add timestamp/sequence number checks for replay protection.
- Test with delayed/duplicate packets to ensure robustness.
- Review and rotate keys as needed.

### Phase 9: Visualization & User Interface

- Connect nodes to displays or mobile devices for live map and messaging.
- Integrate with TAK/ATAK or custom mapping apps if desired.

### Phase 10: Field Testing & Optimization

- Deploy in real-world scenarios; monitor performance and reliability.
- Tune parameters (update rate, TTL, relay weights) for your use case.
- Document lessons learned and update firmware as needed.

---

## ðŸ” How Do I See the Map?

- Connect your device to a phone, tablet, or small screen.
- Use a custom app, TAK/ATAK, or even a web map to view everyoneâ€™s positions.
- Alerts can warn you if someone goes out of bounds or loses contact.

---

## ðŸ”’ How Is It Secure?

- Only devices with the right digital keys can join and share locations.
- All messages are signed, so you know theyâ€™re real.
- No data leaves the meshâ€”no cloud, no tracking, just your team.

---

## ðŸ¤” Why Use This?

- Stay safe and coordinated in the field.
- No monthly fees or cell coverage required.
- Works in tough environments where other tech fails.

---

## ðŸ“‹ Current Implementation Status

### âœ… What's Implemented

- **Protocol Buffer Definitions**: Complete message format definitions in `/lora_mesh/v1/`
- **ESP32 Firmware**: Working C++ implementation in `/firmware/` directory
  - AODV routing protocol with RREQ/RREP/RERR handling
  - HMAC-SHA256 message authentication and replay protection
  - GPS management (hardware/static/manual modes)
  - LoRa radio interface (SX1262 via RadioLib)
  - Serial command-line interface for testing
  - Multi-board support (LilyGO T3S3, T-Deck)
- **Python Simulation**: Working network simulator in `/routing_simulation/`

### ðŸ”§ What's Next

- **Protocol Buffer Integration**: Replace placeholder serialization with actual nanopb
- **Field Testing**: Deploy 6-radio mesh for real-world validation
- **TFT Display UI**: Implement roster and messaging display
- **Python Backend**: Live visualization and monitoring
- **Power Optimization**: Battery life improvements and sleep modes

**Bottom line**: We have working firmware! Ready for hardware testing with 6 LilyGO radios.

## ðŸš€ Getting Started with Firmware

### Quick Start

```bash
cd firmware/
pip install platformio
pio run -e t3s3-gateway -t upload
pio device monitor -b 115200
```

See [firmware/BUILD.md](firmware/BUILD.md) for detailed build instructions and [firmware/README.md](firmware/README.md) for complete documentation.

---

# Technical Deep Dive (For Developers)

## Table of Contents

1. Introduction
2. Network Architecture
3. Node Addressing & Discovery
4. Routing Protocol (AODV for LoRa)
5. Channel & Section Management
6. Message Types & Formats
7. Routing Table Structure
8. Route Discovery & Maintenance
9. Information Synchronization
10. Scalability Considerations
11. Security & Reliability
12. Implementation Guidelines
13. Future Enhancements
14. References
15. Glossary

---

## 1. Introduction

This document details the design, operation, and scaling considerations for a custom LoRa mesh network routing protocol. The protocol is designed for environments with up to 100+ users, each equipped with a 915 MHz LoRa radio, and supports both independent and group communications across logical network sections (channels).

**Primary Use Case: GPS Forward Team Awareness Kit (TAK)**

This protocol is tailored for a GPS-based Forward Team Awareness Kit (TAK), enabling real-time location sharing, situational awareness, and team coordination in the field. Each node collects its GPS position and shares it with the mesh, allowing all team members to view the live positions of others, even in environments without cellular or Wi-Fi coverage.

---

## GPS Data Collection and Transmission (with Protocol Buffers) âœ… SCHEMA IMPLEMENTED

- **GPS Module Integration:** Each ESP32 node will be equipped with a GPS module (e.g., u-blox, NEO-6M) to obtain latitude, longitude, altitude, and timestamp.
- **Data Packetization:** GPS data will be packaged into Protocol Buffer messages defined in `/lora_mesh/v1/geographic.proto` and transmitted using the LoRaMessage wrapper from `/lora_mesh/v1/messages.proto`.
- **Update Rate:** The update interval can be configured (e.g., every 2â€“10 seconds) to balance freshness and network load.
- **Implementation Status:** Protocol Buffer schemas are complete and ready for ESP32 integration.

## GPS Data Protocol Buffer Message Format âœ… IMPLEMENTED

Our actual GPS/location messages use the protocol buffer definitions from the `/lora_mesh/v1/` directory:

```proto
// From geographic.proto - actual implemented schema
message GPSCoordinate {
  double latitude = 1;
  double longitude = 2;
  double altitude = 3;
  float accuracy = 4;
  uint64 timestamp = 5;
}

// From messages.proto - wrapper for all network messages
message LoRaMessage {
  string message_id = 1;
  string source_id = 2;
  string destination_id = 3;
  uint32 source_port = 4;
  uint32 destination_port = 5;
  MessageType message_type = 6;
  bytes payload = 7;        // Contains serialized GPS data
  uint64 timestamp = 8;
  uint32 ttl = 9;
  uint32 hop_count = 10;
  uint32 sequence_number = 11;
  MessagePriority priority = 12;
  RoutingStrategy routing_hint = 13;
  CryptographicSignature signature = 14;
}
```

**ESP32 Implementation Ready:**

- Protocol definitions compiled and tested with nanopb 0.4.9.1
- .options files configured for ESP32 memory constraints
- Ready for integration into ESP32 firmware (implementation needed)

**Future ESP32 Integration Example:**

```cpp
// Example of how GPS data will be packaged (implementation needed)
LoRaMessage message;
message.message_type = MESSAGE_TYPE_GPS_UPDATE;
message.source_id = my_node_id;
message.destination_id = "BROADCAST";

// Serialize GPS coordinate into payload
GPSCoordinate gps_data;
gps_data.latitude = current_lat;
gps_data.longitude = current_lon;
gps_data.timestamp = current_time();
// ... encode with nanopb into message.payload ...

// Sign and transmit (implementation needed)
```

- **Transmission:** Send the serialized and signed buffer over LoRa.
- **Reception:** On receive, verify the signature, then decode the protobuf message to extract GPS data.

**Tip:** Protobufs make it easy to add new fields (e.g., battery level, status) in the future without breaking compatibility.

## Visualization and Team Awareness (Implementation Needed)

- **Display:** Each node can be connected to a display (e.g., OLED, TFT, or mobile device via Bluetooth/serial) to show a live map of team positions.
- **Integration:** Data can be exported to TAK/ATAK, QGIS, or custom mapping apps for advanced visualization.
- **Alerts:** The system can generate alerts for out-of-bounds, lost comms, or proximity events.

## Security and Privacy âœ… SCHEMA IMPLEMENTED

- All GPS/location packets will be signed using the CryptographicSignature protobuf message to prevent spoofing and tampering.
- Only authenticated nodes can participate and view team locations.
- Location data is not sent to the cloud or external serversâ€”mesh only.
- Signature verification and cryptographic protocols defined in `/lora_mesh/v1/common.proto`.

---

## 2. Network Architecture

- **Nodes:** Each user is a node in the mesh network.
- **Mesh Topology:** Nodes form a dynamic, self-healing mesh network.
- **Sections/Channels:** The network is logically divided into sections, each mapped to a LoRa channel or spreading factor. Gateway nodes bridge sections.

## 3. Node Addressing & Discovery

- **Unique IDs:** Each node is assigned a unique address (e.g., 16-bit integer).
- **Neighbor Discovery:** Nodes periodically broadcast signed 'HELLO' beacons to discover and authenticate direct neighbors, ensuring only legitimate nodes are added to routing tables.
- **Section Assignment:** Nodes are assigned to sections based on location or function.

## 4. Routing Protocol (AODV for LoRa) âœ… SCHEMA IMPLEMENTED

- **On-Demand Routing:** Routes are established only when needed, minimizing control traffic.
- **Routing Table:** Each node maintains a table of known routes (destination, next hop, hop count, sequence number, section).
- **Route Discovery:** Initiated via RREQ (Route Request) messages; routes are confirmed with RREP (Route Reply).
- **Route Maintenance:** Broken links trigger RERR (Route Error) messages and route invalidation.
- **TTL & Sequence Numbers:** Used to limit propagation and prevent loops.
- **Protocol Buffer Messages:** RREQ/RREP payloads defined in `/lora_mesh/v1/routing.proto` with LoRaMessage wrapper.

## In-Depth: Routing Algorithm (Hybrid Multi-Protocol System)

This section explains how the mesh finds and maintains routes, how messages travel, and how the protocol is optimized for LoRaâ€™s unique constraints.

### 1. Hybrid Routing Architecture - Comprehensive Design

Our routing system implements a **sophisticated multi-protocol approach** that intelligently selects the best routing strategy based on current network conditions:

**Primary Routing Protocols:**

1. **AODV (Reactive)**: On-demand route discovery when needed - reduces control overhead
2. **Link State (Proactive)**: Maintains network topology awareness for immediate routing decisions
3. **Geographic Routing**: Uses GPS coordinates for position-based forwarding in mobile scenarios
4. **Multi-Path Routing**: Maintains alternate routes for reliability and load balancing

**Enhanced Route Discovery Process:**

- **Intelligent Protocol Selection**: System analyzes network conditions (mobility, density, energy) to choose optimal routing strategy
- **AODV with Enhancements**: Uses `RouteRequestPayload` messages with QoS requirements, energy constraints, and multi-path discovery
- **Geographic Fallback**: When topology routes fail, automatically switches to GPS-based forwarding with greedy and perimeter routing
- **Cryptographic Security**: All routing messages signed with `CryptographicSignature` supporting multiple algorithms (HMAC, Ed25519, ECDSA)

**Advanced Route Discovery Features:**

- **Multi-Path Discovery**: Simultaneously discovers multiple diverse paths with different optimization criteria
- **Collision Avoidance**: Priority-based delays and transmission suppression prevent broadcast storms when multiple nodes receive the same RREQ
- **Message Fragmentation**: Large messages automatically fragmented with per-fragment signing for security
- **QoS-Aware Discovery**: Route requests include service requirements for emergency, tactical, and routine messages

### 2. Geographic Routing & Mobility Adaptation

**When Used**: Mobile networks, topology route failures, sparse networks, GPS-enabled scenarios

**Geographic Routing Strategies Designed**:

- **Greedy Forwarding**: Forward to neighbor closest to destination with collision avoidance using priority-based delays
- **Perimeter Routing**: When greedy fails, route around obstacles using computational geometry and right-hand rule
- **Predictive Routing**: Account for node mobility by predicting future positions based on velocity estimation from GPS history

**Advanced Geographic Features**:

- **Collision Avoidance**: Multiple nodes receiving same message coordinate forwarding using geographic suitability scoring
- **Link Quality Integration**: Combines geographic progress with RSSI and battery levels for optimal forwarding decisions
- **Mobility Prediction**: Estimates node velocity from recent position history to route to predicted future locations

### 3. Multi-Path Routing & Advanced Route Maintenance

**Multi-Path Discovery Process**:

- **Diverse Path Discovery**: Launches multiple RREQs with different optimization preferences (shortest, most reliable, lowest energy)
- **Quality-Based Route Management**: Maintains primary route plus bounded list of alternate routes sorted by quality score
- **Intelligent Route Promotion**: Better routes automatically promoted to primary, with old primary demoted to alternate

**Route Quality Assessment Factors**:

- **Signal Strength**: RSSI measurements and link stability over time
- **Path Efficiency**: Hop count and geographic progress toward destination
- **Energy Considerations**: Battery levels and energy cost of intermediate nodes
- **Reliability Metrics**: Historical packet loss rates and route lifetime statistics

**Load Balancing & Congestion Control**:

- **Primary Route**: Handles normal traffic with highest quality path
- **Alternate Route Usage**: Distributes load during congestion, provides immediate failover
- **Congestion Detection**: Monitors channel utilization and implements adaptive backoff
- **Traffic Shaping**: Different message types get appropriate routing priority and path selection

### 4. Advanced Security & Message Handling

**Cryptographic Integration**:

- **Per-Message Signing**: All routing messages signed with `CryptographicSignature` supporting HMAC-SHA256, Ed25519, ECDSA
- **Fragment-Level Security**: Large messages fragmented with each fragment individually signed for integrity
- **Signature Verification**: All received messages verified before processing, invalid signatures dropped

**Message Fragmentation & Reassembly**:

- **Automatic Fragmentation**: Messages exceeding LoRa payload limits automatically fragmented
- **Fragment Security**: Each fragment includes `FragmentHeader` and individual cryptographic signature
- **Reassembly Management**: Timeout-based fragment collection with complete message reconstruction
- **Security Verification**: Fragment signatures verified before reassembly, complete message optionally re-verified

### 5. LoRa-Specific Optimizations & Collision Avoidance

**Broadcast Storm Prevention**:

- **Priority-Based Delays**: Better forwarders (closer to destination, better signal) get shorter transmission delays
- **Transmission Suppression**: Nodes monitor for duplicate transmissions and cancel their own if already forwarded by better node
- **Geographic Coordination**: Multiple receivers coordinate forwarding based on position, link quality, and battery level

**LoRa Physical Layer Optimizations**:

- **Adaptive TTL**: Dynamic Time To Live based on network density and message priority
- **Channel Awareness**: Route discovery respects LoRa channel boundaries and gateway bridging
- **Duty Cycle Compliance**: Transmission scheduling respects regional duty cycle regulations
- **Power Management**: Adaptive transmission power based on neighbor distance and network density

**Network Density Adaptation**:

- **Sparse Networks**: Increased beacon frequency and transmission power for connectivity
- **Dense Networks**: Reduced control overhead and more selective forwarding to prevent congestion
- **Mobility Detection**: Adjusts routing strategy based on detected node movement patterns

### 6. Routing Algorithm Example Flow

**Intelligent Route Discovery Process**:

```cpp
// Enhanced routing decision with multiple protocols
if (!route_exists(destination_id)) {
    // Analyze network conditions to select optimal strategy
    routing_strategy = select_optimal_routing_strategy(destination_id, message);

    switch(routing_strategy) {
        case AODV_REACTIVE:
            // Multi-path AODV discovery with QoS requirements
            rreq = create_route_request_with_qos(destination_id, qos_requirements);
            broadcast_with_collision_avoidance(rreq);
            break;

        case GEOGRAPHIC_ROUTING:
            // GPS-based forwarding with greedy/perimeter routing
            destination_gps = get_node_position(destination_id);
            geographic_forward_with_prediction(message, destination_gps);
            break;

        case HYBRID_MULTIPATH:
            // Simultaneous discovery using multiple strategies
            discover_multiple_paths(destination_id, max_paths=3);
            break;
    }
}

// Enhanced message forwarding with fragmentation support
if (message.size() > MAX_LORA_PAYLOAD) {
    fragments = fragment_message_with_crypto(message);
    for (fragment : fragments) {
        forward_with_collision_avoidance(fragment, next_hop);
    }
} else {
    forward_message(message, select_best_next_hop(destination_id));
}

// Advanced RREQ processing with multiple receiver coordination
on_rreq_receive(rreq_message, sender_id) {
    if (verify_cryptographic_signature(rreq_message)) {
        rreq = deserialize_route_request_payload(rreq_message);

        if (!is_duplicate_rreq(rreq)) {
            create_reverse_route(rreq, sender_id);

            if (rreq.destination_id == own_node_id) {
                // We are destination - send signed RREP
                rrep = create_route_reply_with_crypto(rreq);
                send_unicast_message(rrep, sender_id);
            } else if (has_fresh_route(rreq.destination_id)) {
                // Intermediate node reply with route metrics
                send_intermediate_route_reply(rreq, sender_id);
            } else {
                // Forward RREQ with collision avoidance
                calculate_forwarding_priority_and_delay(rreq);
                schedule_conditional_forward(rreq);
            }
        }
    }
}
```

if (is_destination(RREQ.dest) || have_fresh_route(RREQ.dest)) {
send_RREP(RREQ.source);
} else {
forward_RREQ();
}

// On RREP receive
update_forward_route(RREP.dest, previous_hop);
if (!is_source(RREP)) forward_RREP();

```

### 6. Best Practices
- Only keep active routes to save memory.
- Use signed control messages to prevent malicious route injection.
- Monitor link quality (RSSI/SNR) and prefer better routes.
- Use random backoff for broadcasts to reduce collisions.

### 7. Visual Flow Example
```

A -- B -- C -- D

A wants to send to D:

1. A â†’ (RREQ) â†’ B â†’ (RREQ) â†’ C â†’ (RREQ) â†’ D
2. D â†’ (RREP) â†’ C â†’ (RREP) â†’ B â†’ (RREP) â†’ A
3. A â†’ (DATA) â†’ B â†’ C â†’ D

````

This approach ensures reliable, efficient routing for GPS and team data, even as the network grows or changes in the field.

## 5. Channel & Section Management
- **Channel Assignment:** Each section operates on a distinct LoRa channel or spreading factor.
- **Gateway Nodes:** Nodes with multi-channel capability relay messages between sections.
- **Channel Switching:** Nodes may temporarily switch channels for inter-section communication if hardware supports it.

## 6. Message Types & Formats
- **RREQ:** Route Request (initiates route discovery)
- **RREP:** Route Reply (confirms route)
- **RERR:** Route Error (notifies of broken links)
- **HELLO:** Neighbor discovery/liveness
- **DATA:** Application data (unicast, multicast, or broadcast)
- **Message Format Example:**
  - Header: [Type][Source][Destination][Seq#][TTL][Section]
  - Payload: [Data]

## 7. Routing Table Structure

The routing table is a critical component of each node, storing the best-known paths to other nodes in the mesh. Each entry contains:

| Field            | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| Destination      | Unique address (ID) of the target node                                      |
| Next Hop         | Address of the immediate neighbor to forward packets to                     |
| Hop Count        | Number of hops to reach the destination                                     |
| Sequence Number  | Latest known sequence number for the destination (prevents stale routes)    |
| Section/Channel  | Logical section or channel for the route                                    |
| Expiry/Timeout   | Time (or timestamp) after which the route is considered invalid             |
| Metric (optional)| Link quality or cost metric (e.g., RSSI, SNR, ETX) for route optimization   |

**Example Routing Table:**

| Destination | Next Hop | Hop Count | Sequence Number | Section | Expiry | Metric |
|-------------|----------|-----------|-----------------|---------|--------|--------|
| 0x01        | 0x02     | 2         | 1001            | 1       | 60s    | 85     |
| 0x03        | 0x05     | 1         | 1005            | 2       | 45s    | 90     |

**Field Details:**
- **Destination:** The final node to which data is being sent.
- **Next Hop:** The immediate neighbor on the path to the destination.
- **Hop Count:** Lower values are preferred; helps avoid routing loops.
- **Sequence Number:** Ensures the freshest route is used; higher is newer.
- **Section/Channel:** Used for channel-aware routing and gateway selection.
- **Expiry/Timeout:** Routes are purged after expiry to keep the table current.
- **Metric:** (Optional) Used for advanced routing decisions (e.g., prefer higher RSSI).

**Best Practices:**
- Only store active routes to conserve memory.
- Update sequence numbers and expiry on route refresh.
- Remove expired or broken routes promptly.
- Use metrics for route selection if available.

**Implementation Note:**
Routing tables can be implemented as hash maps, arrays, or linked lists depending on node resources. For LoRa, keep the structure lightweight and efficient.

## 8. Route Discovery & Maintenance
- **Discovery:**
  - Source broadcasts RREQ with incremented sequence number and TTL.
  - Intermediate nodes forward RREQ if not seen before, updating their routing tables.
  - Destination or node with valid route replies with RREP.
- **Maintenance:**
  - Nodes monitor link health via HELLO messages.
  - On link failure, RERR is sent to affected nodes, triggering route re-discovery if needed.

## 9. Information Synchronization
- **Broadcast/Multicast:**
  - Nodes can broadcast updates to all nodes in a section or the entire network.
  - Sequence numbers prevent duplicate processing.
- **State Sync:**
  - Periodic or event-driven state synchronization for shared data (e.g., map updates, alerts).

## 10. Scalability Considerations
- **Control Traffic Minimization:**
  - On-demand routing and TTL-limited broadcasts reduce network congestion.
- **Channelization:**
  - Logical sections and channel separation allow parallel communications and reduce collisions.
- **Routing Table Size:**
  - Nodes only store active routes, conserving memory.
- **Gateway Placement:**
  - Strategic placement of gateway nodes ensures efficient inter-section communication.

## 11. Security & Reliability
- **Authentication:**
  - All control and data messages, including HELLO beacons, should be cryptographically signed (e.g., HMAC or ECC signatures) to prevent spoofing and ensure authenticity.
  - Signed HELLO beacons allow nodes to verify the identity of neighbors before adding them to routing tables, reducing the risk of malicious or rogue nodes flooding the network.
  - Key management can be handled via pre-shared keys (for HMAC) or public/private key pairs (for ECC), depending on the desired security level and ease of deployment.
  - Unsigned or invalid messages are dropped, minimizing unnecessary network load and improving reliability.
  - ESP32 hardware is capable of efficient cryptographic operations, making this approach practical for the platform.

### Message Signing and Authentication Guide

To ensure authenticity and integrity of all control and data messages, including HELLO beacons, the protocol uses cryptographic signatures. This prevents spoofing, replay, and unauthorized participation in the mesh.

**Recommended Approach:**
- Use HMAC (Hash-based Message Authentication Code) with a pre-shared secret for simplicity and speed, or ECC (Elliptic Curve Cryptography) for public/private key signatures if higher security is needed.
- Each message includes a signature field, generated by the sender and verified by the receiver.
- Unsigned or invalid messages are dropped and not processed or forwarded.

**Implementation Steps:**
1. **Key Distribution:**
   - For HMAC: Distribute a shared secret key to all legitimate nodes before deployment.
   - For ECC: Distribute public keys to all nodes; each node keeps its private key secure.
2. **Message Construction:**
   - When sending, compute the signature over the message header and payload.
   - Attach the signature as a field in the message structure.
3. **Verification:**
   - Upon receiving, verify the signature using the shared secret (HMAC) or sender's public key (ECC).
   - Only process messages with valid signatures.
4. **HELLO Beacons:**
   - Each HELLO beacon is signed, allowing nodes to authenticate neighbors before adding them to routing tables.
   - This prevents rogue or malicious nodes from being accepted into the mesh.
5. **Replay Protection:**
   - Include a timestamp or sequence number in each message to prevent replay attacks.

**Example (HMAC with SHA-256):**
```cpp
#include <mbedtls/md.h>

// Compute HMAC-SHA256 signature
void compute_hmac(const uint8_t* key, size_t key_len, const uint8_t* msg, size_t msg_len, uint8_t* out_sig) {
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    mbedtls_md_hmac_starts(&ctx, key, key_len);
    mbedtls_md_hmac_update(&ctx, msg, msg_len);
    mbedtls_md_hmac_finish(&ctx, out_sig);
    mbedtls_md_free(&ctx);
}
````

**Best Practices:**

- Store keys securely in ESP32 flash or use hardware security features.
- Rotate keys periodically if possible.
- Keep signature size minimal to reduce LoRa packet overhead.
- Use efficient cryptographic libraries (e.g., mbedTLS, ArduinoBearSSL).

**Troubleshooting:**

- If nodes are not joining the mesh, check for key mismatches or signature verification errors.
- Monitor for repeated invalid signature attempts as a sign of possible attack.

## 12. Implementation Guidelines

**Hardware:**

- 915 MHz LoRa radios with multi-channel support recommended for gateways.
- ESP32 microcontrollers for all nodes.

**Software:**

- Node firmware will be written in C++ for ESP32 (using Arduino framework or ESP-IDF).
- Use available LoRa libraries (e.g., RadioHead, LMIC, or ESP32 LoRa libraries).
- Modular design for routing, channel management, and application logic.
- Consider FreeRTOS tasks for concurrent operations (neighbor discovery, routing, data handling).

**Testing:**

- Simulate with virtual nodes or ESP32 dev boards before field deployment.
- Monitor network health and adjust parameters (e.g., HELLO interval, TTL) as needed.
- Use serial output or onboard LEDs for debugging and status indication.

## 13. Future Enhancements

- **Mobility Support:**
  - Dynamic section reassignment for moving nodes.
- **QoS:**
  - Prioritization of critical messages.
- **Over-the-Air Updates:**
  - Secure firmware updates via the mesh.

## 14. References

- Perkins, C. E., & Royer, E. M. (1999). Ad hoc On-Demand Distance Vector Routing.
- LoRa Alliance Technical Documents
- [AODV RFC 3561](https://datatracker.ietf.org/doc/html/rfc3561)

---

## 15. Glossary

- **AODV (Ad hoc On-Demand Distance Vector):** A routing protocol for mobile ad hoc networks.
- **GPS (Global Positioning System):** A satellite-based navigation system that provides location and time information.
- **HMAC (Hash-based Message Authentication Code):** A mechanism for message authentication using a cryptographic hash function and a secret key.
- **LoRa (Long Range):** A low-power wide-area network (LPWAN) protocol designed for long-range communication.
- **Mesh Network:** A network topology in which each node relays data for the network, allowing for flexible and resilient communication.
- **Protobuf (Protocol Buffers):** A language-agnostic binary serialization format developed by Google.
- **RSSI (Received Signal Strength Indicator):** A measurement of the power level that an RF device receives from a signal.
- **TTL (Time to Live):** A field in a packet that specifies the maximum time the packet is allowed to exist in the network.

---

## 16. Protocol Buffer Implementation Details âœ… COMPLETED

Our LoRa mesh network uses Protocol Buffers for all message serialization, providing efficient, extensible, and type-safe communication. All protocol definitions are complete and ready for ESP32 implementation.

### Available Protocol Buffer Schemas

**Core Message Wrapper** (`/lora_mesh/v1/messages.proto`):

- `LoRaMessage`: Universal message wrapper with routing, timing, and security fields
- Supports message types: GPS updates, routing control, heartbeats, application data
- Built-in cryptographic signature support

**Routing Protocol** (`/lora_mesh/v1/routing.proto`):

- `RouteRequestPayload`: AODV route discovery messages
- `RouteReplyPayload`: Route establishment responses
- `RouteErrorPayload`: Link failure notifications
- Full support for sequence numbers, hop counts, and route lifetimes

**Geographic Data** (`/lora_mesh/v1/geographic.proto`):

- `GPSCoordinate`: Precise location data with accuracy metrics
- `GeographicBoundary`: Area definitions and containment checks
- Optimized for tactical and emergency use cases

**Security & Node Management** (`/lora_mesh/v1/common.proto`):

- `CryptographicSignature`: Multi-algorithm signing (HMAC, Ed25519, ECDSA)
- `NodeInfo`: Device capabilities and status reporting
- `QoSRequirements`: Service level specifications

### ESP32 Integration Ready

- **nanopb 0.4.9.1 compatible**: All `.options` files tuned for ESP32 memory constraints
- **Compilation tested**: Protocol Buffer C code generates cleanly
- **Memory optimized**: String limits and array sizes configured for embedded use
- **Ready for firmware integration**: Just add the generated C files to your ESP32 project

### Next Steps for Developers

1. Include generated protobuf C files in your ESP32 firmware project
2. Implement message serialization/deserialization using nanopb APIs
3. Add LoRa radio drivers and integrate with the LoRaMessage wrapper
4. Implement the routing algorithms using the defined payload structures
5. Add GPS integration using the GPSCoordinate message format

---

## ðŸ“š Appendix: Example Message Format (for Techies)

```cpp
struct GpsPacket {
    uint8_t type;        // 0x10 = GPS
    uint16_t source;     // Your device ID
    uint32_t timestamp;  // When
    float latitude;
    float longitude;
    float altitude;
    float accuracy;      // Optional
    uint8_t signature[32]; // Security
};
```

---

_For questions or support, contact the protocol designer or project lead._

