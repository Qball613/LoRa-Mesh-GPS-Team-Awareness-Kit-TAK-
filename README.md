# LoRa Mesh GPS Team Awareness Kit (TAK)

---

## 🚀 Start Here: What Is This?
A simple, secure, and robust way for your team to share live GPS locations—no cell towers, no Wi-Fi, just small radios and a map! Designed for field teams (airsoft, SAR, outdoor events, etc.) who need to know where everyone is, even in remote areas.

---

## 🗺️ How It Works (In Plain English)
- Each team member carries a small device (ESP32 + LoRa radio + GPS module).
- The device gets your GPS position and sends it out over the air.
- All devices form a mesh network, relaying each other’s locations so everyone stays in sync.
- You can see your teammates’ positions on a map display, phone, or even a simple screen.

---

## 🌟 Key Features
- **Live Team Map:** See where everyone is, in real time.
- **No Infrastructure Needed:** Works anywhere—forests, mountains, urban, or rural.
- **Secure:** All messages are signed, so only your team can join and see locations.
- **Scalable:** Supports 100+ users with smart routing to avoid radio overload.
- **Flexible:** Use for games, search and rescue, events, or any team activity.

---

## 🏁 Quick Start: Building Your Mesh
1. **Assemble Hardware:** ESP32 + LoRa + GPS for each teammate.
2. **Flash Starter Firmware:** Blink an LED, read GPS, send/receive LoRa packets.
3. **Test Range & GPS:** Make sure radios and GPS work in your area.
4. **Follow the Actionable Plan Below!**

---

## 🛠️ Actionable Implementation Plan

Follow these steps to build your LoRa Mesh GPS Team Awareness Kit in a logical, testable sequence. Each phase builds on the last, ensuring a robust and maintainable system.

### Phase 1: Hardware Bring-Up & Basic Communication
- Assemble ESP32 + LoRa + GPS hardware for each node.
- Flash basic firmware: blink LED, read GPS, send/receive raw LoRa packets.
- Validate radio range and GPS accuracy in your environment.

### Phase 2: Message Serialization with Protobufs
- Define core message types in `.proto` files (GPS, control, text, etc.).
- Integrate nanopb and generate C/C++ code for ESP32.
- Implement encode/decode for messages; test serialization and deserialization.

### Phase 3: Cryptographic Signing & Verification
- Implement HMAC or ECC signing for all messages (see 'Cryptography' section).
- Add signature verification on receive; drop invalid messages.
- Test with mismatched keys to ensure security enforcement.

### Phase 4: Node Discovery & Routing Table
- Implement signed HELLO beacons for neighbor discovery.
- Build and maintain a routing table (see 'Routing Table Structure').
- Test neighbor addition/removal and table updates.

### Phase 5: On-Demand Routing (AODV)
- Implement RREQ, RREP, and RERR message handling.
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

## 🔍 How Do I See the Map?
- Connect your device to a phone, tablet, or small screen.
- Use a custom app, TAK/ATAK, or even a web map to view everyone’s positions.
- Alerts can warn you if someone goes out of bounds or loses contact.

---

## 🔒 How Is It Secure?
- Only devices with the right digital keys can join and share locations.
- All messages are signed, so you know they’re real.
- No data leaves the mesh—no cloud, no tracking, just your team.

---

## 🤔 Why Use This?
- Stay safe and coordinated in the field.
- No monthly fees or cell coverage required.
- Works in tough environments where other tech fails.

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

## GPS Data Collection and Transmission (with Protobufs)
- **GPS Module Integration:** Each ESP32 node is equipped with a GPS module (e.g., u-blox, NEO-6M) to obtain latitude, longitude, altitude, and timestamp.
- **Data Packetization:** GPS data is periodically packaged into a Protocol Buffers (protobuf) message and broadcast or routed to other nodes.
- **Update Rate:** The update interval can be configured (e.g., every 2–10 seconds) to balance freshness and network load.

## GPS Data Protobuf Message Format
A typical GPS/location packet is defined in a `.proto` file and compiled for use with nanopb:

```proto
syntax = "proto3";

message GpsPacket {
  uint32 type = 1;         // 0x10 for GPS
  uint32 source = 2;       // Node ID
  uint32 timestamp = 3;    // Unix time
  float latitude = 4;
  float longitude = 5;
  float altitude = 6;
  float accuracy = 7;      // Optional
  bytes signature = 8;     // HMAC-SHA256 or ECC signature
}
```

**C++ Usage Example (with nanopb):**
```cpp
// Fill the GpsPacket struct
gps_packet.type = 0x10;
gps_packet.source = my_node_id;
gps_packet.timestamp = now();
gps_packet.latitude = lat;
gps_packet.longitude = lon;
gps_packet.altitude = alt;
gps_packet.accuracy = acc;
// Serialize with nanopb
pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
pb_encode(&GpsPacket_msg, &gps_packet, &stream);
// Sign the serialized buffer and add signature
compute_hmac(key, key_len, buffer, stream.bytes_written, gps_packet.signature);
```

- **Transmission:** Send the serialized and signed buffer over LoRa.
- **Reception:** On receive, verify the signature, then decode the protobuf message to extract GPS data.

**Tip:** Protobufs make it easy to add new fields (e.g., battery level, status) in the future without breaking compatibility.

## Visualization and Team Awareness
- **Display:** Each node can be connected to a display (e.g., OLED, TFT, or mobile device via Bluetooth/serial) to show a live map of team positions.
- **Integration:** Data can be exported to TAK/ATAK, QGIS, or custom mapping apps for advanced visualization.
- **Alerts:** The system can generate alerts for out-of-bounds, lost comms, or proximity events.

## Security and Privacy
- All GPS/location packets are signed to prevent spoofing and tampering.
- Only authenticated nodes can participate and view team locations.
- Location data is not sent to the cloud or external servers—mesh only.

---

## 2. Network Architecture
- **Nodes:** Each user is a node in the mesh network.
- **Mesh Topology:** Nodes form a dynamic, self-healing mesh network.
- **Sections/Channels:** The network is logically divided into sections, each mapped to a LoRa channel or spreading factor. Gateway nodes bridge sections.

## 3. Node Addressing & Discovery
- **Unique IDs:** Each node is assigned a unique address (e.g., 16-bit integer).
- **Neighbor Discovery:** Nodes periodically broadcast signed 'HELLO' beacons to discover and authenticate direct neighbors, ensuring only legitimate nodes are added to routing tables.
- **Section Assignment:** Nodes are assigned to sections based on location or function.

## 4. Routing Protocol (AODV for LoRa)
- **On-Demand Routing:** Routes are established only when needed, minimizing control traffic.
- **Routing Table:** Each node maintains a table of known routes (destination, next hop, hop count, sequence number, section).
- **Route Discovery:** Initiated via RREQ (Route Request) messages; routes are confirmed with RREP (Route Reply).
- **Route Maintenance:** Broken links trigger RERR (Route Error) messages and route invalidation.
- **TTL & Sequence Numbers:** Used to limit propagation and prevent loops.

## In-Depth: Routing Algorithm (AODV for LoRa Mesh)

This section explains how the mesh finds and maintains routes, how messages travel, and how the protocol is optimized for LoRa’s unique constraints.

### 1. Route Discovery (How a Path is Found)
- When a node (e.g., Node A) wants to send data to another node (e.g., Node D) and has no route, it starts a **Route Request (RREQ)**.
- The RREQ is broadcast to neighbors, who forward it (with a hop count and unique sequence number) until it reaches the destination or a node with a valid route.
- The destination (or an intermediate node with a fresh route) sends a **Route Reply (RREP)** back along the reverse path.
- Each node along the way updates its routing table with the next hop to the destination.

**Example Flow:**
1. Node A wants to send to Node D, but has no route.
2. Node A broadcasts RREQ (A→*, seq=101).
3. Node B and C receive RREQ, forward it (if not seen before).
4. Node D receives RREQ, sends RREP (D→C→A).
5. Node A now has a route to D (via C).

### 2. Sending Data
- Once a route exists, Node A sends the data packet to its next hop (Node C), which forwards it to D.
- If the route breaks, a new discovery is triggered.

### 3. Route Maintenance & Error Handling
- Nodes periodically send signed HELLO beacons to confirm neighbors are alive.
- If a node detects a broken link (e.g., no HELLO from next hop), it sends a **Route Error (RERR)** to affected nodes.
- Affected nodes remove the route and, if needed, start a new RREQ.

### 4. LoRa-Specific Optimizations
- **Minimize Broadcasts:** Use small TTL (Time To Live) for RREQs to limit their range.
- **Sequence Numbers:** Prevent loops and stale routes.
- **Route Caching:** Nodes can cache routes for frequently-used destinations.
- **Channel Awareness:** Only forward RREQs within the same section/channel unless acting as a gateway.
- **Duty Cycling:** Schedule transmissions to avoid collisions and save power.

### 5. Pseudocode Example: Route Discovery
```cpp
// On data send request
if (!route_exists(dest)) {
    send_RREQ(dest);
    wait_for_RREP();
}
send_data(dest, payload);

// On RREQ receive
if (already_seen(RREQ)) return;
update_reverse_route(RREQ.source, previous_hop);
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
1. A → (RREQ) → B → (RREQ) → C → (RREQ) → D
2. D → (RREP) → C → (RREP) → B → (RREP) → A
3. A → (DATA) → B → C → D
```

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
```

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
## 📚 Appendix: Example Message Format (for Techies)
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

*For questions or support, contact the protocol designer or project lead.*
