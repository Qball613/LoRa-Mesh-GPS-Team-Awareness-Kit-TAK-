# LoRa Mesh GPS TAK Firmware

ESP32-S3 firmware for the LoRa Mesh GPS Team Awareness Kit. Implements AODV routing protocol, AES-256-GCM authenticated encryption, and GPS position sharing over LoRa mesh networks.

## Features

- **AODV Routing Protocol**: On-demand route discovery with multi-hop forwarding
- **AES-256-GCM Encryption**: Full message confidentiality with authenticated encryption
- **GPS Management**: Hardware GPS, static coordinates, or manual positioning
- **Text Messaging**: Unicast and broadcast messaging across the mesh
- **Serial CLI**: Command-line interface for testing and control
- **Protocol Buffers**: Efficient message serialization with nanopb
- **Multi-Board Support**: Unified codebase for LilyGO T3S3 and T-Deck devices
- **Self-Healing Mesh Sync**: Version-tracked differential synchronization
- **GPS-Less Node Support**: Roster sync for nodes without GPS (indoor use)

## Mesh Synchronization

The mesh implements a self-healing synchronization protocol to ensure all nodes have consistent state:

### Version Tracking

- Each node maintains a `mesh_version` counter
- Version increments ONLY when node contributes NEW data:
  - Sending GPS position update
  - Sending text message
  - Discovering multi-hop route (2+ hops)
- Version does NOT increment for:
  - Hearing a neighbor's HELLO (prevents flood when many nodes hear same new node)
  - Adding 1-hop direct route (same reason)

### Join Protocol

1. **New node broadcasts HELLO** with mesh_version=0
2. **All neighbors add it locally** (no version increment, no flood)
3. **Best RSSI neighbor responds** with neighbor update (local topology)
4. **Edge responder also responds** with distant nodes (network extent)
5. **Edge responder broadcasts LSA** (Link State Advertisement) with TTL=3
6. **ContentSync follows** with positions, messages, and roster

### Roster Support (GPS-Less Nodes)

Nodes without GPS are still visible in team awareness:
- ContentSync includes `roster[]` field with active node IDs lacking GPS
- GPS-less nodes appear in team roster, just without map position
- AODV routing works without GPS (routes by node ID, not location)

### Differential Sync

When a node detects it's behind (incoming `mesh_version` > local):

1. **Request sync** from the ahead node (rate limited: max 1 per 60s per peer)
2. **Responder sends ContentSync** with positions, messages, roster
3. **Receiver updates** neighbor table and adopts mesh_version

### Flood Prevention

- **Designated responders only**: Best RSSI + edge node respond to JOIN
- **Rate limiting**: ContentSync push max 1 per 30s per peer
- **Empty packet prevention**: Skip sending if nothing useful to share

## Hardware Support

### LilyGO T3-S3

- ESP32-S3 microcontroller
- SX1262 LoRa radio (915 MHz)
- ST7789 TFT display (170×320)
- Optional GPS module (UART)
- Bluetooth (for gateway mode)

### LilyGO T-Deck

- ESP32-S3 microcontroller
- Optional SX1262 LoRa radio
- ST7789 TFT display (320×240)
- QWERTY keyboard
- Optional GPS module

## Quick Start

### 1. Install PlatformIO

```bash
# Install PlatformIO CLI
pip install platformio

# Or use VS Code with PlatformIO extension
```

### 2. Clone and Build

```bash
cd firmware/
pio run -e t3s3-gateway      # For T3S3 gateway
pio run -e t3s3-standalone   # For T3S3 standalone
pio run -e tdeck-lora        # For T-Deck with LoRa
```

### 3. Upload Firmware

```bash
# Upload to T3S3
pio run -e t3s3-gateway -t upload

# Monitor serial output
pio device monitor -b 115200
```

### 4. Initial Configuration

First boot will generate a unique node ID and HMAC key. **Copy the HMAC key to all nodes in your mesh!**

```text
Node ID: NODE_A1B2C3D4
HMAC Key (HEX): 1A2B3C4D...

Type 'help' for available commands
```

## Serial Commands

### GPS Commands

```text
set_manual_gps <lat> <lon>   - Set manual GPS coordinates (for testing)
set_static_gps <lat> <lon>   - Set static GPS coordinates (persists in NVS)
set_gps_mode <0|1|2>         - Set GPS mode (0=hardware, 1=static, 2=manual)
send_gps                     - Broadcast current GPS position
show_gps                     - Display GPS information
```

### Messaging Commands

```text
send_msg <dest> <text>       - Send text message to destination node
send_msg BROADCAST <text>    - Broadcast message to all nodes
```

### Routing Commands

```text
show_routes                  - Display routing table
show_neighbors               - Display neighbor table (1-hop)
show_stats                   - Display statistics
```

### System Commands

```text
set_node_id <id>             - Set custom node ID (requires restart)
generate_key                 - Generate new HMAC key (copy to all nodes!)
help                         - Show help message
```

## Testing Without GPS Hardware

For initial testing without physical GPS modules:

```text
> set_manual_gps 37.7749 -122.4194
OK: Manual GPS set to 37.774900, -122.419400

> set_gps_mode 2
OK: GPS mode set to 2

> show_gps
=== GPS Information ===
GPS Mode: 2 (Manual)
Has Fix: Yes
Latitude:  37.774900
Longitude: -122.419400
Altitude:  0.0 m
```

## Multi-Radio Testing Workflow

### 2-Node Test (Direct Link)

```text
# Node A
> set_manual_gps 37.7749 -122.4194
> set_node_id NODE_A
> send_gps

# Node B (separate computer)
> set_manual_gps 37.7750 -122.4195
> set_node_id NODE_B
> send_msg NODE_A Hello from B
```

### 3-Node Test (Multi-Hop)

```text
# Node A (originator)
> send_msg NODE_C Test multi-hop

# Node B (relay) - should forward message
# Watch logs for "Message forwarded"

# Node C (destination) - should receive message
# Watch for "TEXT MESSAGE RECEIVED"
```

### 6-Node Mesh Test

- Deploy nodes in different locations
- Use `show_routes` to verify route discovery
- Use `show_neighbors` to see 1-hop connectivity
- Send messages between non-adjacent nodes
- Monitor AODV route discovery (RREQ/RREP logs)

## Build Environments

### t3s3-gateway

- LoRa + Bluetooth + TFT + GPS
- Bluetooth gateway for Python backend
- Minimal TFT status display

### t3s3-standalone

- LoRa + TFT + GPS
- Full standalone operation
- No Bluetooth gateway

### tdeck-lora

- LoRa + Keyboard + TFT + GPS
- Full keyboard input
- Standalone operation

### tdeck-standalone

- Keyboard + TFT + Bluetooth (no LoRa)
- Bluetooth-only gateway device
- Connects to LoRa nodes via BLE

## Project Structure

```text
firmware/
├── platformio.ini          # Build configuration
├── include/                # Header files
│   ├── config.h           # Global configuration
│   ├── node_id.h          # Node identification
│   ├── security.h         # HMAC-SHA256 security
│   ├── gps_manager.h      # GPS management
│   ├── lora_radio.h       # LoRa radio interface
│   ├── routing_engine.h   # AODV routing protocol
│   ├── serial_cli.h       # Command-line interface
│   └── protobuf_handler.h # Protocol Buffer wrapper
├── src/                   # Implementation files
│   ├── main.cpp           # Main firmware entry
│   ├── node_id.cpp
│   ├── security.cpp
│   ├── gps_manager.cpp
│   ├── lora_radio.cpp
│   ├── routing_engine.cpp
│   ├── serial_cli.cpp
│   └── protobuf_handler.cpp
└── lib/                   # External libraries
    └── (nanopb protobuf code will go here)
```

## Configuration

Edit `include/config.h` to customize:

```cpp
// LoRa parameters
#define LORA_FREQUENCY_MHZ 915.0
#define LORA_SPREADING_FACTOR 7
#define LORA_TX_POWER_DBM 20

// Routing parameters
#define MAX_ROUTING_TABLE_SIZE 100
#define HELLO_BEACON_INTERVAL_MS 15000
#define ROUTE_LIFETIME_MS 30000

// GPS parameters
#define GPS_UPDATE_INTERVAL_MS 10000
```

## Security

The firmware uses AES-256-GCM authenticated encryption for full message confidentiality and integrity.

### Encryption Details

- **Algorithm**: AES-256-GCM (Galois/Counter Mode)
- **Key Size**: 256 bits (32 bytes)
- **Nonce Size**: 96 bits (12 bytes, randomly generated per message)
- **Auth Tag Size**: 128 bits (16 bytes)
- **Hardware Acceleration**: Uses ESP32-S3 crypto accelerator

### Wire Format

Each encrypted LoRa packet has the following structure:

```text
[12-byte nonce][encrypted protobuf payload][16-byte GCM auth tag]
```

### Key Management

1. **Generate Key on First Node**:

   ```text
   > generate_key
   OK: New encryption key generated
   Key (HEX): 1A2B3C4D5E6F...
   ```

2. **Copy Key to All Nodes**:
   - All nodes in the mesh MUST share the same 256-bit key
   - Key is stored in NVS flash (persists across reboots)
   - Messages with invalid GCM tags are silently dropped

3. **Replay Protection**:
   - Message IDs tracked for 5 minutes (100 message limit)
   - Duplicate messages with same ID are dropped

## Troubleshooting

### LoRa Initialization Fails

```text
ERROR: LoRa init failed, code: -2
```

- Check SPI pin definitions in platformio.ini
- Verify LoRa module power supply
- Check SX1262 BUSY/DIO1 connections

### No GPS Fix

```text
GPS Mode: 0 (Hardware)
Has Fix: No
Satellites: 0
```

- GPS needs outdoor location or near window
- Wait 30-60 seconds for satellite acquisition
- Use manual mode for indoor testing

### No Neighbors Detected

```text
Total neighbors: 0
```

- Check LoRa frequency (must match all nodes)
- Verify HMAC keys match on all nodes
- Increase TX power or reduce distance
- Check `show_stats` for RX errors

### Messages Not Forwarding

- Use `show_routes` to verify route exists
- Check TTL hasn't expired (default: 10 hops)
- Monitor logs for "Message forwarded"
- Verify intermediate node is powered on

## Message Types

| Type | Name | Description |
|------|------|-------------|
| 0 | UNSPECIFIED | Invalid/unset message type |
| 1 | GPS_UPDATE | GPS position update |
| 2 | TEXT_MESSAGE | User text message |
| 3 | HELLO | Periodic beacon with node info |
| 5 | ACKNOWLEDGMENT | Message acknowledgment |
| 6 | EMERGENCY | Emergency alert |
| 7 | NETWORK_DISCOVERY | Neighbor/topology sharing |
| 8 | CONTENT_SYNC | Bundled positions + messages + roster |
| 10 | ROUTE_REQUEST | AODV route discovery |
| 11 | ROUTE_REPLY | AODV route response |
| 12 | ROUTE_ERROR | Link failure notification |
| 13 | LINK_STATE_ADVERTISEMENT | Proactive topology update |
| 30 | FRAGMENT | Fragmented large message part |

## Next Steps

### TODO: Python Backend Integration

- Implement Meshtastic-style serial framing
- Create Python library for serial/Bluetooth connection
- Extend `routing_simulation/visualization.py` for live data
- Add bidirectional command injection

### TODO: TFT Display UI

- Implement roster display for standalone mode
- Add message inbox viewer
- Create keyboard input handler for T-Deck
- Display routing statistics

## Performance

Initial testing metrics (simulated):

- **Route Discovery**: <3 seconds for 6-node mesh
- **Message Latency**: ~100-500ms per hop
- **Throughput**: ~1-2 messages/second per node (SF7)
- **Range**: 500m-2km (line of sight, terrain dependent)

## Contributing

Refer to main project README for contribution guidelines.

## License

See LICENSE file in project root.
