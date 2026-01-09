# LoRa Mesh GPS TAK Firmware

ESP32-S3 firmware for the LoRa Mesh GPS Team Awareness Kit. Implements AODV routing protocol, HMAC-SHA256 message authentication, and GPS position sharing over LoRa mesh networks.

## Features

- **AODV Routing Protocol**: On-demand route discovery with multi-hop forwarding
- **HMAC-SHA256 Security**: Message signing and verification using ESP32-S3 crypto accelerator
- **GPS Management**: Hardware GPS, static coordinates, or manual positioning
- **Text Messaging**: Unicast and broadcast messaging across the mesh
- **Serial CLI**: Command-line interface for testing and control
- **Protocol Buffers**: Efficient message serialization (placeholder implementation)
- **Multi-Board Support**: Unified codebase for LilyGO T3S3 and T-Deck devices

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

```
Node ID: NODE_A1B2C3D4
HMAC Key (HEX): 1A2B3C4D...

Type 'help' for available commands
```

## Serial Commands

### GPS Commands
```
set_manual_gps <lat> <lon>   - Set manual GPS coordinates (for testing)
set_static_gps <lat> <lon>   - Set static GPS coordinates (persists in NVS)
set_gps_mode <0|1|2>         - Set GPS mode (0=hardware, 1=static, 2=manual)
send_gps                     - Broadcast current GPS position
show_gps                     - Display GPS information
```

### Messaging Commands
```
send_msg <dest> <text>       - Send text message to destination node
send_msg BROADCAST <text>    - Broadcast message to all nodes
```

### Routing Commands
```
show_routes                  - Display routing table
show_neighbors               - Display neighbor table (1-hop)
show_stats                   - Display statistics
```

### System Commands
```
set_node_id <id>             - Set custom node ID (requires restart)
generate_key                 - Generate new HMAC key (copy to all nodes!)
help                         - Show help message
```

## Testing Without GPS Hardware

For initial testing without physical GPS modules:

```
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
```
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
```
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

```
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

### HMAC Key Management

1. **Generate Key on First Node**:
   ```
   > generate_key
   OK: New HMAC key generated
   HMAC Key (HEX): 1A2B3C4D5E6F...
   ```

2. **Copy Key to All Nodes**:
   - All nodes in the mesh MUST share the same HMAC key
   - Key is stored in NVS flash (persists across reboots)
   - Messages with invalid signatures are dropped

3. **Replay Protection**:
   - Sequence numbers prevent message replay
   - Timestamp validation (60-second tolerance)
   - Per-node sequence tracking

## Troubleshooting

### LoRa Initialization Fails
```
ERROR: LoRa init failed, code: -2
```
- Check SPI pin definitions in platformio.ini
- Verify LoRa module power supply
- Check SX1262 BUSY/DIO1 connections

### No GPS Fix
```
GPS Mode: 0 (Hardware)
Has Fix: No
Satellites: 0
```
- GPS needs outdoor location or near window
- Wait 30-60 seconds for satellite acquisition
- Use manual mode for indoor testing

### No Neighbors Detected
```
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

## Next Steps

### TODO: Protocol Buffer Integration
The current implementation uses placeholder binary serialization. To integrate actual nanopb-generated Protocol Buffers:

1. Copy generated `.pb.h` and `.pb.c` files from `../lora_mesh/v1/` to `lib/nanopb/`
2. Update `#include` statements in `protobuf_handler.h`
3. Replace encode/decode functions with nanopb API calls
4. Test with real protobuf messages

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
