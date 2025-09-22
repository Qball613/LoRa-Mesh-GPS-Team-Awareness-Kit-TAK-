# Protocol Buffer Definitions for LoRa Mesh GPS TAK

This directory contains Protocol Buffer definitions for the LoRa Mesh GPS Team Awareness Kit (TAK). The protocol buffers are organized by domain to facilitate modular development and maintenance.

## File Organization

The protocol buffers are organized into domain-specific files for better maintainability and code reloading:

- **`common.proto`** - Shared data structures, enums, and base types used across all protocols
- **`messages.proto`** - Core message wrapper and basic application messages (GPS, Hello, Text, Control, Emergency)  
- **`geographic.proto`** - Geographic routing specific messages and location-based services
- **`routing.proto`** - AODV and routing protocol messages (Route Request/Reply/Error, Link State)
- **`config.proto`** - Network configuration, policies, and system settings
- **`metrics.proto`** - Network monitoring, performance metrics, and diagnostics

This structure allows you to:
- **Import only what you need** - Geographic routing modules only need `geographic.proto` + `common.proto`
- **Reload code easily** - Changes to routing don't affect geographic or messaging components
- **Develop independently** - Different teams can work on different protocol domains
- **Maintain separation** - Each file handles its own message types and enums

## Core Message Types

### Application Messages (messages.proto)
- `GPSUpdatePayload` - GPS position updates with satellite info
- `HelloPayload` - Neighbor discovery beacons with network metrics
- `TextMessagePayload` - User text communications with priority handling
- `ControlMessagePayload` - System control commands (ping, reboot, config)
- `NetworkDiscoveryPayload` - Network topology discovery
- `EmergencyPayload` - Critical alerts and emergency communications
- `AckPayload` - Message acknowledgments with error reporting

### Geographic Routing (geographic.proto)  
- `GeographicBeaconPayload` - Position-aware routing beacons
- `PositionUpdatePayload` - Predictive position updates with movement patterns
- `PredictedPosition` - Future position predictions for mobile nodes
- `MovementPattern` - Movement behavior detection and waypoint tracking

### Routing Protocol (routing.proto)
- `AODVRouteRequest` / `AODVRouteReply` / `AODVRouteError` - AODV protocol messages
- `LinkStateAdvertisement` - Link state routing updates
- `RoutingTable` - Route table management and synchronization

## ESP32/nanopb Compatibility

✅ **All protocol buffers are verified compatible with ESP32 and nanopb**

The protocol definitions include nanopb optimization files (`.options`) for memory-constrained environments:
- String size limits appropriate for ESP32 memory
- Array count limits for neighbor lists and route tables  
- Optimized field sizing for LoRa packet constraints

## Bluetooth Protocol Support

The protocol buffers include support for connecting to the radio via Bluetooth for both configuration and communication. This enables:

- Wireless configuration of radio/network parameters from a mobile device or PC
- Secure session-based communication over Bluetooth (e.g., for diagnostics, messaging, or bridging to LoRa)

### Bluetooth Message Types (in `common.proto`)

- `BluetoothConnectionRequest` / `BluetoothConnectionResponse`: Initiate and accept/reject Bluetooth sessions (for config or communication)
- `BluetoothConfigCommand` / `BluetoothConfigResponse`: Set, get, reset, or update radio/network parameters
- `BluetoothDataMessage`: Send/receive data (raw, protobuf, text, or file) over an established Bluetooth session
- Enums for connection type, config type, and data type

#### Example Use Cases

- **Configuration:**
    - Connect via Bluetooth, send `BluetoothConfigCommand` to set frequency, power, or network ID
    - Receive `BluetoothConfigResponse` for success/failure
- **Communication:**
    - Connect via Bluetooth, send/receive `BluetoothDataMessage` for diagnostics, bridging, or direct messaging

Bluetooth support enables easy field configuration and flexible integration with mobile apps or PC tools.

## Standard Port Numbers

All messages include `source_port` and `destination_port` fields to support multiple logical services and robust routing:

| Port | Service | Description |
|------|---------|-------------|
| 0 | Reserved | Invalid/unspecified |
| 1 | Routing Control | AODV, Link State, Route Discovery |
| 2 | Geographic Routing | Position-based routing services |
| 3 | Hello/Discovery | Neighbor discovery beacons |
| 4 | Control/Management | System control and network management |
| 10 | Network Management | Node announcements, topology |
| 20 | GPS Updates | Position reporting and location services |
| 30 | Text Messaging | User communications |
| 40 | Emergency/Broadcast | Critical alerts and emergency messages |
| 1000+ | User Applications | Custom/experimental applications |

### Message Type Organization

The `MessageType` enum in messages.proto organizes message types by domain:

```protobuf
enum MessageType {
  // Basic application messages (0-9)
  MESSAGE_TYPE_GPS_UPDATE = 1;
  MESSAGE_TYPE_TEXT_MESSAGE = 2;
  MESSAGE_TYPE_HELLO = 3;
  MESSAGE_TYPE_CONTROL_MESSAGE = 4;
  MESSAGE_TYPE_ACKNOWLEDGMENT = 5;
  MESSAGE_TYPE_EMERGENCY = 6;
  MESSAGE_TYPE_NETWORK_DISCOVERY = 7;
  
  // Routing protocol messages (10-19)
  MESSAGE_TYPE_ROUTE_REQUEST = 10;
  MESSAGE_TYPE_ROUTE_REPLY = 11; 
  MESSAGE_TYPE_ROUTE_ERROR = 12;
  MESSAGE_TYPE_LINK_STATE_ADVERTISEMENT = 13;
  
  // Geographic routing messages (20-29)
  MESSAGE_TYPE_GEOGRAPHIC_BEACON = 20;
  MESSAGE_TYPE_POSITION_UPDATE = 21;
}
```

Example usage:
```c
// Route request for GPS service
message.source_port = 20; // GPS Updates
message.destination_port = 1; // Routing Control
```

- **common.proto** - Common data structures, enums, and base types used across all message types
- **messages.proto** - Application-level message definitions for user communications
- **routing.proto** - Routing protocol message definitions (AODV, Link State)  
- **geographic.proto** - Geographic routing specific messages and location services
- **config.proto** - Network configuration and policy definitions
- **metrics.proto** - Network monitoring and performance metrics

## Building for ESP32 with nanopb

### Prerequisites

1. **Install Protocol Compiler (protoc)**:
   ```bash
   # Windows (using chocolatey)
   choco install protoc
   
   # Or download from: https://github.com/protocolbuffers/protobuf/releases
   ```

2. **Install nanopb for Python**:
   ```bash
   pip install nanopb protobuf
   ```

### Build Process

1. **Compile all proto files**:
   ```bash
   # Standard protoc compilation (for validation)
   protoc --proto_path=. --python_out=. lora_mesh/v1/*.proto
   ```

2. **Generate nanopb C code for ESP32**:
   ```bash
   # Generate descriptor files with dependencies
   protoc --proto_path=. --include_imports --descriptor_set_out=messages.pb lora_mesh/v1/messages.proto
   protoc --proto_path=. --include_imports --descriptor_set_out=geographic.pb lora_mesh/v1/geographic.proto  
   protoc --proto_path=. --include_imports --descriptor_set_out=routing.pb lora_mesh/v1/routing.proto
   
   # Generate nanopb C files  
   python -m nanopb_generator -I lora_mesh/v1 -D generated/ messages.pb
   python -m nanopb_generator -I lora_mesh/v1 -D generated/ geographic.pb
   python -m nanopb_generator -I lora_mesh/v1 -D generated/ routing.pb
   ```

3. **Nanopb Options Files**:
   
   The project includes `.options` files for ESP32 memory optimization:
   - `messages.options` - Core message size limits
   - `common.options` - Shared data structure limits  
   - `geographic.options` - Geographic routing array limits
   - `routing.options` - Routing table size constraints
   
   Example options:
   ```
   # String size limits for ESP32
   *.message_id max_size:32
   *.node_id max_size:16
   TextMessagePayload.text max_size:256
   
   # Array limits for memory efficiency
   HelloPayload.known_neighbors max_count:16
   GeographicBeaconPayload.geographic_neighbors max_count:8
   RoutingTable.routes max_count:32
   ```

### Generated Files Structure

After building, you'll get ESP32-compatible C code:
```
generated/
├── lora_mesh/v1/
│   ├── common.pb.h        # Shared data structures
│   ├── common.pb.c
│   ├── messages.pb.h      # Core messaging
│   ├── messages.pb.c  
│   ├── geographic.pb.h    # Geographic routing
│   ├── geographic.pb.c
│   └── routing.pb.h       # Routing protocols  
│       routing.pb.c
```

### Integration with ESP32 Build System

Add to your ESP-IDF `CMakeLists.txt`:

```cmake
# Include nanopb component
find_package(Nanopb REQUIRED)

# Add protobuf sources
idf_component_register(
    SRCS 
        "generated/lora_mesh/v1/common.pb.c"
        "generated/lora_mesh/v1/messages.pb.c"
        "generated/lora_mesh/v1/geographic.pb.c"
        "generated/lora_mesh/v1/routing.pb.c"
        "src/lora_mesh_main.c"
    INCLUDE_DIRS 
        "generated"
        "include"
    REQUIRES 
        nanopb
)
```

### Usage Example - GPS Update

```c
#include "lora_mesh/v1/messages.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Send GPS position update
void send_gps_update(float lat, float lon, float alt) {
    // Create main message wrapper
    lora_mesh_v1_LoRaMessage message = lora_mesh_v1_LoRaMessage_init_zero;
    
    // Set message header
    strcpy(message.message_id, generate_message_id());
    strcpy(message.source_id, get_node_id());  
    strcpy(message.destination_id, "BROADCAST");
    message.message_type = lora_mesh_v1_MessageType_MESSAGE_TYPE_GPS_UPDATE;
    message.timestamp = get_timestamp_ms();
    message.source_port = 20; // GPS Updates port
    message.destination_port = 0; // Broadcast
    
    // Create GPS payload
    lora_mesh_v1_GPSUpdatePayload gps_payload = lora_mesh_v1_GPSUpdatePayload_init_zero;
    gps_payload.position.latitude = lat;
    gps_payload.position.longitude = lon;  
    gps_payload.position.altitude = alt;
    gps_payload.position.timestamp = get_timestamp_ms();
    gps_payload.satellites_used = get_satellite_count();
    gps_payload.accuracy = get_position_accuracy();
    
    // Serialize GPS payload to message payload bytes
    pb_ostream_t payload_stream = pb_ostream_from_buffer(message.payload.bytes, sizeof(message.payload.bytes));
    if (pb_encode(&payload_stream, lora_mesh_v1_GPSUpdatePayload_fields, &gps_payload)) {
        message.payload.size = payload_stream.bytes_written;
        
        // Serialize main message
        uint8_t buffer[256];
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        
        if (pb_encode(&stream, lora_mesh_v1_LoRaMessage_fields, &message)) {
            // Send via LoRa radio
            lora_send_packet(buffer, stream.bytes_written);
            ESP_LOGI(TAG, "GPS update sent: %.6f, %.6f", lat, lon);
        }
    }
}

// Handle received messages
void handle_received_packet(uint8_t* data, size_t length) {
    lora_mesh_v1_LoRaMessage message = lora_mesh_v1_LoRaMessage_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(data, length);
    
    if (pb_decode(&stream, lora_mesh_v1_LoRaMessage_fields, &message)) {
        ESP_LOGI(TAG, "Received %s from %s", 
                 get_message_type_name(message.message_type),
                 message.source_id);
        
        switch (message.message_type) {
            case lora_mesh_v1_MessageType_MESSAGE_TYPE_GPS_UPDATE: {
                lora_mesh_v1_GPSUpdatePayload gps;
                pb_istream_t payload_stream = pb_istream_from_buffer(message.payload.bytes, message.payload.size);
                if (pb_decode(&payload_stream, lora_mesh_v1_GPSUpdatePayload_fields, &gps)) {
                    update_node_position(message.source_id, &gps.position);
                }
                break;
            }
            case lora_mesh_v1_MessageType_MESSAGE_TYPE_HELLO: {
                lora_mesh_v1_HelloPayload hello;
                pb_istream_t payload_stream = pb_istream_from_buffer(message.payload.bytes, message.payload.size);
                if (pb_decode(&payload_stream, lora_mesh_v1_HelloPayload_fields, &hello)) {
                    update_neighbor_info(&hello);
                }
                break;
            }
            case lora_mesh_v1_MessageType_MESSAGE_TYPE_ROUTE_REQUEST: {
                // Handle in routing.proto domain
                handle_route_request(&message);
                break;
            }
            // ... handle other message types
        }
    }
}
```

### Memory Optimization for ESP32

The protocol buffers are optimized for embedded systems:

1. **Size-Limited Fields**: All strings and arrays have maximum sizes defined in `.options` files
2. **Optional Fields**: Use optional fields to minimize packet size  
3. **Efficient Enums**: Compact enum values for message types and commands
4. **Modular Loading**: Import only the protocols you need

```c
// Memory-efficient neighbor discovery
void send_hello_beacon() {
    lora_mesh_v1_HelloPayload hello = lora_mesh_v1_HelloPayload_init_zero;
    
    // Only populate fields you need
    hello.node_info.node_id = get_node_id();
    hello.battery_level = get_battery_percentage();
    hello.neighbor_count = get_direct_neighbor_count();
    
    // Known neighbors limited to 16 entries (see messages.options)
    int neighbor_count = min(get_neighbor_count(), 16);
    for (int i = 0; i < neighbor_count; i++) {
        strcpy(hello.known_neighbors[i], get_neighbor_id(i));
    }
    hello.known_neighbors_count = neighbor_count;
    
    send_message(MESSAGE_TYPE_HELLO, &hello, sizeof(hello));
}
```

### Validation and Testing

Test protocol buffer encoding/decoding:

```c
#include "unity.h" // ESP32 unit testing framework

void test_gps_message_roundtrip() {
    // Create original message
    lora_mesh_v1_GPSUpdatePayload original = lora_mesh_v1_GPSUpdatePayload_init_zero;
    original.position.latitude = 40.7128;
    original.position.longitude = -74.0060;
    original.satellites_used = 8;
    original.accuracy = 3.5;
    
    // Encode
    uint8_t buffer[128];
    pb_ostream_t out_stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    TEST_ASSERT_TRUE(pb_encode(&out_stream, lora_mesh_v1_GPSUpdatePayload_fields, &original));
    
    // Decode
    lora_mesh_v1_GPSUpdatePayload decoded = lora_mesh_v1_GPSUpdatePayload_init_zero;
    pb_istream_t in_stream = pb_istream_from_buffer(buffer, out_stream.bytes_written);
    TEST_ASSERT_TRUE(pb_decode(&in_stream, lora_mesh_v1_GPSUpdatePayload_fields, &decoded));
    
    // Verify
    TEST_ASSERT_FLOAT_WITHIN(0.0001, original.position.latitude, decoded.position.latitude);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, original.position.longitude, decoded.position.longitude);
    TEST_ASSERT_EQUAL_UINT32(original.satellites_used, decoded.satellites_used);
}
```

## Message Size Estimates

Optimized for LoRa packet size constraints (51-222 bytes depending on spreading factor):

| Message Type | Typical Size | Max Size | Use Case |
|--------------|-------------|----------|----------|
| GPS Update | 28-35 bytes | 64 bytes | Position reporting |  
| Hello Beacon | 24-48 bytes | 96 bytes | Neighbor discovery |
| Text Message | 32-80 bytes | 256 bytes | User communications |
| Route Request | 32-52 bytes | 128 bytes | Path discovery |
| Route Reply | 28-48 bytes | 144 bytes | Route establishment |
| Geographic Beacon | 48-72 bytes | 192 bytes | Position-aware routing |
| Emergency Alert | 64-128 bytes | 256 bytes | Critical communications |
| Control Command | 16-32 bytes | 128 bytes | System management |

### LoRa Spreading Factor Compatibility

| Spreading Factor | Max Payload | Suitable Messages |
|------------------|-------------|-------------------|
| SF7 | 222 bytes | All message types |
| SF8 | 222 bytes | All message types |  
| SF9 | 123 bytes | GPS, Hello, Route, Control |
| SF10 | 59 bytes | GPS, Hello, Control |
| SF11 | 59 bytes | GPS, Hello, Control |
| SF12 | 51 bytes | GPS, Hello (minimal) |

The protocol buffer design ensures efficient bandwidth usage across all LoRa configurations.

## Development Workflow

1. **Modify Protocol Definitions**: Edit the appropriate `.proto` file for your domain
2. **Update Options**: Adjust `.options` files if you change array sizes or string lengths
3. **Compile and Test**: Run `protoc` compilation to validate syntax
4. **Generate ESP32 Code**: Use nanopb to generate embedded C code
5. **Integrate and Test**: Add to ESP32 project and run unit tests

This organized structure makes it easy to maintain and extend the LoRa mesh protocol definitions while ensuring optimal performance on resource-constrained devices.