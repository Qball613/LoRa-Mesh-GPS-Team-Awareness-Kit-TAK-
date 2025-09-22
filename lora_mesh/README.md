b# Protocol Buffer Build Configuration

This directory contains Protocol Buffer definitions for the LoRa Mesh GPS Team Awareness Kit (TAK).

## Protocol Buffer Files

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

To support multiple logical services and robust routing, all messages include `source_port` and `destination_port` fields. The following port numbers are reserved for core mesh services:

| Port Number | Service/Protocol         | Description                       |
|-------------|--------------------------|-----------------------------------|
| 0           | Reserved                 | Invalid/unspecified               |
| 1           | Routing Control          | AODV, Link State, Route Discovery |
| 2           | Geographic Routing       | Geographic/position-based routing |
| 10          | Network Management       | Node announcements, topology      |
| 20          | GPS Updates              | GPS position reporting            |
| 30          | Text Messaging           | User chat/text messages           |
| 40          | Emergency/Broadcast      | Emergency alerts, broadcast msgs  |
| 1000-1999   | User Applications        | Custom/experimental apps          |

**Note:**
- Ports 1 and 2 should be used for all routing protocol messages (e.g., `RouteRequestPayload`, `RouteReplyPayload`, `RouteErrorPayload`).
- Application messages (e.g., text, GPS, emergency) should use their assigned ports.
- Ports above 1000 are available for user-defined or future services.

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

1. Install Protocol Compiler (protoc):
   ```bash
   # Windows (using chocolatey)
   choco install protoc
   
   # Or download from: https://github.com/protocolbuffers/protobuf/releases
   ```

2. Install nanopb:
   ```bash
   pip install nanopb
   ```

3. Clone nanopb repository for generator:
   ```bash
   git clone https://github.com/nanopb/nanopb.git
   ```

### Build Process

1. **Generate C code for ESP32**:
   ```bash
   # From the proto directory
   python nanopb/generator/protoc --nanopb_out=../src/generated --proto_path=. *.proto
   ```

2. **Alternative using protoc directly**:
   ```bash
   protoc --plugin=protoc-gen-nanopb=nanopb/generator/protoc-gen-nanopb --nanopb_out=../src/generated *.proto
   ```

3. **Generate with custom options** (create nanopb_generator.options):
   ```
   # Maximum string lengths for embedded systems
   *.node_id max_length:16
   *.message_content max_length:256
   *.error_message max_length:128
   
   # Array size limits
   *.neighbors max_count:32
   *.route_path max_count:16
   *.cached_nodes max_count:64
   ```

### Generated Files Structure

After building, you'll get:
```
src/generated/
├── common.pb.h
├── common.pb.c
├── messages.pb.h
├── messages.pb.c
├── routing.pb.h
├── routing.pb.c
├── geographic.pb.h
├── geographic.pb.c
├── config.pb.h
├── config.pb.c
├── metrics.pb.h
└── metrics.pb.c
```

### Integration with ESP32 Build System

Add to your `CMakeLists.txt`:

```cmake
# Include generated protobuf files
set(NANOPB_SRC_ROOT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/components/nanopb)
find_package(Nanopb REQUIRED)

# Add protobuf component
idf_component_register(
    SRCS 
        "src/generated/common.pb.c"
        "src/generated/messages.pb.c"
        "src/generated/routing.pb.c"
        "src/generated/geographic.pb.c"
        "src/generated/config.pb.c"
        "src/generated/metrics.pb.c"
        "src/lora_mesh_main.c"
    INCLUDE_DIRS 
        "src/generated"
        "include"
    REQUIRES 
        nanopb
)
```

### Usage Example

```c
#include "messages.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

// Encoding a GPS update message
void send_gps_update(float lat, float lon) {
    lora_mesh_LoRaMessage message = lora_mesh_LoRaMessage_init_zero;
    
    // Set message type
    message.message_type = lora_mesh_MessageType_GPS_UPDATE;
    message.has_gps_payload = true;
    
    // Set GPS coordinates
    message.gps_payload.position.latitude = lat;
    message.gps_payload.position.longitude = lon;
    message.gps_payload.position.has_altitude = false;
    
    // Set node info
    strcpy(message.header.source_id, get_node_id());
    message.header.timestamp = get_timestamp();
    message.header.message_id = get_next_message_id();
    
    // Encode to buffer
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    
    if (pb_encode(&stream, lora_mesh_LoRaMessage_fields, &message)) {
        // Send via LoRa radio
        lora_send_packet(buffer, stream.bytes_written);
    }
}

// Decoding received message
void handle_received_packet(uint8_t* data, size_t length) {
    lora_mesh_LoRaMessage message = lora_mesh_LoRaMessage_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(data, length);
    
    if (pb_decode(&stream, lora_mesh_LoRaMessage_fields, &message)) {
        switch (message.message_type) {
            case lora_mesh_MessageType_GPS_UPDATE:
                handle_gps_update(&message.gps_payload);
                break;
            case lora_mesh_MessageType_TEXT_MESSAGE:
                handle_text_message(&message.text_payload);
                break;
            case lora_mesh_MessageType_ROUTE_REQUEST:
                handle_route_request(&message.routing_payload.route_request);
                break;
            // ... handle other message types
        }
    }
}
```

### Memory Optimization

For constrained ESP32 environments:

1. **Use callback encoding/decoding** for large arrays:
   ```c
   bool encode_neighbors_callback(pb_ostream_t *stream, const pb_field_t *field, void * const *arg);
   bool decode_neighbors_callback(pb_istream_t *stream, const pb_field_t *field, void **arg);
   ```

2. **Limit field sizes** in nanopb options:
   ```
   NeighborInfo.neighbor_nodes max_count:16
   RouteTable.routes max_count:32
   ```

3. **Use unions** for message payloads to save RAM.

### Validation and Testing

Create unit tests for protocol buffer encoding/decoding:

```c
void test_message_encoding() {
    // Test GPS message
    // Test routing message  
    // Test configuration message
    // Verify round-trip encoding/decoding
}
```

## Message Size Estimates

| Message Type | Typical Size | Max Size |
|--------------|-------------|----------|
| GPS Update | 32 bytes | 64 bytes |
| Text Message | 64 bytes | 256 bytes |
| Route Request | 48 bytes | 128 bytes |
| Route Reply | 56 bytes | 144 bytes |
| Link State | 72 bytes | 256 bytes |
| Network Config | 128 bytes | 512 bytes |

These sizes fit well within LoRa packet limits (51-222 bytes depending on spreading factor).