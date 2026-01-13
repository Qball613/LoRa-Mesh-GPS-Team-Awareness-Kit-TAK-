# LoRa Mesh Protocol Buffers v1

Protocol Buffer definitions for the LoRa Mesh GPS Team Awareness Kit (TAK).

**Registry:** [buf.build/qballq/tacprotobuf](https://buf.build/qballq/tacprotobuf)

## Proto Files

| File | Description |
|------|-------------|
| `common.proto` | Common data types: GPS coordinates, node info, link quality, Bluetooth messages |
| `messages.proto` | Application messages: LoRaMessage wrapper, GPS updates, text, hello, content sync |
| `routing.proto` | AODV routing: route request/reply/error, link state advertisements |
| `geographic.proto` | Geographic routing: position-based forwarding, beacons |
| `config.proto` | Network configuration and policy definitions |
| `metrics.proto` | Network monitoring and performance metrics |

## Publishing to Buf Registry

```bash
cd firmware
buf push
```

The `buf.yaml` in the `firmware/` directory configures the registry push. The `.pio` and other non-proto directories are excluded.

## Key Message Types

### LoRaMessage (messages.proto)
Main wrapper for all network communications:
- `message_id` - Unique identifier
- `source_id` / `destination_id` - Sender and recipient
- `source_port` / `destination_port` - Logical service ports
- `message_type` - GPS_UPDATE, TEXT_MESSAGE, ROUTE_REQUEST, etc.
- `payload` - Serialized content
- `signature` - Cryptographic authentication
- `priority` - Message priority level
- `routing_hint` - Suggested routing strategy

### Standard Ports

| Port | Service |
|------|---------|
| 1 | Routing Control (AODV, Link State) |
| 2 | Geographic Routing |
| 10 | Network Management |
| 20 | GPS Updates |
| 30 | Text Messaging |
| 40 | Emergency/Broadcast |
| 1000-1999 | User Applications |

### Routing Protocol (routing.proto)

**AODV Messages:**
- `RouteRequestPayload` - Route discovery with QoS support
- `RouteReplyPayload` - Route confirmation with alternate paths
- `RouteErrorPayload` - Link failure notifications

**Link State:**
- `LinkStateAdvertisement` - Topology announcements with neighbor info

### Geographic Routing (geographic.proto)
- `GeographicBeaconPayload` - Position announcements with velocity
- `PositionUpdatePayload` - GPS position sharing

### Security (common.proto & messages.proto)
- `CryptographicSignature` - AES-256-GCM, Ed25519, ECDSA support
- `KeyExchangePayload` - Secure key distribution
- `CertificatePayload` - PKI certificate management

**Primary encryption:** AES-256-GCM provides authenticated encryption with:
- 256-bit key (32 bytes)
- 96-bit nonce/IV (12 bytes)
- 128-bit authentication tag (16 bytes)

## Building for ESP32 with nanopb

Pre-generated C files are included (`*.pb.c`, `*.pb.h`). To regenerate:

```bash
# From firmware/v1 directory
python nanopb/generator/protoc --nanopb_out=. --proto_path=.. *.proto
```

### nanopb Options Files

Each proto has a corresponding `.options` file controlling field sizes:

```
# Example from common.options
lora_mesh.v1.NodeInfo.node_id max_size:17
lora_mesh.v1.GPSCoordinate.timestamp int_size:IS_64
```

## Message Size Estimates

| Message Type | Typical Size | Max Size |
|--------------|-------------|----------|
| GPS Update | 32 bytes | 64 bytes |
| Text Message | 64 bytes | 256 bytes |
| Route Request | 48 bytes | 128 bytes |
| Route Reply | 56 bytes | 144 bytes |
| Link State | 72 bytes | 256 bytes |
| Hello Beacon | 40 bytes | 96 bytes |

All sizes fit within LoRa packet limits (51-222 bytes depending on SF).

## Integration Example

```c
#include "v1/messages.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

void send_gps_update(float lat, float lon) {
    lora_mesh_v1_LoRaMessage message = lora_mesh_v1_LoRaMessage_init_zero;
    
    message.message_type = lora_mesh_v1_MessageType_MESSAGE_TYPE_GPS_UPDATE;
    message.has_gps_payload = true;
    message.gps_payload.position.latitude = lat;
    message.gps_payload.position.longitude = lon;
    
    uint8_t buffer[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    
    if (pb_encode(&stream, lora_mesh_v1_LoRaMessage_fields, &message)) {
        lora_send_packet(buffer, stream.bytes_written);
    }
}
```

## Changes from lora_mesh/v1

This directory (`firmware/v1`) is now the **canonical source** for protocol definitions:

1. **Buf registry sync** - Push directly from `firmware/` folder
2. **nanopb integration** - Options files and generated C code co-located
3. **Import paths** - Use `import "v1/common.proto"` format
4. **Pre-generated code** - `.pb.c` and `.pb.h` files included for ESP32 builds

The `lora_mesh/` folder is deprecated - use `firmware/v1/` for all proto work.
