# Serial Protocol Documentation

This document describes the SLIP-framed protobuf serial protocol for communicating with LoRa Mesh TAK firmware.

## Framing: SLIP (RFC 1055)

All packets are wrapped using Serial Line Internet Protocol (SLIP) framing:

| Byte | Name | Description |
|------|------|-------------|
| `0xC0` | END | Packet delimiter (start and end) |
| `0xDB` | ESC | Escape character |
| `0xDC` | ESC_END | Escaped END (when `0xC0` appears in data) |
| `0xDD` | ESC_ESC | Escaped ESC (when `0xDB` appears in data) |

### Encoding Rules
1. Start packet with `0xC0`
2. For each byte in payload:
   - If byte is `0xC0`, send `0xDB 0xDC`
   - If byte is `0xDB`, send `0xDB 0xDD`
   - Otherwise, send byte as-is
3. End packet with `0xC0`

### Decoding Rules
1. Wait for `0xC0` (packet start)
2. Read bytes until next `0xC0`:
   - If `0xDB` received, read next byte:
     - `0xDC` → append `0xC0` to buffer
     - `0xDD` → append `0xDB` to buffer
   - Otherwise, append byte to buffer
3. Decode buffer as protobuf `SerialPacket`

## Packet Structure

All messages are wrapped in a `SerialPacket`:

```protobuf
message SerialPacket {
    uint32 sequence = 1;      // Sequence number for request/response matching
    oneof payload {
        ToDevice to_device = 2;      // Commands from host to device
        FromDevice from_device = 3;  // Responses/events from device to host
    }
}
```

## Commands (Host → Device)

Send a `SerialPacket` with `to_device` field set:

```protobuf
message ToDevice {
    oneof command {
        Empty get_info = 1;           // Request node info
        Empty get_gps = 2;            // Request GPS position
        Empty get_neighbors = 3;      // Request neighbor list
        Empty get_routes = 4;         // Request routing table
        Empty get_roster = 5;         // Request team roster
        Empty get_stats = 6;          // Request statistics
        SendEmergency send_emergency = 10;  // Trigger emergency
    }
}

message Empty {}

message SendEmergency {
    EmergencyType type = 1;  // UNKNOWN, SOS, MEDICAL, FIRE, POLICE, CUSTOM
    string message = 2;      // Optional message (max 64 chars)
}
```

## Responses (Device → Host)

Device responds with `SerialPacket` containing `from_device`:

```protobuf
message FromDevice {
    oneof payload {
        InfoResponse info_response = 1;
        GPSResponse gps_response = 2;
        NeighborsResponse neighbors_response = 3;
        RoutesResponse routes_response = 4;
        RosterResponse roster_response = 5;
        StatsResponse stats_response = 6;
        ErrorResponse error = 15;
        
        // Async events (unsolicited)
        NodeInfo node_update = 20;
        GPSCoordinate position_update = 21;
    }
}
```

### InfoResponse
```protobuf
message InfoResponse {
    NodeInfo node = 1;  // This node's info
}

// From common.proto
message NodeInfo {
    bytes node_id = 1;        // 4-byte node ID
    string callsign = 2;      // Callsign (max 16 chars)
    uint32 team = 3;          // Team number
    Role role = 4;            // STANDARD, TRACKER, ROUTER, GATEWAY
    bool gps_capable = 5;
    uint32 last_seen = 6;     // Unix timestamp
}
```

### GPSResponse
```protobuf
message GPSResponse {
    GPSCoordinate position = 1;
    uint32 satellites = 2;
    float hdop = 3;
}

// From common.proto
message GPSCoordinate {
    sint32 latitude_i = 1;   // lat * 1e7
    sint32 longitude_i = 2;  // lon * 1e7
    sint32 altitude = 3;     // meters
    uint32 timestamp = 4;    // Unix timestamp
    float accuracy = 5;      // meters
    float speed = 6;         // m/s
    float heading = 7;       // degrees
}
```

### NeighborsResponse
```protobuf
message NeighborsResponse {
    repeated NeighborInfo neighbors = 1;  // Max 16 entries
}

message NeighborInfo {
    bytes node_id = 1;      // 4-byte neighbor ID
    int32 rssi = 2;         // dBm
    float snr = 3;          // dB
    uint32 last_seen = 4;   // Unix timestamp
    uint32 link_quality = 5; // 0-100
}
```

### RoutesResponse
```protobuf
message RoutesResponse {
    repeated RouteInfo routes = 1;  // Max 32 entries
}

message RouteInfo {
    bytes destination = 1;   // 4-byte destination ID
    bytes next_hop = 2;      // 4-byte next hop ID
    uint32 hop_count = 3;
    uint32 metric = 4;
    uint32 last_used = 5;    // Unix timestamp
}
```

### RosterResponse
```protobuf
message RosterResponse {
    repeated RosterEntry roster = 1;  // Max 32 entries
}

message RosterEntry {
    NodeInfo node = 1;
    GPSCoordinate last_position = 2;
    bool has_position = 3;
}
```

### StatsResponse
```protobuf
message StatsResponse {
    uint32 tx_packets = 1;
    uint32 rx_packets = 2;
    uint32 tx_bytes = 3;
    uint32 rx_bytes = 4;
    uint32 uptime_seconds = 5;
}
```

### ErrorResponse
```protobuf
message ErrorResponse {
    uint32 code = 1;
    string message = 2;  // Max 64 chars
}
```

## Example: Python Client Pseudocode

```python
import serial
from v1 import serial_pb2  # Generated from serial.proto

SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

def slip_encode(data: bytes) -> bytes:
    """Encode data with SLIP framing."""
    result = bytearray([SLIP_END])
    for b in data:
        if b == SLIP_END:
            result.extend([SLIP_ESC, SLIP_ESC_END])
        elif b == SLIP_ESC:
            result.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            result.append(b)
    result.append(SLIP_END)
    return bytes(result)

def slip_decode(data: bytes) -> bytes:
    """Decode SLIP-framed data."""
    result = bytearray()
    i = 0
    while i < len(data):
        if data[i] == SLIP_ESC:
            i += 1
            if data[i] == SLIP_ESC_END:
                result.append(SLIP_END)
            elif data[i] == SLIP_ESC_ESC:
                result.append(SLIP_ESC)
        elif data[i] != SLIP_END:
            result.append(data[i])
        i += 1
    return bytes(result)

def send_command(ser: serial.Serial, command: str, seq: int = 0):
    """Send a command to the device."""
    packet = serial_pb2.SerialPacket()
    packet.sequence = seq
    
    if command == "get_info":
        packet.to_device.get_info.CopyFrom(serial_pb2.Empty())
    elif command == "get_gps":
        packet.to_device.get_gps.CopyFrom(serial_pb2.Empty())
    # ... etc
    
    encoded = slip_encode(packet.SerializeToString())
    ser.write(encoded)

def read_response(ser: serial.Serial) -> serial_pb2.SerialPacket:
    """Read and decode a response packet."""
    buffer = bytearray()
    in_packet = False
    
    while True:
        b = ser.read(1)
        if not b:
            continue
        b = b[0]
        
        if b == SLIP_END:
            if in_packet and buffer:
                # End of packet
                decoded = slip_decode(bytes(buffer))
                packet = serial_pb2.SerialPacket()
                packet.ParseFromString(decoded)
                return packet
            else:
                # Start of packet
                in_packet = True
                buffer.clear()
        elif in_packet:
            buffer.append(b)
```

## Generating Python Protobuf Files

From the `firmware/` directory:

```bash
# Install protobuf compiler
pip install grpcio-tools

# Generate Python files
python -m grpc_tools.protoc -I. --python_out=../python_client v1/common.proto
python -m grpc_tools.protoc -I. --python_out=../python_client v1/messages.proto  
python -m grpc_tools.protoc -I. --python_out=../python_client v1/serial.proto
```

## Wire Format Example

Request `get_info`:
```
Sequence: 1
Command: get_info (Empty)

Protobuf hex: 08 01 12 02 0A 00
  08 01       = field 1 (sequence) varint = 1
  12 02       = field 2 (to_device) length = 2
    0A 00     = field 1 (get_info) length = 0

SLIP-framed: C0 08 01 12 02 0A 00 C0
```

## Baud Rate

Default: **115200 baud**, 8N1

## Sequence Numbers

- Host should increment sequence number for each request
- Device echoes sequence number in response
- Useful for matching responses to requests in async scenarios
- Unsolicited events (node_update, position_update) use sequence = 0
