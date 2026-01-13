# Quick Build Guide

## Prerequisites

1. **Install PlatformIO**:

   ```bash
   pip install platformio
   ```

   Or install the [PlatformIO IDE extension for VS Code](https://platformio.org/install/ide?install=vscode)

2. **Install Git** (if not already installed)

## Build Steps

### Option 1: Command Line

```bash
# Navigate to firmware directory
cd firmware/

# Build for LilyGO T3S3 gateway
pio run -e t3s3-gateway

# Or build for T3S3 standalone
pio run -e t3s3-standalone

# Or build for T-Deck with LoRa
pio run -e tdeck-lora
```

### Option 2: VS Code with PlatformIO

1. Open the `firmware/` folder in VS Code
2. PlatformIO will automatically detect the project
3. Click the environment selector at the bottom (default: `t3s3-gateway`)
4. Click the "Build" button (checkmark icon) or press `Ctrl+Alt+B`

## Upload to Device

### Command Line

```bash
# Connect T3S3 via USB
pio run -e t3s3-gateway -t upload

# Monitor serial output
pio device monitor -b 115200
```

### VS Code

1. Connect device via USB
2. Click the "Upload" button (arrow icon) or press `Ctrl+Alt+U`
3. Click the "Serial Monitor" button (plug icon) to view output

## First Boot

### 1. Initial Setup

On first boot, the device will generate a unique node ID and HMAC key:

```text
Node ID: NODE_A1B2C3D4
HMAC Key (HEX): 1A2B3C4D5E6F7A8B9C0D1E2F3A4B5C6D

Type 'help' for available commands
```

**IMPORTANT**: Copy the HMAC key and flash it to all nodes in your mesh!

### 2. Configure GPS for Testing

```text
> set_manual_gps 37.7749 -122.4194
OK: Manual GPS set to 37.774900, -122.419400

> set_gps_mode 2
OK: GPS mode set to 2

> show_gps
GPS Mode: 2 (Manual)
Has Fix: Yes
```

### 3. Test Communication

```text
# On Node A
> set_node_id NODE_A
> send_gps

# On Node B (separate computer)
> set_node_id NODE_B
> send_msg NODE_A Hello!
```

## Common Issues

### Build Errors

**Missing RadioLib**:

```text
fatal error: RadioLib.h: No such file or directory
```

Solution: PlatformIO will auto-install dependencies. If not, run:

```bash
pio lib install "jgromes/RadioLib@^6.6.0"
```

**Missing TinyGPSPlus**:

```bash
pio lib install "mikalhart/TinyGPSPlus@^1.0.3"
```

### Upload Errors

**Port Not Found**:

```text
Error: Could not open COM3, the port doesn't exist
```

Solution:

- Check USB cable connection
- Install CH340/CP2102 drivers if needed
- Check port in Device Manager (Windows) or `ls /dev/tty*` (Linux/Mac)

**Permission Denied (Linux)**:

```text
Error: Permission denied: '/dev/ttyUSB0'
```

Solution:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

## Multi-Radio Testing

### 6-Radio Setup

1. **Build and flash all 6 radios**:

   ```bash
   pio run -e t3s3-gateway -t upload
   ```

2. **Copy HMAC key from first radio to all others**:
   - Note the key from first radio's boot
   - On each subsequent radio, use that same key (store in code or flash via NVS)

3. **Configure each radio with unique node ID**:

   ```text
   # Radio 1
   > set_node_id NODE_A
   > set_manual_gps 37.7749 -122.4194

   # Radio 2
   > set_node_id NODE_B
   > set_manual_gps 37.7750 -122.4195

   # etc...
   ```

4. **Test mesh connectivity**:

   ```text
   # From any radio
   > show_neighbors
   > show_routes
   > send_msg BROADCAST Testing 1-2-3
   ```

## Next Steps

- See [firmware/README.md](README.md) for detailed documentation
- Review [../routing_simulation/](../routing_simulation/) for algorithm reference
- Check main [../README.md](../README.md) for project overview
