# Board Variant System

This project uses Meshtastic-style board variants for clean multi-chip support. No more manual editing of `pin_config.h` - just select your board variant when building!

## How It Works

Each board variant has its own folder in `variants/[platform]/[board_name]/` containing:

- `variant.h` - Pin definitions and feature flags for that specific board

The PlatformIO build system automatically includes the correct variant via the `-I` build flag.

## Supported Boards

### LilyGO T3-S3 V1.2 Variants

All T3-S3 variants share the same PCB and pinout, but support different LoRa chips:

#### t3s3-lr1121 (Default)

**LoRa Chip:** Semtech LR1121  
**Build Command:** `pio run -e t3s3-lr1121`  
**Variant Path:** `variants/esp32s3/t3s3_lr1121/`

Features:

- LR1121 with 3.0V TCXO
- BUSY pin on GPIO 34
- IRQ pin on GPIO 36
- Auto-detection with fallback

#### t3s3-sx1262

**LoRa Chip:** Semtech SX1262/SX1268  
**Build Command:** `pio run -e t3s3-sx1262`  
**Variant Path:** `variants/esp32s3/t3s3_sx1262/`

Features:

- SX1262 with 1.8V TCXO
- BUSY pin on GPIO 34
- DIO1 pin on GPIO 33
- DIO2_AS_RF_SWITCH enabled

#### t3s3-sx1276

**LoRa Chip:** Semtech SX1276/SX1277/SX1278  
**Build Command:** `pio run -e t3s3-sx1276`  
**Variant Path:** `variants/esp32s3/t3s3_sx1276/`

Features:

- SX1276 classic chip
- DIO0 on GPIO 9
- DIO1 on GPIO 33
- DIO2 on GPIO 34

## Common T3-S3 Hardware

All T3-S3 variants share these specifications:

**SPI Pins (LoRa):**

- SCK: GPIO 5
- MISO: GPIO 3
- MOSI: GPIO 6
- CS: GPIO 7
- RST: GPIO 8

**I2C Pins:**

- SDA: GPIO 18
- SCL: GPIO 17

**Other Features:**

- LED: GPIO 37
- Button: GPIO 0 (with pullup)
- Battery ADC: GPIO 1
- GPS RX/TX: GPIO 44/43
- SD Card: SPI1

## Building for Your Board

### Method 1: Select Default Environment

Edit `platformio.ini` and change the default:

```ini
[platformio]
default_envs = t3s3-lr1121  ; Change this to your chip
```

Then build:

```bash
pio run
pio run -t upload
```

### Method 2: Specify Environment

Build for a specific chip directly:

```bash
# For LR1121
pio run -e t3s3-lr1121 -t upload

# For SX1262
pio run -e t3s3-sx1262 -t upload

# For SX1276
pio run -e t3s3-sx1276 -t upload
```

## Adding a New Variant

1. **Create variant folder:**

   ```text
   variants/esp32s3/my_board_name/
   ```

2. **Create variant.h:**

   ```cpp
   #ifndef VARIANT_MY_BOARD_H
   #define VARIANT_MY_BOARD_H

   #define USE_SX1262  // or USE_LR1121, USE_SX1276

   // Define all pin numbers
   #define LORA_SCK 5
   #define LORA_MISO 3
   // ... etc

   #endif
   ```

3. **Add environment to platformio.ini:**
   ```ini
   [env:my-board]
   board = esp32-s3-devkitc-1
   build_flags =
       ${env.build_flags}
       -I variants/esp32s3/my_board_name
   ```

## Migration from pin_config.h

If you have old code using `pin_config.h`:

1. The variant system is backwards compatible
2. Auto-detection still works
3. Pin definitions come from `variant.h` instead

The old `pin_config.h` file is deprecated but can coexist during transition.

## How Variant Selection Works

The build process:

1. PlatformIO reads the `-I variants/esp32s3/[board]/` flag
2. Compiler searches that folder for `variant.h`
3. Code includes `<variant.h>` which resolves to the correct board
4. RadioLib initialization uses pins from variant.h
5. Feature flags (`USE_LR1121` etc.) enable chip-specific code paths

## Advantages

✅ **No manual editing** - Select board via environment  
✅ **No conditional compilation** - Each variant is self-contained  
✅ **Easy to add boards** - Just create a new variant folder  
✅ **Professional** - Same approach used by Meshtastic  
✅ **Multi-chip support** - Build for different chips effortlessly  
✅ **CI/CD friendly** - Each variant is a separate build target

## References

This system is inspired by the [Meshtastic firmware](https://github.com/meshtastic/firmware) variant architecture:

- Meshtastic: `variants/esp32s3/tlora_t3s3_v1/variant.h`
- This project: `variants/esp32s3/t3s3_lr1121/variant.h`
