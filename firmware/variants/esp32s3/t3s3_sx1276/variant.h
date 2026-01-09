#ifndef VARIANT_T3S3_SX1276_H
#define VARIANT_T3S3_SX1276_H

// =============================================================================
// LilyGO T3-S3 V1.2 Board with SX1276/SX1277/SX1278 LoRa Chip
// All pin definitions for this board variant
// =============================================================================

// Radio Chip Type
#define USE_SX1276

// LoRa SPI Pins
#define LORA_SCK 5
#define LORA_SCLK 5      // Alias for old code
#define LORA_MISO 3
#define LORA_MOSI 6
#define LORA_CS 7
#define LORA_RESET 8
#define LORA_RST 8       // Alias for old code

// SX1276 Specific Pins
#define LORA_DIO0 9      // RxDone/TxDone
#define LORA_DIO1 33     // RxTimeout/FhssChangeChannel
#define LORA_DIO2 34     // FhssChangeChannel (LoRa) / Data (FSK)
#define LORA_DIO3 21
#define LORA_DIO4 10
#define LORA_DIO5 36
#define LORA_BUSY 34     // Map to DIO2 (SX1276 doesn't have BUSY pin)

// I2C Pins
#define I2C_SDA 18
#define I2C_SCL 17
#define IIC_SDA 18       // Alias
#define IIC_SCL 17       // Alias

// Screen (SSD1306)
#define SCREEN_SDA 18
#define SCREEN_SCL 17
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define SCREEN_RST -1

// LED and Button
#define LED_PIN 37
#define LED_1 37         // Alias
#define BUTTON_PIN 0
#define BOOT_KEY 0       // Alias
#define BUTTON_NEED_PULLUP

// Battery Monitoring
#define BATTERY_PIN 1
#define ADC_CHANNEL ADC1_GPIO1_CHANNEL
#define ADC_MULTIPLIER 2.11

// GPS
#define GPS_RX_PIN 44
#define GPS_TX_PIN 43

// SD Card
#define HAS_SDCARD
#define SDCARD_USE_SPI1
#define SD_CS 13
#define SD_MOSI 11
#define SD_MISO 2
#define SD_SCLK 14

// Board Version
#define T3_S3_MVSRBoard_V1_1

#endif // VARIANT_T3S3_SX1276_H
