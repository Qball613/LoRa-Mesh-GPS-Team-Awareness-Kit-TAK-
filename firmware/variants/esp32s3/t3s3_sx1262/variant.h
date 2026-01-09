#ifndef VARIANT_T3S3_SX1262_H
#define VARIANT_T3S3_SX1262_H

// =============================================================================
// LilyGO T3-S3 V1.2 Board with SX1262 LoRa Chip
// All pin definitions for this board variant
// =============================================================================

// Radio Chip Type
#define USE_SX1262

// LoRa SPI Pins
#define LORA_SCK 5
#define LORA_SCLK 5      // Alias for old code
#define LORA_MISO 3
#define LORA_MOSI 6
#define LORA_CS 7
#define LORA_RESET 8
#define LORA_RST 8       // Alias for old code

// SX1262 Specific Pins
#define LORA_DIO1 33
#define LORA_BUSY 34
#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY LORA_BUSY
#define SX126X_RESET LORA_RESET
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

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

#endif // VARIANT_T3S3_SX1262_H
