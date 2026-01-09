/*
 * @Description: Pin Configuration
 * @version: 2.0
 * @Author: None
 * @Date: 2023-06-05 13:01:59
 * @LastEditors: Migrated to variant system
 * @LastEditTime: 2026-01-07
 * 
 * All pin definitions now come from variant.h files.
 * Select your board variant in platformio.ini:
 *   - t3s3-lr1121  (LR1121 chip)
 *   - t3s3-sx1262  (SX1262 chip)
 *   - t3s3-sx1276  (SX1276/1277/1278 chip)
 * 
 * Each variant defines all pins - no need to edit this file.
 */
#pragma once

// Include board variant (selected by platformio.ini build flag -I variants/...)
#include "variant.h"

// IIC
#define IIC_SDA 42
#define IIC_SCL 45

// SD
#define SD_SCLK 14
#define SD_MISO 2
#define SD_MOSI 11
#define SD_CS 13

// BOOT
#define BOOT_KEY 0

// LED
#define LED_1 37

// Screen SSD1306
#define SCREEN_RST -1
#define SCREEN_SDA 18
#define SCREEN_SCL 17
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

// Vibratino Motor
#define VIBRATINO_MOTOR_PWM 46

// PCF85063
#define PCF85063_IIC_SDA 42
#define PCF85063_IIC_SCL 45
#define PCF85063_INT 16

#if defined T3_S3_MVSRBoard_V1_0
// MSM261S4030H0R
#define MSM261_EN 35
#define MSM261_BCLK 47
#define MSM261_WS 15
#define MSM261_DATA 48
#elif defined T3_S3_MVSRBoard_V1_1
// MP34DT05TR
#define MP34DT05TR_LRCLK 15
#define MP34DT05TR_DATA 48
#define MP34DT05TR_EN 35
#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

// MAX98357AETE+T
#define MAX98357A_BCLK 40
#define MAX98357A_LRCLK 41
#define MAX98357A_DATA 39
#define MAX98357A_SD_MODE 38

/////////////////////////////////////////////////////////////////////////