/**
 * @file lora_radio.cpp
 * @brief LoRa radio implementation for LilyGO T3-S3 with LR1121
 *
 * Based on analysis of Meshtastic firmware which successfully runs on this hardware.
 * Key findings:
 * - SPI.begin() must be called with pins BEFORE creating ArduinoHal
 * - ArduinoHal is passed an already-initialized SPI
 * - No need to override init() or spiBegin() if SPI is pre-initialized
 * - LR1121 chip uses LR1120 driver (RadioLib doesn't have separate LR1121 class)
 * - LR1121 needs 3.0V TCXO voltage
 */

#include "lora_radio.h"
#include "security.h"
#include <SPI.h>

// Meshtastic-style locking HAL - simple wrapper that uses pre-initialized SPI
// The key is that SPI.begin() is called with pins BEFORE creating this HAL,
// so we override spiBegin() to prevent RadioLib from re-initializing with wrong pins
class MeshtasticStyleHal : public ArduinoHal {
public:
    MeshtasticStyleHal(SPIClass& spi, SPISettings settings)
        : ArduinoHal(spi, settings) {}

    // Override spiBegin to prevent RadioLib from calling SPI.begin() again
    void spiBegin() override {
        // SPI already initialized with correct pins - do nothing
    }
};

// SPI settings - Meshtastic uses 4MHz, but we start at 2MHz for safety
static SPISettings spiSettings(2000000, MSBFIRST, SPI_MODE0);
static MeshtasticStyleHal* radioHal = nullptr;

LoRaRadio* LoRaRadio::instance = nullptr;

// Interrupt flag - set when packet received
volatile bool LoRaRadio::receivedFlag = false;

LoRaRadio::LoRaRadio()
    : radio(nullptr)
    , radio_module(nullptr)
    , radio_initialized(false)
    , last_tx_time(0) {

    memset(&stats, 0, sizeof(stats));
    memset(last_tx_first_bytes, 0, sizeof(last_tx_first_bytes));
    instance = this;
}

/**
 * @brief Detect LoRa chip type by reading chip version
 * @return 1=SX1262/SX1268, 2=SX1276/SX1277/SX1278, 3=LR1121, 0=unknown
 */
static uint8_t detectChipType() {
    LOG_I("Detecting LoRa chip type...");

    // Configure pins for detection
    pinMode(LORA_RST, OUTPUT);
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    // Reset the chip
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

    // Try LR1121 detection (two-phase SPI protocol)
    #if defined(LORA_BUSY)
    pinMode(LORA_BUSY, INPUT);
    delay(200);  // LR1121 needs ~273ms after reset

    // LR1121 GetVersion command (0x0101)
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(LORA_CS, LOW);
    delayMicroseconds(1);
    SPI.transfer(0x01);
    SPI.transfer(0x01);
    digitalWrite(LORA_CS, HIGH);
    SPI.endTransaction();

    delay(5);
    uint32_t timeout = millis();
    while (digitalRead(LORA_BUSY) == HIGH && (millis() - timeout < 100)) {
        delayMicroseconds(100);
    }

    // Read response
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(LORA_CS, LOW);
    delayMicroseconds(1);
    uint8_t stat = SPI.transfer(0x00);
    uint8_t hw = SPI.transfer(0x00);
    uint8_t device = SPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    SPI.endTransaction();

    if (device == 0x03) {
        LOG_I("Detected: LR1121 (device=0x%02X, hw=0x%02X, stat=0x%02X)", device, hw, stat);
        return 3;
    }
    #endif

    // Try SX126x detection (read register 0x0740 for version)
    #if defined(LORA_BUSY)
    delay(10);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(LORA_CS, LOW);
    SPI.transfer(0x1D);  // Read register command
    SPI.transfer(0x07);  // Address MSB
    SPI.transfer(0x40);  // Address LSB (0x0740 = version)
    SPI.transfer(0x00);  // NOP
    uint8_t version = SPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    SPI.endTransaction();

    if (version == 0x12 || version == 0x10) {
        LOG_I("Detected: SX126x (version=0x%02X)", version);
        return 1;
    }
    #endif

    // Try SX127x detection (read register 0x42 for version)
    #if defined(LORA_DIO0) || !defined(LORA_BUSY)
    delay(10);
    digitalWrite(LORA_RST, LOW);
    delay(10);
    digitalWrite(LORA_RST, HIGH);
    delay(100);

    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    digitalWrite(LORA_CS, LOW);
    SPI.transfer(0x42 & 0x7F);  // Read register 0x42 (version)
    uint8_t sx127x_ver = SPI.transfer(0x00);
    digitalWrite(LORA_CS, HIGH);
    SPI.endTransaction();

    if (sx127x_ver == 0x12) {
        LOG_I("Detected: SX1276/SX1277/SX1278 (version=0x%02X)", sx127x_ver);
        return 2;
    }
    #endif

    LOG_W("Could not detect LoRa chip type!");
    return 0;
}

bool LoRaRadio::init() {
#if !HAS_LORA
    LOG_E("LoRa hardware not available");
    return false;
#endif

    LOG_I("=== LoRa Radio Initialization (Auto-detect) ===");
    #if defined(LORA_BUSY)
    LOG_I("Pins: CS=%d, RST=%d, BUSY=%d", LORA_CS, LORA_RST, LORA_BUSY);
    #if defined(LORA_DIO9)
    LOG_I("      IRQ/DIO9=%d, SCLK=%d, MISO=%d, MOSI=%d", LORA_DIO9, LORA_SCLK, LORA_MISO, LORA_MOSI);
    #elif defined(LORA_DIO1)
    LOG_I("      DIO1=%d, SCLK=%d, MISO=%d, MOSI=%d", LORA_DIO1, LORA_SCLK, LORA_MISO, LORA_MOSI);
    #endif
    #elif defined(LORA_DIO0)
    LOG_I("Pins: CS=%d, DIO0=%d, RST=%d", LORA_CS, LORA_DIO0, LORA_RST);
    LOG_I("      SCLK=%d, MISO=%d, MOSI=%d", LORA_SCLK, LORA_MISO, LORA_MOSI);
    #endif

    // Step 1: Release any GPIO holds from deep sleep (ESP32 specific)
    gpio_hold_dis((gpio_num_t)LORA_RST);

    // Step 2: Initialize all CS pins HIGH before SPI.begin() (Meshtastic T-Deck approach)
    // This prevents SPI bus contention with any other devices
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH);

    // Step 3: Initialize SPI with correct pins - CRITICAL!
    // Meshtastic does: SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    // This must happen BEFORE creating ArduinoHal
    LOG_I("Initializing SPI bus with custom pins...");
    SPI.begin(LORA_SCLK, LORA_MISO, LORA_MOSI, LORA_CS);
    SPI.setFrequency(2000000);  // 2MHz - conservative, Meshtastic uses 4MHz
    LOG_I("SPI initialized: SCK=%d, MISO=%d, MOSI=%d, CS=%d, Freq=2MHz",
          LORA_SCLK, LORA_MISO, LORA_MOSI, LORA_CS);

    // Step 4: Configure RST and BUSY pins
    pinMode(LORA_RST, OUTPUT);
    pinMode(LORA_BUSY, INPUT);

    // Step 5: Create HAL and Module
    LOG_I("Creating RadioLib HAL and Module...");
    radioHal = new MeshtasticStyleHal(SPI, spiSettings);

    int state = RADIOLIB_ERR_UNKNOWN;

    // Step 6: Initialize chip-specific radio (conditional compilation)
#if defined(USE_LR1121)
    LOG_I("Initializing LR1121...");
    radio_module = new Module(radioHal, LORA_CS, LORA_DIO9, LORA_RST, LORA_BUSY);
    radio = new LR1121(radio_module);

    state = radio->begin(
        LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
        LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM,
        LORA_PREAMBLE_LENGTH, 3.0  // 3.0V TCXO for LR1121
    );

    if (state == RADIOLIB_ERR_NONE) {
        radio->setCRC(2);
        radio->setRegulatorDCDC();

        // Configure RF switch (from Meshtastic tlora_t3s3_v1)
        static const uint32_t rfswitch_dio_pins[] = {
            RADIOLIB_LR11X0_DIO5, RADIOLIB_LR11X0_DIO6,
            RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC
        };
        static const Module::RfSwitchMode_t rfswitch_table[] = {
            {LR11x0::MODE_STBY, {LOW, LOW}}, {LR11x0::MODE_RX, {HIGH, LOW}},
            {LR11x0::MODE_TX, {LOW, HIGH}}, {LR11x0::MODE_TX_HP, {LOW, HIGH}},
            {LR11x0::MODE_TX_HF, {LOW, LOW}}, {LR11x0::MODE_GNSS, {LOW, LOW}},
            {LR11x0::MODE_WIFI, {LOW, LOW}}, END_OF_MODE_TABLE,
        };
        radio->setRfSwitchTable(rfswitch_dio_pins, rfswitch_table);

        // Print actual configuration
        LOG_I("LR1121 Config: Freq=%.2f MHz, BW=%.1f kHz, SF=%d, CR=4/%d, SW=0x%02X, Preamble=%d",
              LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
              LORA_CODING_RATE, LORA_SYNC_WORD, LORA_PREAMBLE_LENGTH);
    }

#elif defined(USE_SX1262)
    LOG_I("Initializing SX1262...");
    radio_module = new Module(radioHal, LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
    radio = new SX1262(radio_module);

    state = radio->begin(
        LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
        LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM,
        LORA_PREAMBLE_LENGTH
    );

    if (state == RADIOLIB_ERR_NONE) {
        radio->setCRC(2);
        radio->setDio2AsRfSwitch(true);
        radio->explicitHeader();  // Ensure explicit header mode

        // Print actual configuration
        LOG_I("SX1262 Config: Freq=%.2f MHz, BW=%.1f kHz, SF=%d, CR=4/%d, SW=0x%02X, Preamble=%d",
              LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
              LORA_CODING_RATE, LORA_SYNC_WORD, LORA_PREAMBLE_LENGTH);
    }

#elif defined(USE_SX1276)
    LOG_I("Initializing SX1276/1277/1278...");
    radio_module = new Module(radioHal, LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
    radio = new SX1276(radio_module);

    state = radio->begin(
        LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
        LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER_DBM,
        LORA_PREAMBLE_LENGTH
    );

    if (state == RADIOLIB_ERR_NONE) {
        radio->setCRC(true);

        // Print actual configuration
        LOG_I("SX1276 Config: Freq=%.2f MHz, BW=%.1f kHz, SF=%d, CR=4/%d, SW=0x%02X, Preamble=%d",
              LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ, LORA_SPREADING_FACTOR,
              LORA_CODING_RATE, LORA_SYNC_WORD, LORA_PREAMBLE_LENGTH);
    }
#endif

    if (state != RADIOLIB_ERR_NONE) {
        LOG_E("RadioLib initialization failed! Error code: %d", state);
        LOG_E("Error reference: -2=CHIP_NOT_FOUND, -3=SPI_CMD_TIMEOUT, -707=INVALID_BW");
        return false;
    }

    const char* chip_name =
#if defined(USE_LR1121)
        "LR1121";
#elif defined(USE_SX1262)
        "SX1262";
#elif defined(USE_SX1276)
        "SX1276/1277/1278";
#endif
    LOG_I("=== %s Initialization Complete ===", chip_name);
    radio_initialized = true;
    LOG_I("  Frequency: %.2f MHz", LORA_FREQUENCY_MHZ);
    LOG_I("  Bandwidth: %.1f kHz", LORA_BANDWIDTH_KHZ);
    LOG_I("  SF: %d, CR: 4/%d", LORA_SPREADING_FACTOR, LORA_CODING_RATE);
    LOG_I("  TX Power: %d dBm", LORA_TX_POWER_DBM);

    return true;
}

bool LoRaRadio::waitForClearChannel() {
    for (uint8_t attempt = 0; attempt < CAD_MAX_RETRIES; attempt++) {
        // Perform Channel Activity Detection
        int state = radio->scanChannel();

        if (state == RADIOLIB_LORA_DETECTED) {
            // Channel is busy - random backoff
            uint16_t backoff = random(CAD_BACKOFF_MIN_MS, CAD_BACKOFF_MAX_MS);
            LOG_D("CAD: channel busy, backoff %d ms (attempt %d/%d)",
                  backoff, attempt + 1, CAD_MAX_RETRIES);
            delay(backoff);
        } else if (state == RADIOLIB_CHANNEL_FREE) {
            // Channel is clear - OK to transmit
            if (attempt > 0) {
                LOG_D("CAD: channel clear after %d attempts", attempt + 1);
            }
            return true;
        } else {
            // CAD error - log and try anyway
            LOG_W("CAD scan error: %d", state);
            return true;  // Don't block TX on CAD errors
        }
    }

    // Max retries exhausted - channel still busy
    return false;
}

bool LoRaRadio::transmit(const LoRaMessage& msg) {
    if (!radio_initialized) {
        LOG_E("Radio not initialized");
        return false;
    }

    // Serialize the message first (no signature needed - GCM provides auth)
    uint8_t plaintext[MAX_LORA_PAYLOAD_BYTES];
    size_t plaintext_len = ProtobufHandler::encodeLoRaMessage(msg, plaintext, sizeof(plaintext));

    if (plaintext_len == 0) {
        LOG_E("Message encoding failed");
        return false;
    }

    // Encrypt the entire serialized message
    // Output: [12-byte nonce][ciphertext][16-byte GCM tag]
    uint8_t buffer[MAX_LORA_PAYLOAD_BYTES];
    size_t msg_len = 0;

    if (!Security::encryptMessage(plaintext, plaintext_len, buffer, &msg_len)) {
        LOG_E("Failed to encrypt message");
        return false;
    }

    if (msg_len > MAX_LORA_PAYLOAD_BYTES) {
        LOG_E("Encrypted message too large: %d bytes", msg_len);
        return false;
    }

    // Wait for clear channel before transmitting (CAD + random backoff)
    if (!waitForClearChannel()) {
        LOG_W("Channel busy after max retries, transmitting anyway");
    }

    // Transmit (blocking - waits for TX to complete)
    LOG_D("TX: %d bytes encrypted (plaintext was %d bytes)", msg_len, plaintext_len);

    // Save TX info for echo detection
    memcpy(last_tx_first_bytes, buffer, min((size_t)8, msg_len));

    int state = radio->transmit(buffer, msg_len);

    // Record TX time AFTER transmission completes
    last_tx_time = millis();

    if (state == RADIOLIB_ERR_NONE) {
        LOG_D("TX successful");
        stats.tx_count++;
        LOG_D("TX OK: type=%d, dst=%s, %d bytes",
              msg.message_type, msg.destination_id, msg_len);

        // Return to continuous receive mode after transmission
        radio->startReceive();
        return true;
    } else {
        stats.tx_errors++;
        LOG_E("TX failed, code: %d", state);

        // Try to return to receive mode even after error
        radio->startReceive();
        return false;
    }
}

bool LoRaRadio::receive(LoRaMessage& msg, int8_t& rssi) {
    if (!radio_initialized) {
        return false;
    }

    // Check if interrupt flag is set
    if (!receivedFlag) {
        return false;  // No packet received yet
    }

    // Reset flag
    receivedFlag = false;

    uint8_t buffer[MAX_LORA_PAYLOAD_BYTES];

    // Get packet length BEFORE calling readData()
    // This is required for LR11x0 chips - see RadioLib examples
    size_t msg_len = radio->getPacketLength();

    // Ignore spurious interrupts with 0-byte packets
    if (msg_len == 0) {
        return false;
    }

    // Read the received data
    // readData() returns error code (0 = success), NOT packet length
    int state = radio->readData(buffer, msg_len);

    if (state == RADIOLIB_ERR_NONE) {
        // Successfully received packet
        int8_t rssi_temp = radio->getRSSI();
        float snr = radio->getSNR();
        rssi = rssi_temp;

        LOG_I("RX: %d bytes, RSSI=%d dBm, SNR=%.1f dB", msg_len, rssi, snr);

        // Echo detection: Check if this is our own transmission bouncing back
        // This happens with high TX power or in enclosed spaces
        unsigned long now = millis();
        if (now - last_tx_time < TX_ECHO_WINDOW_MS) {
            // Within echo window - check if first bytes match our last TX
            if (msg_len >= 8 && memcmp(buffer, last_tx_first_bytes, 8) == 0) {
                LOG_D("Ignoring TX echo (received %lu ms after TX)", now - last_tx_time);
                return false;
            }
        }
    } else {
        // Error occurred during readData()
        if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            LOG_W("RX CRC mismatch (corrupted packet)");
        } else {
            LOG_E("readData() failed with code: %d", state);
        }
        stats.rx_errors++;
        return false;
    }

        // Log first bytes for debugging (will be nonce - random each time)
        if (msg_len >= 8) {
            LOG_D("RX first 8 bytes (nonce): %02X %02X %02X %02X %02X %02X %02X %02X",
                  buffer[0], buffer[1], buffer[2], buffer[3],
                  buffer[4], buffer[5], buffer[6], buffer[7]);
        }

        // Minimum encrypted packet size: nonce(12) + min_protobuf(~20) + tag(16) = ~48
        if (msg_len < 48) {
            LOG_W("Received packet too small (%d bytes), ignoring", msg_len);
            stats.rx_errors++;
            return false;
        }

        // Decrypt the entire message (GCM tag verifies authenticity)
        uint8_t plaintext[MAX_LORA_PAYLOAD_BYTES];
        size_t plaintext_len = 0;

        if (!Security::decryptMessage(buffer, msg_len, plaintext, &plaintext_len)) {
            unsigned long ms_since_tx = millis() - last_tx_time;
            LOG_W("Decrypt failed (%lu ms after our TX) - likely collision", ms_since_tx);
            stats.rx_errors++;
            return false;
        }

        LOG_D("RX decrypted: %d bytes -> %d bytes plaintext", msg_len, plaintext_len);

        // Decode the decrypted protobuf message
        if (!ProtobufHandler::decodeLoRaMessage(plaintext, plaintext_len, msg)) {
            LOG_W("Failed to decode decrypted message");
            stats.rx_errors++;
            return false;
        }

        // Check for replay using message_id (GCM already verified authenticity)
        if (!Security::checkReplayProtection(msg.message_id, msg.timestamp)) {
            LOG_W("Replay detected, dropping message: %s", msg.message_id);
            stats.rx_errors++;
            return false;
        }

        stats.rx_count++;
        LOG_I("RX: type=%d, src=%s, dst=%s, rssi=%d", 
              msg.message_type, msg.source_id, msg.destination_id, rssi);
        return true;
}

void LoRaRadio::setReceiveCallback(std::function<void(LoRaMessage&, int8_t)> callback) {
    rx_callback = callback;
}

void LoRaRadio::startListening() {
    if (!radio_initialized) {
        return;
    }

    // Set interrupt on DIO/IRQ based on radio type
#if defined(USE_LR1121)
    radio->setIrqAction(onReceiveISR);
#elif defined(USE_SX1262)
    radio->setDio1Action(onReceiveISR);
#elif defined(USE_SX1276)
    radio->setDio0Action(onReceiveISR, RISING);
#endif

    // Start listening
    int state = radio->startReceive();

    if (state != RADIOLIB_ERR_NONE) {
        LOG_E("Failed to start listening, code: %d", state);
    } else {
        LOG_I("LoRa radio listening...");
        // Clear any spurious interrupt flag from initialization
        receivedFlag = false;
    }
}

void LoRaRadio::onReceiveISR() {
    // Set flag that will be checked in receive()
    receivedFlag = true;
}
