/**
 * @file lora_radio.h
 * @brief LoRa radio interface using RadioLib for SX1262
 */

#ifndef LORA_RADIO_H
#define LORA_RADIO_H

#include <Arduino.h>
#include <RadioLib.h>
#include "config.h"
#include "protobuf_handler.h"

// Include board variant configuration
#include "variant.h"

class LoRaRadio {
public:
    LoRaRadio();

    /**
     * @brief Initialize LoRa radio
     * @return true if successful
     */
    bool init();

    /**
     * @brief Transmit a LoRaMessage
     * @param msg Message to transmit
     * @return true if successful
     */
    bool transmit(const LoRaMessage& msg);

    /**
     * @brief Check for received messages (non-blocking)
     * @param msg Output message
     * @param rssi Output RSSI
     * @return true if message received
     */
    bool receive(LoRaMessage& msg, int8_t& rssi);

    /**
     * @brief Set receive callback (for interrupt-driven reception)
     */
    void setReceiveCallback(std::function<void(LoRaMessage&, int8_t)> callback);

    /**
     * @brief Start listening (interrupt mode)
     */
    void startListening();

    /**
     * @brief Get statistics
     */
    struct {
        uint32_t tx_count;
        uint32_t rx_count;
        uint32_t tx_errors;
        uint32_t rx_errors;
    } stats;

private:
    // Only compile the radio type we're using
#if defined(USE_SX1262)
    SX1262* radio;
    static constexpr uint8_t radio_type = 1;
#elif defined(USE_SX1276)
    SX1276* radio;
    static constexpr uint8_t radio_type = 2;
#elif defined(USE_LR1121)
    LR1121* radio;
    static constexpr uint8_t radio_type = 3;
#else
    #error "No radio type defined! USE_SX1262, USE_SX1276, or USE_LR1121 must be set"
#endif

    Module* radio_module;

    std::function<void(LoRaMessage&, int8_t)> rx_callback;

    bool radio_initialized;

    // Echo suppression - ignore packets received right after TX
    unsigned long last_tx_time;
    uint8_t last_tx_first_bytes[8];  // First 8 bytes of last TX for echo detection
    static constexpr unsigned long TX_ECHO_WINDOW_MS = 100;  // Ignore RX within 100ms of TX

    // CAD (Channel Activity Detection) settings
    static constexpr uint8_t CAD_MAX_RETRIES = 5;       // Max CAD attempts before forcing TX
    static constexpr uint16_t CAD_BACKOFF_MIN_MS = 50;  // Min random backoff when channel busy
    static constexpr uint16_t CAD_BACKOFF_MAX_MS = 200; // Max random backoff when channel busy

    bool waitForClearChannel();  // Returns true if channel is clear

    // Interrupt flag - set by ISR when packet received
    static volatile bool receivedFlag;

    static void onReceiveISR();
    static LoRaRadio* instance;
};

#endif // LORA_RADIO_H
