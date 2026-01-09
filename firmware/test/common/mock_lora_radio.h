#ifndef MOCK_LORA_RADIO_H
#define MOCK_LORA_RADIO_H

#include <vector>
#include <string>
#include "../../include/protobuf_handler.h"

/**
 * Mock LoRa Radio for Unit Testing
 * Simulates LoRa radio behavior without hardware
 */
class MockLoRaRadio {
public:
    MockLoRaRadio() : 
        transmit_should_succeed(true),
        last_rssi(-80),
        last_snr(8.0),
        transmit_count(0),
        receive_count(0) {}
    
    // Simulated transmission
    bool transmit(const LoRaMessage& msg) {
        if (transmit_should_succeed) {
            transmitted_messages.push_back(msg);
            transmit_count++;
            return true;
        }
        return false;
    }
    
    // Inject message for testing receive path
    void injectMessage(const LoRaMessage& msg) {
        received_messages.push_back(msg);
        receive_count++;
    }
    
    // Get next received message (for testing)
    bool receive(LoRaMessage& msg) {
        if (!received_messages.empty()) {
            msg = received_messages.front();
            received_messages.erase(received_messages.begin());
            return true;
        }
        return false;
    }
    
    // Simulate RSSI
    int getRSSI() const { return last_rssi; }
    void setRSSI(int rssi) { last_rssi = rssi; }
    
    // Simulate SNR
    float getSNR() const { return last_snr; }
    void setSNR(float snr) { last_snr = snr; }
    
    // Test helpers
    void clear() {
        transmitted_messages.clear();
        received_messages.clear();
        transmit_count = 0;
        receive_count = 0;
    }
    
    size_t getTransmitCount() const { return transmit_count; }
    size_t getReceiveCount() const { return receive_count; }
    
    const std::vector<LoRaMessage>& getTransmittedMessages() const {
        return transmitted_messages;
    }
    
    // Control transmission success
    void setTransmitSuccess(bool should_succeed) {
        transmit_should_succeed = should_succeed;
    }

public:
    std::vector<LoRaMessage> transmitted_messages;
    std::vector<LoRaMessage> received_messages;
    
private:
    bool transmit_should_succeed;
    int last_rssi;
    float last_snr;
    size_t transmit_count;
    size_t receive_count;
};

#endif // MOCK_LORA_RADIO_H
