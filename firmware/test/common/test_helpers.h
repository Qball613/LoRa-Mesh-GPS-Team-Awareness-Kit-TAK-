#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <cstdint>
#include <cmath>

/**
 * Test Helper Functions
 * Common utilities for unit tests
 */

// Simulate millis() for native testing
uint64_t mock_millis_value = 0;

uint64_t millis() {
    return mock_millis_value;
}

void set_mock_millis(uint64_t value) {
    mock_millis_value = value;
}

void advance_mock_millis(uint64_t delta_ms) {
    mock_millis_value += delta_ms;
}

// Helper for floating point comparisons
bool floatsEqual(float a, float b, float epsilon = 0.001f) {
    return std::fabs(a - b) < epsilon;
}

bool doublesEqual(double a, double b, double epsilon = 0.0001) {
    return std::fabs(a - b) < epsilon;
}

// Generate test node IDs
const char* generateNodeId(int index) {
    static char buffer[16];
    snprintf(buffer, sizeof(buffer), "NODE_%02d", index);
    return buffer;
}

// Generate test message IDs
const char* generateMessageId(int index) {
    static char buffer[32];
    snprintf(buffer, sizeof(buffer), "MSG_%08d", index);
    return buffer;
}

#endif // TEST_HELPERS_H
