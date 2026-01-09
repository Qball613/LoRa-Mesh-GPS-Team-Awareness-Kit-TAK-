/**
 * @file display_manager.h
 * @brief OLED display manager for message and status display
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"

// Forward declarations
class RoutingEngine;

// Maximum messages to keep in scroll buffer
#define DISPLAY_MAX_MESSAGES 10
#define DISPLAY_LINE_HEIGHT 8
#define DISPLAY_LINES 8  // 64 pixels / 8 = 8 lines

// Display mode
enum DisplayMode {
    DISPLAY_MODE_STATUS,    // Show node status
    DISPLAY_MODE_MESSAGES,  // Show message log
    DISPLAY_MODE_NEIGHBORS, // Show neighbor list
    DISPLAY_MODE_ROUTES     // Show routing table
};

struct DisplayMessage {
    char text[64];
    char sender[16];
    uint64_t timestamp;
    int8_t rssi;
};

class DisplayManager {
public:
    DisplayManager();
    
    /**
     * @brief Initialize the OLED display
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Update the display (call periodically)
     */
    void update();
    
    /**
     * @brief Set the routing engine reference for status info
     */
    void setRoutingEngine(RoutingEngine* engine) { routing_engine = engine; }
    
    /**
     * @brief Add a received message to display buffer
     * @param text Message text
     * @param sender Sender node ID
     * @param rssi Signal strength
     */
    void addMessage(const char* text, const char* sender, int8_t rssi);
    
    /**
     * @brief Show a temporary status message
     * @param status Status text (shown for a few seconds)
     */
    void showStatus(const char* status);
    
    /**
     * @brief Cycle to next display mode
     */
    void nextMode();
    
    /**
     * @brief Set display mode
     */
    void setMode(DisplayMode mode) { current_mode = mode; }
    
    /**
     * @brief Get current display mode
     */
    DisplayMode getMode() const { return current_mode; }
    
    /**
     * @brief Clear the display
     */
    void clear();
    
    /**
     * @brief Check if display is initialized
     */
    bool isInitialized() const { return initialized; }

private:
    Adafruit_SSD1306* display;
    RoutingEngine* routing_engine;
    
    bool initialized;
    DisplayMode current_mode;
    
    // Message buffer (circular)
    DisplayMessage messages[DISPLAY_MAX_MESSAGES];
    uint8_t message_head;
    uint8_t message_count;
    
    // Temporary status
    char status_text[32];
    uint64_t status_until;
    
    // Node info
    char node_id[16];
    
    // Draw functions
    void drawStatusScreen();
    void drawMessagesScreen();
    void drawNeighborsScreen();
    void drawRoutesScreen();
    void drawHeader();
};

#endif // DISPLAY_MANAGER_H
