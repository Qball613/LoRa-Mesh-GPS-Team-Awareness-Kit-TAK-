/**
 * @file display_manager.cpp
 * @brief OLED display implementation for SSD1306
 */

#include "display_manager.h"
#include "routing_engine.h"
#include "node_id.h"

// Include variant for pin definitions
#if defined(DEVICE_TYPE_T3S3)
    #include "variant.h"
#else
    #include "pin_config.h"
#endif

DisplayManager::DisplayManager()
    : display(nullptr)
    , routing_engine(nullptr)
    , initialized(false)
    , current_mode(DISPLAY_MODE_STATUS)
    , message_head(0)
    , message_count(0)
    , status_until(0) {
    
    memset(messages, 0, sizeof(messages));
    memset(status_text, 0, sizeof(status_text));
    memset(node_id, 0, sizeof(node_id));
}

bool DisplayManager::init() {
    // Initialize I2C
    Wire.begin(SCREEN_SDA, SCREEN_SCL);
    
    // Create display object
    display = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, SCREEN_RST);
    
    // Initialize display
    if (!display->begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        LOG_E("SSD1306 initialization failed");
        delete display;
        display = nullptr;
        return false;
    }
    
    // Store node ID
    String nid = NodeID::getNodeID();
    strncpy(node_id, nid.c_str(), sizeof(node_id) - 1);
    
    // Clear and show startup
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 0);
    display->println("LoRa Mesh TAK");
    display->println();
    display->print("Node: ");
    display->println(node_id);
    display->println();
    display->println("Initializing...");
    display->display();
    
    initialized = true;
    LOG_I("Display initialized (128x64 SSD1306)");
    return true;
}

void DisplayManager::update() {
    if (!initialized || !display) {
        return;
    }
    
    display->clearDisplay();
    
    // Check for temporary status message
    if (status_until > 0 && millis() < status_until) {
        display->setTextSize(1);
        display->setTextColor(SSD1306_WHITE);
        display->setCursor(0, 24);
        display->println(status_text);
        display->display();
        return;
    }
    
    // Draw based on current mode
    switch (current_mode) {
        case DISPLAY_MODE_STATUS:
            drawStatusScreen();
            break;
        case DISPLAY_MODE_MESSAGES:
            drawMessagesScreen();
            break;
        case DISPLAY_MODE_NEIGHBORS:
            drawNeighborsScreen();
            break;
        case DISPLAY_MODE_ROUTES:
            drawRoutesScreen();
            break;
    }
    
    display->display();
}

void DisplayManager::addMessage(const char* text, const char* sender, int8_t rssi) {
    // Add to circular buffer
    DisplayMessage& msg = messages[message_head];
    strncpy(msg.text, text, sizeof(msg.text) - 1);
    strncpy(msg.sender, sender, sizeof(msg.sender) - 1);
    msg.timestamp = millis();
    msg.rssi = rssi;
    
    message_head = (message_head + 1) % DISPLAY_MAX_MESSAGES;
    if (message_count < DISPLAY_MAX_MESSAGES) {
        message_count++;
    }
    
    LOG_D("Display: Added message from %s", sender);
}

void DisplayManager::showStatus(const char* status) {
    strncpy(status_text, status, sizeof(status_text) - 1);
    status_until = millis() + 3000;  // Show for 3 seconds
}

void DisplayManager::nextMode() {
    current_mode = static_cast<DisplayMode>((current_mode + 1) % 4);
    
    const char* mode_names[] = {"STATUS", "MESSAGES", "NEIGHBORS", "ROUTES"};
    showStatus(mode_names[current_mode]);
}

void DisplayManager::clear() {
    if (display) {
        display->clearDisplay();
        display->display();
    }
}

void DisplayManager::drawHeader() {
    display->setTextSize(1);
    display->setTextColor(SSD1306_WHITE);
    display->setCursor(0, 0);
    
    // Show abbreviated node ID and mode
    display->print(node_id + 5);  // Skip "NODE_" prefix
    display->print(" ");
    
    // Mode indicator
    switch (current_mode) {
        case DISPLAY_MODE_STATUS:    display->print("[STS]"); break;
        case DISPLAY_MODE_MESSAGES:  display->print("[MSG]"); break;
        case DISPLAY_MODE_NEIGHBORS: display->print("[NBR]"); break;
        case DISPLAY_MODE_ROUTES:    display->print("[RTE]"); break;
    }
    
    // Draw separator line
    display->drawLine(0, 9, SCREEN_WIDTH, 9, SSD1306_WHITE);
}

void DisplayManager::drawStatusScreen() {
    drawHeader();
    
    display->setCursor(0, 12);
    
    if (routing_engine) {
        auto& stats = routing_engine->stats;
        
        display->print("MeshV: ");
        display->println(routing_engine->getMeshVersion());
        
        display->print("TX:");
        display->print(stats.messages_sent);
        display->print(" RX:");
        display->println(stats.messages_received);
        
        display->print("FWD:");
        display->print(stats.messages_forwarded);
        display->print(" DRP:");
        display->println(stats.messages_dropped);
        
        // Neighbor count
        display->print("Neighbors: ");
        // Note: Would need getter for neighbor count
        display->println("--");
    } else {
        display->println("No routing engine");
    }
    
    // Uptime
    uint32_t uptime_sec = millis() / 1000;
    display->print("Up: ");
    display->print(uptime_sec / 60);
    display->print("m ");
    display->print(uptime_sec % 60);
    display->println("s");
}

void DisplayManager::drawMessagesScreen() {
    drawHeader();
    
    if (message_count == 0) {
        display->setCursor(0, 24);
        display->println("No messages yet");
        return;
    }
    
    // Show last few messages (newest at bottom)
    int y = 12;
    int max_lines = 6;  // Lines available after header
    
    // Calculate start index for showing newest messages
    int start = (message_head - min((int)message_count, max_lines) + DISPLAY_MAX_MESSAGES) % DISPLAY_MAX_MESSAGES;
    
    for (int i = 0; i < min((int)message_count, max_lines); i++) {
        int idx = (start + i) % DISPLAY_MAX_MESSAGES;
        DisplayMessage& msg = messages[idx];
        
        display->setCursor(0, y);
        
        // Truncate sender to 4 chars
        char short_sender[5];
        strncpy(short_sender, msg.sender + 5, 4);  // Skip "NODE_"
        short_sender[4] = '\0';
        
        display->print(short_sender);
        display->print(":");
        
        // Truncate message to fit
        char short_text[16];
        strncpy(short_text, msg.text, 15);
        short_text[15] = '\0';
        display->println(short_text);
        
        y += DISPLAY_LINE_HEIGHT;
    }
}

void DisplayManager::drawNeighborsScreen() {
    drawHeader();
    
    display->setCursor(0, 12);
    display->println("Neighbors:");
    
    // Would need access to neighbor table
    display->setCursor(0, 24);
    display->println("(Use show_neighbors)");
}

void DisplayManager::drawRoutesScreen() {
    drawHeader();
    
    display->setCursor(0, 12);
    display->println("Routes:");
    
    // Would need access to routing table
    display->setCursor(0, 24);
    display->println("(Use show_routes)");
}
