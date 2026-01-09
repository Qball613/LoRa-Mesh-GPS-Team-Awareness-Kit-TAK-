/**
 * @file node_id.cpp
 * @brief Node identification implementation
 */

#include "node_id.h"
#include "config.h"
#include <Preferences.h>
#include <esp_system.h>

String NodeID::nodeID = "";
bool NodeID::initialized = false;

bool NodeID::init() {
    if (initialized) {
        return true;
    }
    
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        LOG_E("Failed to open NVS namespace for node ID");
        return false;
    }
    
    // Try to load existing node ID from NVS
    String stored = prefs.getString(NVS_KEY_NODE_ID, "");
    
    if (stored.length() > 0 && isValid(stored)) {
        nodeID = stored;
        LOG_I("Loaded node ID from NVS: %s", nodeID.c_str());
    } else {
        // Generate new node ID from MAC address
        nodeID = generateFromMAC();
        prefs.putString(NVS_KEY_NODE_ID, nodeID);
        LOG_I("Generated new node ID: %s", nodeID.c_str());
    }
    
    prefs.end();
    initialized = true;
    return true;
}

String NodeID::getNodeID() {
    if (!initialized) {
        init();
    }
    return nodeID;
}

bool NodeID::setNodeID(const String& id) {
    if (!isValid(id)) {
        LOG_E("Invalid node ID format: %s", id.c_str());
        return false;
    }
    
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        LOG_E("Failed to open NVS namespace");
        return false;
    }
    
    prefs.putString(NVS_KEY_NODE_ID, id);
    prefs.end();
    
    nodeID = id;
    LOG_I("Node ID updated: %s", nodeID.c_str());
    return true;
}

String NodeID::generateFromMAC() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    char id[32];
    snprintf(id, sizeof(id), "NODE_%02X%02X%02X%02X",
             mac[2], mac[3], mac[4], mac[5]);
    
    return String(id);
}

bool NodeID::isValid(const String& id) {
    // Node ID must be 4-16 characters, alphanumeric + underscore
    if (id.length() < 4 || id.length() > 16) {
        return false;
    }
    
    for (size_t i = 0; i < id.length(); i++) {
        char c = id.charAt(i);
        if (!isalnum(c) && c != '_') {
            return false;
        }
    }
    
    return true;
}
