/**
 * @file node_id.h
 * @brief Node identification and addressing
 */

#ifndef NODE_ID_H
#define NODE_ID_H

#include <Arduino.h>
#include <string>

class NodeID {
public:
    /**
     * @brief Initialize node ID system
     * @return true if successful
     */
    static bool init();
    
    /**
     * @brief Get this node's unique ID
     * @return Node ID as string (e.g., "NODE_A1B2C3D4")
     */
    static String getNodeID();
    
    /**
     * @brief Set a custom node ID (stored in NVS)
     * @param id Custom node ID string
     * @return true if successfully saved
     */
    static bool setNodeID(const String& id);
    
    /**
     * @brief Generate node ID from ESP32 MAC address
     * @return Generated node ID
     */
    static String generateFromMAC();
    
    /**
     * @brief Check if a node ID is valid format
     * @param id Node ID to validate
     * @return true if valid
     */
    static bool isValid(const String& id);

private:
    static String nodeID;
    static bool initialized;
};

#endif // NODE_ID_H
