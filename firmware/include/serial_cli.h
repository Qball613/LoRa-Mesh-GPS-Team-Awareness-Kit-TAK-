/**
 * @file serial_cli.h
 * @brief Serial command-line interface for testing and control
 */

#ifndef SERIAL_CLI_H
#define SERIAL_CLI_H

#include <Arduino.h>
#include <functional>
#include "routing_engine.h"
#include "gps_manager.h"

class SerialCLI {
public:
    SerialCLI();
    
    /**
     * @brief Initialize serial CLI
     */
    bool init();
    
    /**
     * @brief Process incoming serial commands (call from main loop)
     */
    void process();
    
    /**
     * @brief Set routing engine reference
     */
    void setRoutingEngine(RoutingEngine* engine);
    
    /**
     * @brief Set GPS manager reference
     */
    void setGPSManager(GPSManager* gps);
    
    /**
     * @brief Print welcome message
     */
    void printWelcome();
    
    /**
     * @brief Print help message
     */
    void printHelp();

private:
    RoutingEngine* routing_engine;
    GPSManager* gps_manager;
    
    String command_buffer;
    
    void processCommand(const String& cmd);
    void handleSendGPS(const String& args);
    void handleSendMsg(const String& args);
    void handleSetStaticGPS(const String& args);
    void handleSetManualGPS(const String& args);
    void handleSetGPSMode(const String& args);
    void handleShowRoutes();
    void handleShowNeighbors();
    void handleShowStats();
    void handleShowGPS();
    void handleSetNodeID(const String& args);
    void handleGenerateKey();
    
    // New protobuf-based commands
    void handlePing(const String& args);
    void handleEmergency(const String& args);
    void handleControl(const String& args);
    void handleDiscover();
    void handleJoin();
    void handleShowNode();
    void handleFlushRoutes();
};

#endif // SERIAL_CLI_H
