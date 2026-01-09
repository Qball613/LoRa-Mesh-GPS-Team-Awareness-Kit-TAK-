/**
 * @file gps_manager.h
 * @brief GPS position management with hardware/static/manual modes
 */

#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include "config.h"
#include "protobuf_handler.h"
#include "variant.h"  // For GPS_RX_PIN, GPS_TX_PIN

class GPSManager {
public:
    GPSManager();
    
    /**
     * @brief Initialize GPS system
     * @return true if successful
     */
    bool init();
    
    /**
     * @brief Update GPS (call regularly from main loop)
     */
    void update();
    
    /**
     * @brief Get current GPS position
     * @param coord Output coordinate
     * @return true if position is valid
     */
    bool getPosition(GPSCoordinate& coord);
    
    /**
     * @brief Set GPS source mode
     */
    void setSourceMode(GPSSourceMode mode);
    
    /**
     * @brief Get current GPS source mode
     */
    GPSSourceMode getSourceMode() const { return source_mode; }
    
    /**
     * @brief Set static GPS coordinates (for fixed nodes)
     * @param lat Latitude
     * @param lon Longitude
     * @param alt Altitude (optional)
     * @return true if successful
     */
    bool setStaticPosition(double lat, double lon, double alt = 0.0);
    
    /**
     * @brief Set manual GPS coordinates (for testing)
     */
    bool setManualPosition(double lat, double lon, double alt = 0.0);
    
    /**
     * @brief Load static position from NVS
     */
    bool loadStaticPositionFromNVS();
    
    /**
     * @brief Save static position to NVS
     */
    bool saveStaticPositionToNVS();
    
    /**
     * @brief Check if GPS has valid fix
     */
    bool hasFix() const;
    
    /**
     * @brief Get number of satellites
     */
    uint8_t getSatelliteCount() const;
    
    /**
     * @brief Calculate distance between two coordinates (Haversine formula)
     * @return Distance in meters
     */
    static double calculateDistance(const GPSCoordinate& pos1, const GPSCoordinate& pos2);
    
    /**
     * @brief Calculate bearing from pos1 to pos2
     * @return Bearing in degrees (0-360)
     */
    static double calculateBearing(const GPSCoordinate& pos1, const GPSCoordinate& pos2);

private:
    TinyGPSPlus gps;
    HardwareSerial* gps_serial;
    
    GPSSourceMode source_mode;
    GPSCoordinate current_position;
    GPSCoordinate static_position;
    GPSCoordinate manual_position;
    
    bool position_valid;
    uint64_t last_update_time;
    
    void updateFromHardware();
    void updateFromStatic();
    void updateFromManual();
};

#endif // GPS_MANAGER_H
