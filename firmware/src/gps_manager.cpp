/**
 * @file gps_manager.cpp
 * @brief GPS management implementation
 */

#include "gps_manager.h"
#include <Preferences.h>
#include <math.h>

#define EARTH_RADIUS_M 6371000.0  // Earth's radius in meters

GPSManager::GPSManager()
    : gps_serial(nullptr)
    , source_mode(GPS_SOURCE_MANUAL)  // Start in manual mode for testing
    , position_valid(false)
    , last_update_time(0) {
    
    memset(&current_position, 0, sizeof(GPSCoordinate));
    memset(&static_position, 0, sizeof(GPSCoordinate));
    memset(&manual_position, 0, sizeof(GPSCoordinate));
}

bool GPSManager::init() {
#if HAS_GPS
    // Initialize GPS UART
    gps_serial = &Serial2;
    gps_serial->begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    LOG_I("GPS UART initialized (RX=%d, TX=%d)", GPS_RX_PIN, GPS_TX_PIN);
    
    // Try to load static position from NVS
    if (loadStaticPositionFromNVS()) {
        LOG_I("Loaded static GPS position from NVS");
    }
    
    // Default to hardware GPS if available
    source_mode = GPS_SOURCE_HARDWARE;
#else
    LOG_I("GPS hardware not available, using manual mode");
    source_mode = GPS_SOURCE_MANUAL;
#endif
    
    return true;
}

void GPSManager::update() {
    switch (source_mode) {
        case GPS_SOURCE_HARDWARE:
            updateFromHardware();
            break;
        case GPS_SOURCE_STATIC:
            updateFromStatic();
            break;
        case GPS_SOURCE_MANUAL:
            updateFromManual();
            break;
    }
}

void GPSManager::updateFromHardware() {
#if HAS_GPS
    if (!gps_serial) return;
    
    // Feed GPS parser
    while (gps_serial->available() > 0) {
        char c = gps_serial->read();
        gps.encode(c);
    }
    
    // Check if we have a valid fix
    if (gps.location.isValid() && gps.satellites.value() >= GPS_MIN_SATELLITES) {
        current_position.latitude = gps.location.lat();
        current_position.longitude = gps.location.lng();
        current_position.altitude = gps.altitude.meters();
        current_position.accuracy = gps.hdop.hdop();
        current_position.timestamp = ProtobufHandler::getCurrentTimestamp();
        
        position_valid = true;
        last_update_time = millis();
        
        LOG_V("GPS: lat=%.6f, lon=%.6f, alt=%.1f, sats=%d",
              current_position.latitude, current_position.longitude,
              current_position.altitude, gps.satellites.value());
    } else {
        position_valid = false;
    }
#endif
}

void GPSManager::updateFromStatic() {
    current_position = static_position;
    current_position.timestamp = ProtobufHandler::getCurrentTimestamp();
    position_valid = true;
}

void GPSManager::updateFromManual() {
    current_position = manual_position;
    current_position.timestamp = ProtobufHandler::getCurrentTimestamp();
    position_valid = true;
}

bool GPSManager::getPosition(GPSCoordinate& coord) {
    if (!position_valid) {
        return false;
    }
    
    coord = current_position;
    return true;
}

void GPSManager::setSourceMode(GPSSourceMode mode) {
    source_mode = mode;
    LOG_I("GPS source mode set to: %d", mode);
}

bool GPSManager::setStaticPosition(double lat, double lon, double alt) {
    static_position.latitude = lat;
    static_position.longitude = lon;
    static_position.altitude = alt;
    static_position.accuracy = 1.0; // High accuracy for static
    static_position.timestamp = ProtobufHandler::getCurrentTimestamp();
    
    // Save to NVS
    saveStaticPositionToNVS();
    
    LOG_I("Static GPS position set: %.6f, %.6f", lat, lon);
    return true;
}

bool GPSManager::setManualPosition(double lat, double lon, double alt) {
    manual_position.latitude = lat;
    manual_position.longitude = lon;
    manual_position.altitude = alt;
    manual_position.accuracy = 1.0;
    manual_position.timestamp = ProtobufHandler::getCurrentTimestamp();
    
    LOG_I("Manual GPS position set: %.6f, %.6f", lat, lon);
    return true;
}

bool GPSManager::loadStaticPositionFromNVS() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, true)) {
        return false;
    }
    
    size_t len = prefs.getBytesLength(NVS_KEY_STATIC_GPS);
    if (len != sizeof(GPSCoordinate)) {
        prefs.end();
        return false;
    }
    
    prefs.getBytes(NVS_KEY_STATIC_GPS, &static_position, sizeof(GPSCoordinate));
    prefs.end();
    
    return true;
}

bool GPSManager::saveStaticPositionToNVS() {
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, false)) {
        LOG_E("Failed to open NVS");
        return false;
    }
    
    prefs.putBytes(NVS_KEY_STATIC_GPS, &static_position, sizeof(GPSCoordinate));
    prefs.end();
    
    return true;
}

bool GPSManager::hasFix() const {
    return position_valid;
}

uint8_t GPSManager::getSatelliteCount() const {
#if HAS_GPS
    // Cast away const since TinyGPSInteger::value() is not const-qualified
    TinyGPSPlus& non_const_gps = const_cast<TinyGPSPlus&>(gps);
    return non_const_gps.satellites.value();
#else
    return 0;
#endif
}

double GPSManager::calculateDistance(const GPSCoordinate& pos1, const GPSCoordinate& pos2) {
    // Haversine formula
    double lat1 = pos1.latitude * M_PI / 180.0;
    double lat2 = pos2.latitude * M_PI / 180.0;
    double dlat = (pos2.latitude - pos1.latitude) * M_PI / 180.0;
    double dlon = (pos2.longitude - pos1.longitude) * M_PI / 180.0;
    
    double a = sin(dlat/2.0) * sin(dlat/2.0) +
               cos(lat1) * cos(lat2) *
               sin(dlon/2.0) * sin(dlon/2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0-a));
    
    return EARTH_RADIUS_M * c;
}

double GPSManager::calculateBearing(const GPSCoordinate& pos1, const GPSCoordinate& pos2) {
    double lat1 = pos1.latitude * M_PI / 180.0;
    double lat2 = pos2.latitude * M_PI / 180.0;
    double dlon = (pos2.longitude - pos1.longitude) * M_PI / 180.0;
    
    double y = sin(dlon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    
    double bearing = atan2(y, x) * 180.0 / M_PI;
    
    // Normalize to 0-360
    bearing = fmod(bearing + 360.0, 360.0);
    
    return bearing;
}
