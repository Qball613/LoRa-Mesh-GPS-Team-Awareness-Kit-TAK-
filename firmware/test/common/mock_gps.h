#ifndef MOCK_GPS_H
#define MOCK_GPS_H

#include "../../include/gps_manager.h"

/**
 * Mock GPS Manager for Unit Testing
 * Simulates GPS behavior without hardware
 */
class MockGPS {
public:
    MockGPS() : 
        has_fix(true),
        latitude(37.7749),   // San Francisco
        longitude(-122.4194),
        altitude(10.0),
        speed(0.0),
        heading(0.0),
        update_count(0) {}
    
    // Get current position
    GPSCoordinate getPosition() const {
        GPSCoordinate coord;
        coord.latitude = latitude;
        coord.longitude = longitude;
        coord.altitude = altitude;
        return coord;
    }
    
    // Set test position
    void setPosition(double lat, double lon, double alt = 0.0) {
        latitude = lat;
        longitude = lon;
        altitude = alt;
        update_count++;
    }
    
    // Set test velocity
    void setVelocity(double spd, double hdg) {
        speed = spd;
        heading = hdg;
    }
    
    // Get velocity
    double getSpeed() const { return speed; }
    double getHeading() const { return heading; }
    
    // GPS fix status
    bool hasFix() const { return has_fix; }
    void setFixStatus(bool fix) { has_fix = fix; }
    
    // Test helpers
    void clear() {
        update_count = 0;
    }
    
    size_t getUpdateCount() const { return update_count; }
    
    // Preset test locations
    void setSanFrancisco() {
        setPosition(37.7749, -122.4194, 10.0);
    }
    
    void setLosAngeles() {
        setPosition(34.0522, -118.2437, 15.0);
    }
    
    void setNewYork() {
        setPosition(40.7128, -74.0060, 20.0);
    }

private:
    bool has_fix;
    double latitude;
    double longitude;
    double altitude;
    double speed;
    double heading;
    size_t update_count;
};

#endif // MOCK_GPS_H
