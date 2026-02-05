#include "Sensors.h"
#include <Arduino.h>
#include <math.h>

// Sensor state variables
static float currentAltitude = 0.0f;
static float currentPressure = 101.325f;  // kPa, sea level default
static float currentTemperature = 20.0f;  // °C
static float currentVoltage = 0.0f;
static float currentCurrent = 0.0f;
static float currentGyroR = 0.0f, currentGyroP = 0.0f, currentGyroY = 0.0f;
static float currentAccelR = 0.0f, currentAccelP = 0.0f, currentAccelY = 0.0f;
static float altitudeOffset = 0.0f;
static float previousAltitude = 0.0f;
static uint32_t lastUpdateTime = 0;

// GPS data
static uint8_t gpsHour = 0, gpsMinute = 0, gpsSecond = 0;
static float gpsAltitude = 0.0f;
static float gpsLatitude = 0.0f;
static float gpsLongitude = 0.0f;
static uint8_t gpsSatellites = 0;

// Simulation mode (required: F4-F6)
static bool simulationModeEnabled = false;
static bool simulationModeActive = false;
static float simulatedPressure = 101325.0f;  // Pascals

void initSensors() {
    // TODO: Initialize sensor hardware
    // - Barometric pressure sensor (for altitude)
    // - Temperature sensor
    // - Battery voltage/current monitoring
    // - IMU (gyro + accelerometer)
    // - GPS module
    // Initialize I2C, SPI, or other communication protocols
    
    altitudeOffset = 0.0f;
    previousAltitude = 0.0f;
    lastUpdateTime = millis();
}

float getAltitude() {
    return currentAltitude;
}

float getPressure() {
    // In simulation mode, return simulated pressure converted to kPa
    if (simulationModeActive) {
        return simulatedPressure / 1000.0f;  // Convert Pa to kPa
    }
    return currentPressure;
}

float getTemperature() {
    return currentTemperature;
}

float getBatteryVoltage() {
    return currentVoltage;
}

float getBatteryCurrent() {
    return currentCurrent;
}

float getGyroRoll() {
    return currentGyroR;
}

float getGyroPitch() {
    return currentGyroP;
}

float getGyroYaw() {
    return currentGyroY;
}

float getAccelRoll() {
    return currentAccelR;
}

float getAccelPitch() {
    return currentAccelP;
}

float getAccelYaw() {
    return currentAccelY;
}

bool getGPSTime(uint8_t& hour, uint8_t& minute, uint8_t& second) {
    hour = gpsHour;
    minute = gpsMinute;
    second = gpsSecond;
    return gpsSatellites > 0;  // Return true if GPS has fix
}

float getGPSAltitude() {
    return gpsAltitude;
}

float getGPSLatitude() {
    return gpsLatitude;
}

float getGPSLongitude() {
    return gpsLongitude;
}

uint8_t getGPSSatellites() {
    return gpsSatellites;
}

float getVerticalVelocity() {
    // Calculate vertical velocity from altitude change
    uint32_t now = millis();
    if (lastUpdateTime > 0 && now > lastUpdateTime) {
        float dt = (now - lastUpdateTime) / 1000.0f;  // Convert to seconds
        if (dt > 0) {
            return (currentAltitude - previousAltitude) / dt;
        }
    }
    return 0.0f;
}

void zeroAltitude() {
    // TODO: Calibrate altitude to zero at launch pad (required: G1, CAL command)
    // Store current altitude reading as offset
    altitudeOffset = currentAltitude;
    currentAltitude = 0.0f;
    previousAltitude = 0.0f;
}

void setSimulationMode(bool enabled) {
    simulationModeEnabled = enabled;
}

bool isSimulationMode() {
    return simulationModeActive;
}

void setSimulatedPressure(float pressure_pa) {
    simulatedPressure = pressure_pa;
    // TODO: Calculate altitude from simulated pressure
    // Use barometric formula: altitude = 44330 * (1 - (P/P0)^0.1903)
    // where P0 = sea level pressure (typically 101325 Pa)
    const float P0 = 101325.0f;  // Sea level pressure in Pa
    currentAltitude = 44330.0f * (1.0f - powf(pressure_pa / P0, 0.1903f));
    currentPressure = pressure_pa / 1000.0f;  // Convert to kPa
}

void updateSensors() {
    // TODO: Read sensor data and update all sensor values
    // This should be called periodically from main loop
    
    uint32_t now = millis();
    
    if (!simulationModeActive) {
        // TODO: Read actual sensor values
        // - Read barometric pressure sensor
        // - Read temperature sensor
        // - Read battery voltage/current
        // - Read IMU (gyro + accelerometer)
        // - Read GPS module
        
        // Calculate altitude from pressure (barometric formula)
        // altitude = 44330 * (1 - (P/P0)^0.1903)
        const float P0 = 101325.0f;  // Sea level pressure in Pa
        float pressurePa = currentPressure * 1000.0f;  // Convert kPa to Pa
        float calculatedAltitude = 44330.0f * (1.0f - powf(pressurePa / P0, 0.1903f));
        currentAltitude = calculatedAltitude - altitudeOffset;
    }
    
    // Update vertical velocity calculation
    if (lastUpdateTime > 0) {
        previousAltitude = currentAltitude;
    }
    lastUpdateTime = now;
}
