#include "telemetry.h"
#include "FlightState.h"
#include "Sensors.h"
#include "Timing.h"
#include "Commands.h"
#include "XBee.h"
#include <Arduino.h>
#include <EEPROM.h>  // Teensy 4.1 EEPROM library
#include <stdio.h>
#include <string.h>

static bool telemetryEnabled = false;
static uint32_t packetCount = 0;
static TelemetryMode telemetryMode = MODE_FLIGHT;
static char commandEcho[32] = "";
static uint16_t teamID = 0;

// EEPROM address for packet count persistence (required: F1)
// Reserve addresses 20-23 for packet count (4 bytes)
const int EEPROM_PACKET_COUNT_ADDR = 20;

void initTelemetry() {
    // Initialize telemetry system
    // Set up communication link (XBee, radio, etc.)
    telemetryEnabled = false;
    telemetryMode = MODE_FLIGHT;
    commandEcho[0] = '\0';
    
    // Restore packet count from EEPROM (required: F1)
    // Teensy 4.1 EEPROM - read 4 bytes for uint32_t packet count
    packetCount = (uint32_t)EEPROM.read(EEPROM_PACKET_COUNT_ADDR) | 
                  ((uint32_t)EEPROM.read(EEPROM_PACKET_COUNT_ADDR + 1) << 8) |
                  ((uint32_t)EEPROM.read(EEPROM_PACKET_COUNT_ADDR + 2) << 16) |
                  ((uint32_t)EEPROM.read(EEPROM_PACKET_COUNT_ADDR + 3) << 24);
}

void setTelemetryEnabled(bool enabled) {
    telemetryEnabled = enabled;
}

bool isTelemetryEnabled() {
    return telemetryEnabled;
}

bool telemetryActive() {
    return telemetryEnabled && xbeeReady();
}

uint32_t getPacketCount() {
    return packetCount;
}

void resetPacketCount() {
    // Reset packet count when installed on launch pad (required: F1)
    packetCount = 0;
    
    // Save to EEPROM (Teensy 4.1 - writes are immediate)
    EEPROM.write(EEPROM_PACKET_COUNT_ADDR, packetCount & 0xFF);
    EEPROM.write(EEPROM_PACKET_COUNT_ADDR + 1, (packetCount >> 8) & 0xFF);
    EEPROM.write(EEPROM_PACKET_COUNT_ADDR + 2, (packetCount >> 16) & 0xFF);
    EEPROM.write(EEPROM_PACKET_COUNT_ADDR + 3, (packetCount >> 24) & 0xFF);
}

void incrementPacketCount() {
    packetCount++;
    
    // Save to EEPROM periodically (required: F1)
    // Save every 10 packets to reduce EEPROM wear (Teensy 4.1 EEPROM is flash-based)
    // Note: Flash has limited write cycles, so we minimize writes
    if (packetCount % 10 == 0) {
        EEPROM.write(EEPROM_PACKET_COUNT_ADDR, packetCount & 0xFF);
        EEPROM.write(EEPROM_PACKET_COUNT_ADDR + 1, (packetCount >> 8) & 0xFF);
        EEPROM.write(EEPROM_PACKET_COUNT_ADDR + 2, (packetCount >> 16) & 0xFF);
        EEPROM.write(EEPROM_PACKET_COUNT_ADDR + 3, (packetCount >> 24) & 0xFF);
    }
}

void setTelemetryMode(TelemetryMode mode) {
    telemetryMode = mode;
}

TelemetryMode getTelemetryMode() {
    return telemetryMode;
}

void setCommandEcho(const char* cmd) {
    if (cmd != nullptr) {
        strncpy(commandEcho, cmd, sizeof(commandEcho) - 1);
        commandEcho[sizeof(commandEcho) - 1] = '\0';
    } else {
        commandEcho[0] = '\0';
    }
}

const char* getCommandEcho() {
    return commandEcho;
}

void sendTelemetry() {
    if (!telemetryEnabled) {
        return;
    }
    
    // Format telemetry packet as required (Section 3.1.1.1)
    // TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE, 
    // PRESSURE, VOLTAGE, CURRENT, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y,
    // GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS, CMD_ECHO [,OPTIONAL_DATA]
    
    char buffer[512];
    char missionTimeStr[9];
    char gpsTimeStr[9];
    
    // Get mission time
    uint8_t h, m, s;
    if (getMissionTime(h, m, s)) {
        snprintf(missionTimeStr, sizeof(missionTimeStr), "%02d:%02d:%02d", h, m, s);
    } else {
        strcpy(missionTimeStr, "00:00:00");
    }
    
    // Get GPS time
    uint8_t gpsH, gpsM, gpsS;
    bool gpsValid = getGPSTime(gpsH, gpsM, gpsS);
    if (gpsValid) {
        snprintf(gpsTimeStr, sizeof(gpsTimeStr), "%02d:%02d:%02d", gpsH, gpsM, gpsS);
    } else {
        strcpy(gpsTimeStr, "00:00:00");
    }
    
    // Format telemetry packet
    snprintf(buffer, sizeof(buffer),
        "%04d,%s,%lu,%c,%s,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%.1f,%.4f,%.4f,%d,%s\r\n",
        teamID,                                    // TEAM_ID
        missionTimeStr,                           // MISSION_TIME
        packetCount,                              // PACKET_COUNT
        (telemetryMode == MODE_FLIGHT) ? 'F' : 'S', // MODE
        flightStateToString(flightState),         // STATE
        getAltitude(),                            // ALTITUDE (0.1m resolution)
        getTemperature(),                         // TEMPERATURE (0.1°C resolution)
        getPressure(),                            // PRESSURE (0.1 kPa resolution)
        getBatteryVoltage(),                      // VOLTAGE (0.1V resolution)
        getBatteryCurrent(),                      // CURRENT (0.01A resolution)
        getGyroRoll(),                           // GYRO_R
        getGyroPitch(),                          // GYRO_P
        getGyroYaw(),                            // GYRO_Y
        getAccelRoll(),                          // ACCEL_R
        getAccelPitch(),                         // ACCEL_P
        getAccelYaw(),                           // ACCEL_Y
        gpsTimeStr,                              // GPS_TIME
        getGPSAltitude(),                        // GPS_ALTITUDE (0.1m resolution)
        getGPSLatitude(),                         // GPS_LATITUDE (0.0001° resolution)
        getGPSLongitude(),                        // GPS_LONGITUDE (0.0001° resolution)
        getGPSSatellites(),                       // GPS_SATS
        commandEcho                               // CMD_ECHO
    );
    
    // Send via XBee
    xbeeSend((const uint8_t*)buffer, strlen(buffer));
    incrementPacketCount();
}

void updateTelemetry() {
    // TODO: Check telemetry connection status
    // Update connection state, handle errors, etc.
}
