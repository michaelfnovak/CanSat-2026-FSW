#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

// Initialize telemetry system
void initTelemetry();

// Telemetry control (required: CX command)
void setTelemetryEnabled(bool enabled);
bool isTelemetryEnabled();

// Check if telemetry is active/connected
bool telemetryActive();

// Packet count management (required: F1)
// Must be maintained through processor resets
uint32_t getPacketCount();
void resetPacketCount();  // Called when Cansat installed on launch pad
void incrementPacketCount();

// Telemetry mode (required: F4-F6)
enum TelemetryMode {
    MODE_FLIGHT = 'F',
    MODE_SIMULATION = 'S'
};
void setTelemetryMode(TelemetryMode mode);
TelemetryMode getTelemetryMode();

// Command echo (required: CMD_ECHO field)
void setCommandEcho(const char* cmd);
const char* getCommandEcho();

// Send telemetry data packet at 1 Hz (required: X4, C9)
// Format: TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE, 
//         PRESSURE, VOLTAGE, CURRENT, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y,
//         GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS, CMD_ECHO [,OPTIONAL_DATA]
void sendTelemetry();

// Update telemetry system (call periodically)
void updateTelemetry();

#endif // TELEMETRY_H
