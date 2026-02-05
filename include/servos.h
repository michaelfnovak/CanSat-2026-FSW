#ifndef SERVOS_H
#define SERVOS_H

// Initialize servo control system
void initServos();

// Reset servos to default/initial position
void resetServos();

// Release probe (typically at 80% of max altitude)
void releaseProbe();

// Release payload (typically at 2m altitude)
void releasePayload();

// Paraglider control (during PROBE_RELEASE phase)
// Controls two servos for steering the paraglider toward target location
void updateParagliderControl();

// Set target GPS coordinates for paraglider navigation
void setTargetLocation(float targetLat, float targetLon);

// Get current servo positions (for debugging/telemetry)
void getParagliderServoPositions(float& servo1Pos, float& servo2Pos);

// Update servo positions (call periodically)
void updateServos();

#endif // SERVOS_H
