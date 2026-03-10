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

// Direct test control of flight-surface servos (used by MEC commands)
// When active is true, deflect to ~90 degrees; when false, return to 0 degrees.
void setFlightSurface1Test(bool active);
void setFlightSurface2Test(bool active);

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
