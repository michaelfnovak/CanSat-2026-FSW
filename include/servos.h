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

// Update servo positions (call periodically)
void updateServos();

#endif // SERVOS_H
