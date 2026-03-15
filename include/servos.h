#ifndef SERVOS_H
#define SERVOS_H

// Initialize servo control system
void initServos();

// Reset servos to default/initial position (called when entering LAUNCH_PAD)
void resetServos();

// Probe hatch (Servo 3) and egg release (Servo 4) — called from updateServos()
// when flight state is PROBE_RELEASE / PAYLOAD_RELEASE (state is set in StateLogic;
// main loop calls updateServos() which runs these).
void releaseProbe();
void releasePayload();

// MEC command test control of flight-surface servos (FS1, FS2)
void setFlightSurface1Test(bool active);
void setFlightSurface2Test(bool active);

// Paraglider steering during PROBE_RELEASE — skeleton only; extend for real control
void updateParagliderControl();

// Target for paraglider navigation (used when you implement updateParagliderControl)
void setTargetLocation(float targetLat, float targetLon);

// Called every main loop iteration; runs releaseProbe/releasePayload and paraglider logic by state
void updateServos();

#endif // SERVOS_H
