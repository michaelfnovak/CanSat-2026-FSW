#ifndef SERVOS_H
#define SERVOS_H

// Attach pins, center flight surfaces, close probe/payload; clear release flags
// and descent-controller state. Call before or with flight init; follow with
// setTargetLocation if using autonomous glide.
void initServos();

// LAUNCH_PAD (StateLogic): center surfaces, close mechanisms, reset descent state.
// Keeps landing target from setTargetLocation so guidance stays enabled.
void resetServos();

// Idempotent releases (also used by MEC). State machine drives them via updateServos().
void releaseProbe();
void releasePayload();

// MEC: bench test flight surfaces (angles 0° / 90°, not the flight clamp range).
void setFlightSurface1Test(bool active);
void setFlightSurface2Test(bool active);

// Staged paraglider control (GPS bearing, heading ref, baro vertical rate).
// Intended caller: updateServos() in PROBE_RELEASE at PARAGLIDER_UPDATE_HZ.
void updateParagliderControl();

// Landing waypoint (decimal degrees). Required for autonomous glide when GPS is valid.
void setTargetLocation(float targetLat, float targetLon);

// PROBE_RELEASE / PAYLOAD_RELEASE behavior; no-op for other FlightState values.
void updateServos();

#endif // SERVOS_H
