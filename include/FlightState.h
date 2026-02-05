#ifndef FLIGHTSTATE_H
#define FLIGHTSTATE_H

// Flight state enumeration (as required by CanSat 2026 rules)
// States telemetered as ASCII text: LAUNCH_PAD, ASCENT, APOGEE, DESCENT, PROBE_RELEASE, PAYLOAD_RELEASE, LANDED
enum FlightState {
    PRELAUNCH,      // Internal state before launch pad
    LAUNCH_READY,   // Internal state - ready for launch
    LAUNCH_PAD,     // On launch pad, ready to launch
    ASCENT,         // Rocket ascending
    APOGEE,         // Peak altitude reached
    DESCENT,        // Descending with parachute
    PROBE_RELEASE,  // At 80% peak altitude - probe released
    PAYLOAD_RELEASE,// At 2m altitude - payload (egg) released
    LANDED          // Landed on ground
};

// Global flight state variable
extern FlightState flightState;

// Convert flight state to ASCII string for telemetry
const char* flightStateToString(FlightState state);

#endif // FLIGHTSTATE_H
