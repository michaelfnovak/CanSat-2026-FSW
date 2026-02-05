#include "FlightState.h"

// Global flight state variable
FlightState flightState = PRELAUNCH;

// Convert flight state to ASCII string for telemetry (as required by rules)
const char* flightStateToString(FlightState state) {
    switch (state) {
        case LAUNCH_PAD: return "LAUNCH_PAD";
        case ASCENT: return "ASCENT";
        case APOGEE: return "APOGEE";
        case DESCENT: return "DESCENT";
        case PROBE_RELEASE: return "PROBE_RELEASE";
        case PAYLOAD_RELEASE: return "PAYLOAD_RELEASE";
        case LANDED: return "LANDED";
        default: return "UNKNOWN";
    }
}
