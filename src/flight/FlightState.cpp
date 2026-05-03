#include "FlightState.h"
#include <Arduino.h>
#include <EEPROM.h>

// Global flight state variable
FlightState flightState = PRELAUNCH;

// EEPROM address for persistent flight state
// Reserve address 52 for FlightState enum (stored as uint8_t).
const int EEPROM_FLIGHT_STATE_ADDR = 52;

void initFlightState() {
    // Restore flight state from EEPROM.
    uint8_t stored = EEPROM.read(EEPROM_FLIGHT_STATE_ADDR);

    if (stored <= (uint8_t)LANDED) {
        flightState = static_cast<FlightState>(stored);
    } else {
        // If invalid, default to PRELAUNCH and persist it.
        flightState = PRELAUNCH;
        EEPROM.write(EEPROM_FLIGHT_STATE_ADDR, (uint8_t)flightState);
    }
}

void setFlightState(FlightState state) {
    flightState = state;
    EEPROM.write(EEPROM_FLIGHT_STATE_ADDR, (uint8_t)state);
}

// Convert flight state to ASCII string for telemetry (as required by rules)
const char* flightStateToString(FlightState state) {
    switch (state) {
        case PRELAUNCH:       return "PRELAUNCH";
        case LAUNCH_PAD:      return "LAUNCH_PAD";
        case ASCENT:          return "ASCENT";
        case APOGEE:          return "APOGEE";
        case DESCENT:         return "DESCENT";
        case PROBE_RELEASE:   return "PROBE_RELEASE";
        case PAYLOAD_RELEASE: return "PAYLOAD_RELEASE";
        case LANDED:          return "LANDED";
        default:              return "UNKNOWN";
    }
}
