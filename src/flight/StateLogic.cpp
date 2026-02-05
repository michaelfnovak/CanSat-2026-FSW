#include "StateLogic.h"
#include "FlightState.h"
#include "Sensors.h"
#include "telemetry.h"
#include "Timing.h"
#include "servos.h"
#include <math.h>
#include <stdint.h>

static float maxAltitude = 0.0f; //starts at 0 but gets updated for comparison 
static bool apogeeLatched = false;

void updateFlightState(uint32_t now_ms) {

    float alt = getAltitude();
    float vel = getVerticalVelocity();

    switch (flightState) {

    case PRELAUNCH:
        if (telemetryActive() && timeSetComplete()) {
            flightState = LAUNCH_READY;
        }
        break;

    case LAUNCH_PAD:
        zeroAltitude();
        resetServos();
        flightState = ASCENT;
        break;

    case ASCENT:
        if (alt > maxAltitude) {
            maxAltitude = alt;
        }

        if (vel < 0 && !apogeeLatched) {
            flightState = APOGEE;
            apogeeLatched = true;
        }
        break;

    case APOGEE:
        flightState = DESCENT;
        break;

    case DESCENT:
        if (alt <= 0.8f * maxAltitude) {
            flightState = PROBE_RELEASE;
        }
        break;

    case PROBE_RELEASE:
        if (alt <= 2.0f) {
            flightState = PAYLOAD_RELEASE;
        }
        break;

    case PAYLOAD_RELEASE:
        if (fabs(vel) < 0.05f) {
            flightState = LANDED;
        }
        break;

    case LANDED:
        break;
    }
}
