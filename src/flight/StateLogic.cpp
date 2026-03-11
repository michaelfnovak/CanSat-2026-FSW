#include "StateLogic.h"
#include "FlightState.h"
#include "Sensors.h"
#include "telemetry.h"
#include "Timing.h"
#include "servos.h"
#include "cameras.h"
#include <math.h>
#include <stdint.h>

static float maxAltitude = 0.0f; //starts at 0 but gets updated for comparison 
static bool apogeeLatched = false;
static bool camera1Started = false;
static bool camera2Started = false;
static bool launchPadInitialized = false;
static float launchPadAltitude = 0.0f;

void updateFlightState(uint32_t now_ms) {

    float alt = getAltitude();
    float vel = getVerticalVelocity();

    switch (flightState) {

    case PRELAUNCH:
        if (telemetryActive() && timeSetComplete()) {
            // Telemetry link is up and mission time is set; ready on the pad.
            setFlightState(LAUNCH_PAD);
            launchPadInitialized = false;  // will initialize on first pass through LAUNCH_PAD
        }
        break;

    case LAUNCH_PAD:
        // Initialize launch pad state once when we first enter LAUNCH_PAD
        if (!launchPadInitialized) {
            zeroAltitude();
            resetServos();
            // Reset camera state in case of re-flight or reset before launch
            camera1Started = false;
            camera2Started = false;
            launchPadAltitude = getAltitude();
            launchPadInitialized = true;
        } else {
            // Stay in LAUNCH_PAD until altitude starts to increase above pad level.
            // A small threshold (e.g., 5 m) avoids noise.
            const float ASCENT_THRESHOLD_M = 5.0f;
            if (alt >= launchPadAltitude + ASCENT_THRESHOLD_M) {
                setFlightState(ASCENT);
            }
        }
        break;

    case ASCENT:
        if (alt > maxAltitude) {
            maxAltitude = alt;
        }

        if (vel < 0 && !apogeeLatched) {
            setFlightState(APOGEE);
            apogeeLatched = true;
        }
        break;

    case APOGEE:
        // Start camera 1 (payload separation camera) at apogee
        // so it captures separation and early descent.
        if (!camera1Started) {
            startCamera1Recording();
            camera1Started = true;
        }
        setFlightState(DESCENT);
        break;

    case DESCENT:
        if (alt <= 0.8f * maxAltitude) {
            setFlightState(PROBE_RELEASE);
        }
        break;

    case PROBE_RELEASE:
        if (alt <= 2.0f) {
            setFlightState(PAYLOAD_RELEASE);
        }
        break;

    case PAYLOAD_RELEASE:
        if (fabs(vel) < 0.05f) {
            setFlightState(LANDED);
        }
        // Start camera 2 (egg release camera) at payload release so it
        // captures the instrument (egg) drop and touchdown.
        if (!camera2Started) {
            startCamera2Recording();
            camera2Started = true;
        }
        break;

    case LANDED:
        // Stop cameras once we confirm landing (low vertical velocity).
        if (camera1Started) {
            stopCamera1Recording();
            camera1Started = false;
        }
        if (camera2Started) {
            stopCamera2Recording();
            camera2Started = false;
        }
        break;
    }
}
