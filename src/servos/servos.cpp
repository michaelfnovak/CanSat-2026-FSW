
 //Servos: probe hatch (3), egg release (4), flight surfaces (1, 2).
 //StateLogic sets flightState (e.g. PROBE_RELEASE, PAYLOAD_RELEASE). Main loop
 //calls updateServos(); it switches on flightState and calls releaseProbe(),
 //releasePayload(), and updateParagliderControl() as appropriate.
 
#include "servos.h"
#include "FlightState.h"
#include "Sensors.h"
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// ================= PINS =================

// Pins (Teensy 4.1; adjust to match wiring)
const int SERVO_PROBE_PIN        = 2;  // Servo 3: probe release hatch
const int SERVO_PAYLOAD_PIN      = 3;  // Servo 4: egg release
const int SERVO_PARAGLIDER_1_PIN = 4;  // Servo 1: left flight surface
const int SERVO_PARAGLIDER_2_PIN = 5;  // Servo 2: right flight surface

// ================= SERVO LIMITS =================

const float SERVO_CENTER     = 90.0f;
const float SERVO_MIN_ANGLE  = 60.0f;   // tune after bench test
const float SERVO_MAX_ANGLE  = 120.0f;  // tune after bench test

const float PROBE_CLOSED_ANGLE    = 0.0f;
const float PROBE_RELEASE_ANGLE   = 90.0f;
const float PAYLOAD_CLOSED_ANGLE  = 0.0f;
const float PAYLOAD_RELEASE_ANGLE = 90.0f;

// ================= UPDATE RATE =================

static const float PARAGLIDER_UPDATE_HZ = 10.0f;
static uint32_t lastParagliderUpdate = 0;

// ================= AUTONOMOUS DESCENT SETTINGS =================

// Stage timing
static const uint32_t STABILIZE_TIME_MS  = 3500;
static const uint32_t MIN_STAGE_DWELL_MS = 1200;

// Distance / altitude thresholds
static const float FAR_DISTANCE_M        = 40.0f;
static const float SPIRAL_DISTANCE_M     = 15.0f;
static const float FINAL_APPROACH_ALT_M  = 15.0f;
static const float RELEASE_ZONE_ALT_M    = 5.0f;
static const float EGG_RELEASE_ALT_M     = 2.0f;

// Guidance / control
static const float MIN_VALID_SATS        = 4.0f;

// Turn control without heading/course:
// we use bearing-to-target sign + gyro yaw-rate damping
static const float GYRO_DAMP_GAIN_FAR    = 0.30f;
static const float GYRO_DAMP_GAIN_MID    = 0.40f;
static const float GYRO_DAMP_GAIN_FINAL  = 0.50f;

// Base turn magnitudes
static const float BASE_TURN_FAR         = 10.0f;
static const float BASE_TURN_MID         = 14.0f;
static const float BASE_TURN_SPIRAL      = 16.0f;
static const float BASE_TURN_FINAL       = 8.0f;

// Symmetric braking values
static const float SYM_BRAKE_NONE        = 0.0f;
static const float SYM_BRAKE_SPIRAL      = 3.0f;
static const float SYM_BRAKE_FINAL       = 4.0f;
static const float SYM_BRAKE_RELEASE     = 2.0f;

// Spiral mode
static const uint32_t MAX_SPIRAL_TIME_MS = 6000;

// Landing detection using only current interface
static const float LANDED_ALT_M          = 1.0f;
static const float LANDED_VZ_MPS         = 0.3f;

// ================= SERVO OBJECTS =================

static Servo servoProbe;
static Servo servoPayload;
static Servo servoParaglider1;
static Servo servoParaglider2;

// ================= SERVO / MISSION STATE =================

static float servo1Position = SERVO_CENTER;
static float servo2Position = SERVO_CENTER;

static bool probeReleased = false;
static bool payloadReleased = false;

// Target
static float targetLatitude = 0.0f;
static float targetLongitude = 0.0f;
static bool targetSet = false;

// Timing
static uint32_t probeReleaseMs = 0;
static uint32_t spiralEntryMs = 0;

// ================= DESCENT STAGES =================

enum DescentStage {
    DEPLOY_STABILIZE,
    FAR_ACQUIRE,
    MID_APPROACH,
    SPIRAL_ENERGY_MANAGEMENT,
    FINAL_APPROACH,
    RELEASE_ZONE,
    LANDED_MODE
};

static DescentStage descentStage = DEPLOY_STABILIZE;
static uint32_t stageEntryMs = 0;

// ================= UTILITY FUNCTIONS =================

static float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float wrapAngle180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float wrapAngle360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double p1 = lat1 * M_PI / 180.0;
    double p2 = lat2 * M_PI / 180.0;
    double dp = (lat2 - lat1) * M_PI / 180.0;
    double dl = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dp / 2.0) * sin(dp / 2.0) +
               cos(p1) * cos(p2) * sin(dl / 2.0) * sin(dl / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;
}

static float bearingToTargetDeg(float currentLat, float currentLon,
                                float targetLat, float targetLon) {
    float lat1 = currentLat * M_PI / 180.0f;
    float lat2 = targetLat * M_PI / 180.0f;
    float dLon = (targetLon - currentLon) * M_PI / 180.0f;

    float y = sinf(dLon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);

    float bearing = atan2f(y, x) * 180.0f / M_PI;
    return wrapAngle360(bearing);
}

static void setParagliderNeutral() {
    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
}

static void applyBrakeAndTurn(float symmetricBrakeDeg, float turnCmdDeg) {
    // positive turnCmdDeg => left turn
    float leftCmd  = SERVO_CENTER + symmetricBrakeDeg + turnCmdDeg;
    float rightCmd = SERVO_CENTER + symmetricBrakeDeg - turnCmdDeg;

    leftCmd  = clampf(leftCmd, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    rightCmd = clampf(rightCmd, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    servo1Position = leftCmd;
    servo2Position = rightCmd;

    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
}

static void enterDescentStage(DescentStage nextStage) {
    descentStage = nextStage;
    stageEntryMs = millis();

    if (nextStage == SPIRAL_ENERGY_MANAGEMENT) {
        spiralEntryMs = stageEntryMs;
    }
}

// ================= SENSOR WRAPPERS =================
// Uses only currently available interface.

static bool gpsUsable() {
    return getGPSSatellites() >= (uint8_t)MIN_VALID_SATS;
}

static float currentLat() {
    return getGPSLatitude();
}

static float currentLon() {
    return getGPSLongitude();
}

static float currentAltitude() {
    return getAltitude();
}

static float currentVerticalVelocity() {
    return getVerticalVelocity();
}

static float currentYawRateDegPerSec() {
    // Existing interface already gives gyro Z in deg/s
    return getGyroYaw();
}

static bool landedDetected() {
    return (fabs(currentVerticalVelocity()) < LANDED_VZ_MPS &&
            currentAltitude() < LANDED_ALT_M);
}

// ================= TURN LOGIC =================
//
// Since current interface does NOT expose GPS course / heading,
// we cannot do true heading-error PD steering.
//
// So we use a simpler autonomous strategy:
// - compute bearing to target from current GPS position
// - use bearing hemisphere / sign relative to north as a coarse steering bias
// - use gyro yaw-rate as damping to avoid over-rotation
//
// This is less sophisticated than full heading control, but it is
// implementable using only currently available data.

static float chooseTurnSignFromBearing(float targetBearingDeg) {
    // Coarse rule:
    // Bearings in [0,180) bias one turn direction, [180,360) bias the other.
    // You must bench-test sign convention and swap if needed.
    if (targetBearingDeg < 180.0f) {
        return +1.0f;
    } else {
        return -1.0f;
    }
}

static float dampedTurnCommand(float turnSign,
                               float baseTurnMagnitude,
                               float symmetricBrake,
                               float gyroDampGain) {
    float yawRate = currentYawRateDegPerSec();

    // If turning left (positive command), positive yaw rate should reduce command.
    // If turning right (negative command), negative yaw rate should reduce magnitude.
    float rawCmd = turnSign * baseTurnMagnitude - gyroDampGain * yawRate;

    // Limit to safe turn range relative to symmetric braking.
    float maxTurn = 18.0f;
    rawCmd = clampf(rawCmd, -maxTurn, +maxTurn);

    (void)symmetricBrake;
    return rawCmd;
}

// ================= STAGE DECISION LOGIC =================

static DescentStage determineDescentStage(float h_agl, float d_target, uint32_t now) {
    if (landedDetected()) {
        return LANDED_MODE;
    }

    if ((now - probeReleaseMs) < STABILIZE_TIME_MS) {
        return DEPLOY_STABILIZE;
    }

    if (h_agl <= RELEASE_ZONE_ALT_M) {
        return RELEASE_ZONE;
    }

    if (h_agl <= FINAL_APPROACH_ALT_M) {
        return FINAL_APPROACH;
    }

    if (d_target <= SPIRAL_DISTANCE_M) {
        return SPIRAL_ENERGY_MANAGEMENT;
    }

    if (d_target <= FAR_DISTANCE_M) {
        return MID_APPROACH;
    }

    return FAR_ACQUIRE;
}

// ================= PUBLIC FUNCTIONS =================

void initServos() {
    servoProbe.attach(SERVO_PROBE_PIN);
    servoPayload.attach(SERVO_PAYLOAD_PIN);
    servoParaglider1.attach(SERVO_PARAGLIDER_1_PIN);
    servoParaglider2.attach(SERVO_PARAGLIDER_2_PIN);

    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
    servoProbe.write((int)PROBE_CLOSED_ANGLE);
    servoPayload.write((int)PAYLOAD_CLOSED_ANGLE);

    probeReleased = false;
    payloadReleased = false;
    targetSet = false;

    descentStage = DEPLOY_STABILIZE;
    stageEntryMs = millis();
    probeReleaseMs = 0;
    spiralEntryMs = 0;
    lastParagliderUpdate = 0;
}

void resetServos() {
    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
    servoProbe.write((int)PROBE_CLOSED_ANGLE);
    servoPayload.write((int)PAYLOAD_CLOSED_ANGLE);

    probeReleased = false;
    payloadReleased = false;
    targetSet = false;

    descentStage = DEPLOY_STABILIZE;
    stageEntryMs = millis();
    probeReleaseMs = 0;
    spiralEntryMs = 0;
    lastParagliderUpdate = 0;
}

void setFlightSurface1Test(bool active) {
    float angle = active ? 90.0f : 0.0f;
    servo1Position = angle;
    servoParaglider1.write((int)angle);
}

void setFlightSurface2Test(bool active) {
    float angle = active ? 90.0f : 0.0f;
    servo2Position = angle;
    servoParaglider2.write((int)angle);
}

void releaseProbe() {
    if (!probeReleased) {
        servoProbe.write((int)PROBE_RELEASE_ANGLE);
        probeReleased = true;
        probeReleaseMs = millis();
        enterDescentStage(DEPLOY_STABILIZE);
    }
}

void releasePayload() {
    if (!payloadReleased) {
        servoPayload.write((int)PAYLOAD_RELEASE_ANGLE);
        payloadReleased = true;
    }
}

void setTargetLocation(float targetLat, float targetLon) {
    targetLatitude = targetLat;
    targetLongitude = targetLon;
    targetSet = true;
}

void updateParagliderControl() {
    uint32_t now = millis();

    if (!targetSet || !gpsUsable()) {
        setParagliderNeutral();
        return;
    }

    float h_agl = currentAltitude();
    float lat = currentLat();
    float lon = currentLon();
    float d_target = (float)haversineMeters(lat, lon, targetLatitude, targetLongitude);
    float desiredBearing = bearingToTargetDeg(lat, lon, targetLatitude, targetLongitude);

    DescentStage desiredStage = determineDescentStage(h_agl, d_target, now);

    if (desiredStage != descentStage && (now - stageEntryMs) >= MIN_STAGE_DWELL_MS) {
        enterDescentStage(desiredStage);
    }

    float turnSign = chooseTurnSignFromBearing(desiredBearing);

    switch (descentStage) {
        case DEPLOY_STABILIZE:
            setParagliderNeutral();
            break;

        case FAR_ACQUIRE: {
            float turnCmd = dampedTurnCommand(turnSign,
                                              BASE_TURN_FAR,
                                              SYM_BRAKE_NONE,
                                              GYRO_DAMP_GAIN_FAR);
            applyBrakeAndTurn(SYM_BRAKE_NONE, turnCmd);
            break;
        }

        case MID_APPROACH: {
            float turnCmd = dampedTurnCommand(turnSign,
                                              BASE_TURN_MID,
                                              SYM_BRAKE_NONE,
                                              GYRO_DAMP_GAIN_MID);
            applyBrakeAndTurn(SYM_BRAKE_NONE, turnCmd);
            break;
        }

        case SPIRAL_ENERGY_MANAGEMENT: {
            // Deliberate controlled spiral near target to burn altitude.
            // Keep same turn direction while in this state.
            float turnCmd = dampedTurnCommand(turnSign,
                                              BASE_TURN_SPIRAL,
                                              SYM_BRAKE_SPIRAL,
                                              GYRO_DAMP_GAIN_MID);

            applyBrakeAndTurn(SYM_BRAKE_SPIRAL, turnCmd);

            // Safety escape if spiral lasts too long and drifted outward again
            if ((now - spiralEntryMs) > MAX_SPIRAL_TIME_MS && d_target > SPIRAL_DISTANCE_M) {
                enterDescentStage(MID_APPROACH);
            }
            break;
        }

        case FINAL_APPROACH: {
            float turnCmd = dampedTurnCommand(turnSign,
                                              BASE_TURN_FINAL,
                                              SYM_BRAKE_FINAL,
                                              GYRO_DAMP_GAIN_FINAL);
            applyBrakeAndTurn(SYM_BRAKE_FINAL, turnCmd);
            break;
        }

        case RELEASE_ZONE: {
            if (!payloadReleased && h_agl <= EGG_RELEASE_ALT_M) {
                releasePayload();
            }

            float turnCmd = dampedTurnCommand(turnSign,
                                              BASE_TURN_FINAL * 0.6f,
                                              SYM_BRAKE_RELEASE,
                                              GYRO_DAMP_GAIN_FINAL);
            applyBrakeAndTurn(SYM_BRAKE_RELEASE, turnCmd);
            break;
        }

        case LANDED_MODE:
            setParagliderNeutral();
            break;
    }
}

void updateServos() {
    switch (flightState) {
        case PROBE_RELEASE:
            releaseProbe();
            {
                uint32_t now = millis();
                if (now - lastParagliderUpdate >= (uint32_t)(1000.0f / PARAGLIDER_UPDATE_HZ)) {
                    updateParagliderControl();
                    lastParagliderUpdate = now;
                }
            }
            break;

        case PAYLOAD_RELEASE:
            releasePayload();
            setParagliderNeutral();
            break;

        default:
            break;
    }
}