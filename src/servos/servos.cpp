// Servos: probe hatch (3), egg release (4), flight surfaces (1, 2).
//
// FlightState (StateLogic) selects the regime; main calls updateServos() after
// updateFlightState(). PROBE_RELEASE: open probe once, then run staged paraglider
// descent (internal DescentStage) at PARAGLIDER_UPDATE_HZ with GPS + baro + heading
// from Sensors (getHeadingReferenceDeg). PAYLOAD_RELEASE: egg release + neutral
// surfaces. MEC commands may call releaseProbe/releasePayload or FS test helpers.
// setTargetLocation (e.g. from setup) sets the landing waypoint; resetServos()
// does not clear it so LAUNCH_PAD still has a valid target.

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

// Gyro yaw-rate damping on lateral command (deg/s -> deg command)
static const float GYRO_DAMP_GAIN_FAR    = 0.30f;
static const float GYRO_DAMP_GAIN_MID    = 0.40f;
static const float GYRO_DAMP_GAIN_FINAL  = 0.50f;

// Heading-error gain when GPS COG or IMU heading is available (deg command per deg error)
static const float HEADING_KP_FAR        = 0.32f;
static const float HEADING_KP_MID        = 0.38f;
static const float HEADING_KP_SPIRAL     = 0.42f;
static const float HEADING_KP_FINAL      = 0.28f;
static const float HEADING_KP_RELEASE    = 0.24f;

// Fallback coarse turn when no heading reference (bearing hemisphere only)
static const float BASE_TURN_FAR         = 10.0f;
static const float BASE_TURN_MID         = 14.0f;
static const float BASE_TURN_SPIRAL      = 16.0f;
static const float BASE_TURN_FINAL       = 8.0f;

// Vertical descent rate (C7): baro vz is positive up; target ~ -5 m/s
static const float VZ_CMD_MPS            = -5.0f;
static const float VZ_LPF_ALPHA          = 0.35f;
static const float VZ_KP                 = 0.85f;
static const float VZ_KI                 = 0.20f;
static const float VZ_INT_MAX            = 4.0f;
static const float VZ_DELTA_CLAMP        = 7.0f;

// Symmetric braking values
static const float SYM_BRAKE_NONE        = 0.0f;
static const float SYM_BRAKE_SPIRAL      = 3.0f;
static const float SYM_BRAKE_FINAL       = 4.0f;
static const float SYM_BRAKE_RELEASE     = 2.0f;

// Spiral mode
static const uint32_t MAX_SPIRAL_TIME_MS = 6000;

// Internal paraglider stage: baro vz near zero and low AGL (see getVerticalVelocity)
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

// Vertical-rate PI state (updated at paraglider control rate)
static float vzFiltered = 0.0f;
static float vzIntegral = 0.0f;

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
// Thin wrappers around Sensors.h for paraglider logic.

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

// ================= LATERAL (C8) + VERTICAL (C7) =================
//
// Trade study (implemented policy):
// - Primary heading reference: GPS course-over-ground when speed >= 1.2 m/s and fix is good.
//   COG matches horizontal motion toward the target and is the right control variable for
//   "steer to target" under glide.
// - Fallback: BNO055 Euler heading (mag fusion) when COG is unusable (slow drift / startup).
//   Weaker near batteries/servos and under swing; still better than bearing-hemisphere alone.
// - Last resort: bearing hemisphere sign (legacy) if both references fail.
//
// Symmetric brake baseline per stage + PI trim on baro vertical velocity toward ~5 m/s descent.

static float chooseTurnSignFromBearing(float targetBearingDeg) {
    if (targetBearingDeg < 180.0f) {
        return +1.0f;
    }
    return -1.0f;
}

// Positive command => left turn (see applyBrakeAndTurn).
static float lateralTurnCommandDeg(float bearingToTargetDeg,
                                   float headingKp,
                                   float fallbackBaseTurnDeg,
                                   float gyroDampGain,
                                   float maxTurnDeg) {
    float refHeading = 0.0f;
    uint8_t refSource = 0;
    float yawRate = currentYawRateDegPerSec();

    if (getHeadingReferenceDeg(&refHeading, &refSource)) {
        float err = wrapAngle180(bearingToTargetDeg - refHeading);
        const float deadbandDeg = 5.0f;
        if (fabsf(err) < deadbandDeg) {
            err = 0.0f;
        }
        // Turn toward bearing: err > 0 => need clockwise => negative turn command
        float cmd = -headingKp * err - gyroDampGain * yawRate;
        return clampf(cmd, -maxTurnDeg, maxTurnDeg);
    }

    float turnSign = chooseTurnSignFromBearing(bearingToTargetDeg);
    float cmd = turnSign * fallbackBaseTurnDeg - gyroDampGain * yawRate;
    return clampf(cmd, -maxTurnDeg, maxTurnDeg);
}

static float verticalSymmetricBrakeDeltaDeg(float dtSec) {
    float vz = currentVerticalVelocity();
    vzFiltered = VZ_LPF_ALPHA * vz + (1.0f - VZ_LPF_ALPHA) * vzFiltered;

    // e > 0: measured vz above target (e.g. -2 > -5) => descending too slowly => add brake
    float e = vzFiltered - VZ_CMD_MPS;
    vzIntegral += e * dtSec;
    vzIntegral = clampf(vzIntegral, -VZ_INT_MAX, VZ_INT_MAX);

    float delta = VZ_KP * e + VZ_KI * vzIntegral;
    return clampf(delta, -VZ_DELTA_CLAMP, VZ_DELTA_CLAMP);
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

// Clears targetSet until setTargetLocation() (e.g. end of setup() in main).
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
    vzFiltered = 0.0f;
    vzIntegral = 0.0f;
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
    // Keep targetSet / target lat-lon: set once in setup (or commands); LAUNCH_PAD
    // reset must not disable paraglider navigation for the real flight.

    descentStage = DEPLOY_STABILIZE;
    stageEntryMs = millis();
    probeReleaseMs = 0;
    spiralEntryMs = 0;
    lastParagliderUpdate = 0;
    vzFiltered = 0.0f;
    vzIntegral = 0.0f;
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
    const float paragliderDtSec = 1.0f / PARAGLIDER_UPDATE_HZ;
    const float maxTurn = 18.0f;

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

    switch (descentStage) {
        case DEPLOY_STABILIZE:
            setParagliderNeutral();
            break;

        case FAR_ACQUIRE: {
            float vzTrim = verticalSymmetricBrakeDeltaDeg(paragliderDtSec);
            float sym = clampf(SYM_BRAKE_NONE + vzTrim, 0.0f, SERVO_MAX_ANGLE - SERVO_CENTER);
            float turnCmd = lateralTurnCommandDeg(desiredBearing,
                                                  HEADING_KP_FAR,
                                                  BASE_TURN_FAR,
                                                  GYRO_DAMP_GAIN_FAR,
                                                  maxTurn);
            applyBrakeAndTurn(sym, turnCmd);
            break;
        }

        case MID_APPROACH: {
            float vzTrim = verticalSymmetricBrakeDeltaDeg(paragliderDtSec);
            float sym = clampf(SYM_BRAKE_NONE + vzTrim, 0.0f, SERVO_MAX_ANGLE - SERVO_CENTER);
            float turnCmd = lateralTurnCommandDeg(desiredBearing,
                                                  HEADING_KP_MID,
                                                  BASE_TURN_MID,
                                                  GYRO_DAMP_GAIN_MID,
                                                  maxTurn);
            applyBrakeAndTurn(sym, turnCmd);
            break;
        }

        case SPIRAL_ENERGY_MANAGEMENT: {
            float vzTrim = verticalSymmetricBrakeDeltaDeg(paragliderDtSec);
            float sym = clampf(SYM_BRAKE_SPIRAL + vzTrim, 0.0f, SERVO_MAX_ANGLE - SERVO_CENTER);
            float turnCmd = lateralTurnCommandDeg(desiredBearing,
                                                  HEADING_KP_SPIRAL,
                                                  BASE_TURN_SPIRAL,
                                                  GYRO_DAMP_GAIN_MID,
                                                  maxTurn);
            applyBrakeAndTurn(sym, turnCmd);

            if ((now - spiralEntryMs) > MAX_SPIRAL_TIME_MS && d_target > SPIRAL_DISTANCE_M) {
                enterDescentStage(MID_APPROACH);
            }
            break;
        }

        case FINAL_APPROACH: {
            float vzTrim = verticalSymmetricBrakeDeltaDeg(paragliderDtSec);
            float sym = clampf(SYM_BRAKE_FINAL + vzTrim, 0.0f, SERVO_MAX_ANGLE - SERVO_CENTER);
            float turnCmd = lateralTurnCommandDeg(desiredBearing,
                                                  HEADING_KP_FINAL,
                                                  BASE_TURN_FINAL,
                                                  GYRO_DAMP_GAIN_FINAL,
                                                  maxTurn);
            applyBrakeAndTurn(sym, turnCmd);
            break;
        }

        case RELEASE_ZONE: {
            if (!payloadReleased && h_agl <= EGG_RELEASE_ALT_M) {
                releasePayload();
            }

            float vzTrim = verticalSymmetricBrakeDeltaDeg(paragliderDtSec);
            float sym = clampf(SYM_BRAKE_RELEASE + vzTrim, 0.0f, SERVO_MAX_ANGLE - SERVO_CENTER);
            float turnCmd = lateralTurnCommandDeg(desiredBearing,
                                                  HEADING_KP_RELEASE,
                                                  BASE_TURN_FINAL * 0.6f,
                                                  GYRO_DAMP_GAIN_FINAL,
                                                  maxTurn);
            applyBrakeAndTurn(sym, turnCmd);
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