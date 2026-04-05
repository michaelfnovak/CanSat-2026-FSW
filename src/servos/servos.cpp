/*
 * Servos: probe hatch (3), egg release (4), flight surfaces (1, 2).
 * StateLogic sets flightState (e.g. PROBE_RELEASE, PAYLOAD_RELEASE). Main loop
 * calls updateServos(); it switches on flightState and calls releaseProbe(),
 * releasePayload(), and updateParagliderControl() as appropriate.
 */
#include "servos.h"
#include "FlightState.h"
#include "Sensors.h"
#include <Arduino.h>
#include <Servo.h>

// Pins (Teensy 4.1; adjust to match wiring)
const int SERVO_PROBE_PIN        = 2;  // Servo 3: probe release hatch
const int SERVO_PAYLOAD_PIN      = 3;  // Servo 4: egg release
const int SERVO_PARAGLIDER_1_PIN = 4;  // Servo 1: flight surface
const int SERVO_PARAGLIDER_2_PIN = 5;  // Servo 2: flight surface

const float SERVO_CENTER = 90.0f;
const float SERVO_MIN_ANGLE = 0.0f;
const float SERVO_MAX_ANGLE = 180.0f;

static Servo servoProbe;
static Servo servoPayload;
static Servo servoParaglider1;
static Servo servoParaglider2;

static float servo1Position = SERVO_CENTER;
static float servo2Position = SERVO_CENTER;
static bool probeReleased = false;
static bool payloadReleased = false;

// Paraglider skeleton: target and rate limit only
static float targetLatitude = 0.0f;
static float targetLongitude = 0.0f;
static bool targetSet = false;
static const float PARAGLIDER_UPDATE_HZ = 10.0f;
static uint32_t lastParagliderUpdate = 0;

void initServos() {
    servoProbe.attach(SERVO_PROBE_PIN);
    servoPayload.attach(SERVO_PAYLOAD_PIN);
    servoParaglider1.attach(SERVO_PARAGLIDER_1_PIN);
    servoParaglider2.attach(SERVO_PARAGLIDER_2_PIN);

    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
    servoProbe.write(0);
    servoPayload.write(0);

    probeReleased = false;
    payloadReleased = false;
    targetSet = false;
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
            servo1Position = SERVO_CENTER;
            servo2Position = SERVO_CENTER;
            servoParaglider1.write((int)servo1Position);
            servoParaglider2.write((int)servo2Position);
            break;
        default:
            break;
    }
}

void resetServos() {
    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
    servoProbe.write(0);
    servoPayload.write(0);
    probeReleased = false;
    payloadReleased = false;
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
        servoProbe.write(90);
        probeReleased = true;
    }
}

void releasePayload() {
    if (!payloadReleased) {
        servoPayload.write(90);
        payloadReleased = true;
    }
}

void setTargetLocation(float targetLat, float targetLon) {
    targetLatitude = targetLat;
    targetLongitude = targetLon;
    targetSet = true;
}

// Skeleton: keep flight surfaces neutral during PROBE_RELEASE.
// Replace with your steering logic (heading to target, etc.) when ready.
void updateParagliderControl() {
    (void)targetSet;
    (void)targetLatitude;
    (void)targetLongitude;
    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
}


