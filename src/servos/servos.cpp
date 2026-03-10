#include "servos.h"
#include "FlightState.h"
#include "Sensors.h"
#include <Arduino.h>
#include <math.h>
#include <Servo.h>

// Servo control pins (adjust to match your wiring)
// These are standard PWM-capable pins on Teensy 4.1 and do not conflict with Serial1/2/3/4.
const int SERVO_PROBE_PIN        = 2;  // Servo 3: probe release hatch
const int SERVO_PAYLOAD_PIN      = 3;  // Servo 4: egg release
const int SERVO_PARAGLIDER_1_PIN = 4;  // Servo 1: flight surface (left/right)
const int SERVO_PARAGLIDER_2_PIN = 5;  // Servo 2: flight surface (pitch/roll)

// Servo position limits (adjust based on your servo specifications)
const float SERVO_MIN_ANGLE = 0.0f;    // Minimum servo angle in degrees
const float SERVO_MAX_ANGLE = 180.0f;   // Maximum servo angle in degrees
const float SERVO_CENTER = 90.0f;      // Center position in degrees

// Paraglider control parameters
static float targetLatitude = 0.0f;
static float targetLongitude = 0.0f;
static bool targetSet = false;
static float servo1Position = SERVO_CENTER;  // Current position of servo 1
static float servo2Position = SERVO_CENTER;  // Current position of servo 2

// Paraglider control gains (tune these for your system)
const float STEERING_GAIN = 1.0f;      // Gain for heading correction
const float MAX_SERVO_DEFLECTION = 30.0f; // Maximum servo deflection from center (degrees)
const float SERVO_UPDATE_RATE = 10.0f;   // Servo update rate in Hz
static uint32_t lastParagliderUpdate = 0;

static bool probeReleased = false;
static bool payloadReleased = false;

// Servo objects
static Servo servoProbe;        // Probe hatch (Servo 3)
static Servo servoPayload;      // Egg release (Servo 4)
static Servo servoParaglider1;  // Flight surface 1 (Servo 1)
static Servo servoParaglider2;  // Flight surface 2 (Servo 2)

void initServos() {
    // Attach all servos to their pins (similar to servo_tester.ino using Servo library)
    servoProbe.attach(SERVO_PROBE_PIN);
    servoPayload.attach(SERVO_PAYLOAD_PIN);
    servoParaglider1.attach(SERVO_PARAGLIDER_1_PIN);
    servoParaglider2.attach(SERVO_PARAGLIDER_2_PIN);

    // Initialize flight-surface servos to center position
    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);

    // Initialize hatch servos to "closed" (0 degrees)
    servoProbe.write(0);
    servoPayload.write(0);
    
    probeReleased = false;
    payloadReleased = false;
    targetSet = false;
}

void resetServos() {
    // Reset all servos to initial/default positions
    servo1Position = SERVO_CENTER;
    servo2Position = SERVO_CENTER;
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);

    // Close hatches
    servoProbe.write(0);
    servoPayload.write(0);

    probeReleased = false;
    payloadReleased = false;
}

void setFlightSurface1Test(bool active) {
    // Simple test: OFF = 0 degrees, ON = 90 degrees
    float angle = active ? 90.0f : 0.0f;
    servo1Position = angle;
    servoParaglider1.write((int)angle);
}

void setFlightSurface2Test(bool active) {
    // Simple test: OFF = 0 degrees, ON = 90 degrees
    float angle = active ? 90.0f : 0.0f;
    servo2Position = angle;
    servoParaglider2.write((int)angle);
}

void releaseProbe() {
    // Activate servo to release probe (at 80% peak altitude)
    // This should be called when flight state transitions to PROBE_RELEASE
    if (!probeReleased) {
        // Turn probe hatch servo by ~90 degrees to open
        servoProbe.write(90);
        probeReleased = true;
    }
}

void releasePayload() {
    // Activate servo to release payload (at 2m altitude)
    // This should be called when flight state transitions to PAYLOAD_RELEASE
    if (!payloadReleased) {
        // Turn egg-release servo by ~90 degrees to drop egg
        servoPayload.write(90);
        payloadReleased = true;
    }
}

void setTargetLocation(float targetLat, float targetLon) {
    targetLatitude = targetLat;
    targetLongitude = targetLon;
    targetSet = true;
}

void getParagliderServoPositions(float& servo1Pos, float& servo2Pos) {
    servo1Pos = servo1Position;
    servo2Pos = servo2Position;
}

// Calculate bearing from current position to target (in degrees)
float calculateBearingToTarget(float currentLat, float currentLon, 
                                float targetLat, float targetLon) {
    // Convert to radians
    float lat1 = currentLat * M_PI / 180.0f;
    float lat2 = targetLat * M_PI / 180.0f;
    float dLon = (targetLon - currentLon) * M_PI / 180.0f;
    
    // Calculate bearing using haversine formula
    float y = sinf(dLon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
    float bearing = atan2f(y, x);
    
    // Convert to degrees and normalize to 0-360
    bearing = bearing * 180.0f / M_PI;
    if (bearing < 0) {
        bearing += 360.0f;
    }
    
    return bearing;
}

// Calculate current heading from GPS and IMU data
float calculateCurrentHeading() {
    // TODO: Calculate heading from GPS course or IMU yaw
    // For now, using GPS course if available, otherwise use IMU yaw
    // You may need to combine GPS course with IMU yaw for better accuracy
    
    // Placeholder: Use GPS course if available
    // If GPS is not providing course, use IMU yaw angle
    // Note: You may need to convert IMU yaw to heading based on your coordinate system
    
    // Example: return getGPSYaw(); // If GPS provides course
    // Or: return getGyroYaw(); // If using IMU
    
    // For now, return 0 (North) as placeholder
    return 0.0f;
}

// Convert servo angle to PWM value (for Teensy 4.1 PWM)
int angleToPWM(float angle) {
    // Clamp angle to valid range
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    
    // Convert angle (0-180 degrees) to PWM value
    // Standard RC servos use 1-2ms pulse width (1000-2000 microseconds)
    // For Teensy 4.1 PWM at 50Hz (20ms period):
    // PWM value = (angle / 180.0) * (2000 - 1000) + 1000
    // Then convert to PWM register value based on your PWM frequency
    
    // TODO: Adjust this based on your PWM configuration
    // Example for analogWrite with 8-bit resolution:
    // return map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 0, 255);
    
    // For direct PWM control (if using PWM library):
    // return (int)((angle / 180.0f) * 1000.0f + 1000.0f); // microseconds
    
    return (int)angle; // Placeholder
}

void updateParagliderControl() {
    // Paraglider control logic for steering toward target location
    // This is called during PROBE_RELEASE phase
    
    if (!targetSet) {
        // No target set, keep servos centered
        servo1Position = SERVO_CENTER;
        servo2Position = SERVO_CENTER;
        return;
    }
    
    // Get current GPS position
    float currentLat = getGPSLatitude();
    float currentLon = getGPSLongitude();
    
    // Check if GPS has valid fix
    if (getGPSSatellites() < 4) {
        // Not enough satellites, keep servos centered
        servo1Position = SERVO_CENTER;
        servo2Position = SERVO_CENTER;
        return;
    }
    
    // Calculate bearing to target
    float bearingToTarget = calculateBearingToTarget(currentLat, currentLon, 
                                                      targetLatitude, targetLongitude);
    
    // Calculate current heading
    float currentHeading = calculateCurrentHeading();
    
    // Calculate heading error (difference between desired and current heading)
    float headingError = bearingToTarget - currentHeading;
    
    // Normalize heading error to -180 to +180 degrees
    while (headingError > 180.0f) headingError -= 360.0f;
    while (headingError < -180.0f) headingError += 360.0f;
    
    // Calculate distance to target
    // Simple approximation for small distances (Haversine would be more accurate)
    float latDiff = (targetLatitude - currentLat) * 111320.0f; // meters per degree latitude
    float lonDiff = (targetLongitude - currentLon) * 111320.0f * cosf(currentLat * M_PI / 180.0f);
    float distanceToTarget = sqrtf(latDiff * latDiff + lonDiff * lonDiff);
    
    // If very close to target, center servos
    if (distanceToTarget < 10.0f) { // Within 10 meters
        servo1Position = SERVO_CENTER;
        servo2Position = SERVO_CENTER;
    } else {
        // Calculate servo commands based on heading error
        // Servo 1: Controls left/right steering (yaw control)
        float steeringCommand = headingError * STEERING_GAIN;
        
        // Limit steering command to maximum deflection
        if (steeringCommand > MAX_SERVO_DEFLECTION) {
            steeringCommand = MAX_SERVO_DEFLECTION;
        }
        if (steeringCommand < -MAX_SERVO_DEFLECTION) {
            steeringCommand = -MAX_SERVO_DEFLECTION;
        }
        
        // Set servo positions
        servo1Position = SERVO_CENTER + steeringCommand;
        servo2Position = SERVO_CENTER; // Servo 2 can be used for pitch/roll control if needed
        
        // TODO: Add pitch/roll control for servo 2 if needed
        // Example: Use accelerometer data to maintain level flight
        // float pitchError = getAccelPitch();
        // servo2Position = SERVO_CENTER + pitchError * PITCH_GAIN;
    }
    
    // Clamp servo positions to valid range
    if (servo1Position < SERVO_MIN_ANGLE) servo1Position = SERVO_MIN_ANGLE;
    if (servo1Position > SERVO_MAX_ANGLE) servo1Position = SERVO_MAX_ANGLE;
    if (servo2Position < SERVO_MIN_ANGLE) servo2Position = SERVO_MIN_ANGLE;
    if (servo2Position > SERVO_MAX_ANGLE) servo2Position = SERVO_MAX_ANGLE;
    
    // Send commands to actual servos (standard RC, 0–180 degrees)
    servoParaglider1.write((int)servo1Position);
    servoParaglider2.write((int)servo2Position);
}

void updateServos() {
    // Update servo positions based on current flight state
    
    switch (flightState) {
        case PROBE_RELEASE:
            // Release probe if not already released
            releaseProbe();
            
            // Update paraglider control during descent
            // Limit update rate to avoid jitter
            uint32_t now = millis();
            if (now - lastParagliderUpdate >= (1000 / SERVO_UPDATE_RATE)) {
                updateParagliderControl();
                lastParagliderUpdate = now;
            }
            break;
            
        case PAYLOAD_RELEASE:
            releasePayload();
            // Center paraglider servos after payload release
            servo1Position = SERVO_CENTER;
            servo2Position = SERVO_CENTER;
            break;
            
        default:
            // No servo action needed
            break;
    }
}
