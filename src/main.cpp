#include <Arduino.h>
#include "FlightState.h"
#include "StateLogic.h"
#include "Sensors.h"
#include "Timing.h"
#include "telemetry.h"
#include "XBee.h"
#include "servos.h"
#include "Commands.h"

// Main loop timing
const uint32_t MAIN_LOOP_PERIOD_MS = 10;  // 100 Hz main loop for responsiveness
const uint32_t TELEMETRY_PERIOD_MS = 1000;  // 1 Hz telemetry (required: X4, C9)
uint32_t lastLoopTime = 0;
uint32_t lastTelemetryTime = 0;

// Team ID - MUST BE SET TO YOUR TEAM NUMBER
const uint16_t TEAM_ID = 0;  // TODO: Set your team ID

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    delay(1000);
    
    // Set team ID for command processing
    setTeamID(1057);
    
    // Initialize all subsystems
    initTiming();
    initSensors();
    initXBee();
    initTelemetry();
    initServos();
    initCommands();
    
    // Restore mission time from persistent storage (required: F2)
    // Note: restoreMissionTime() is called in initTiming()
    
    // Initialize flight state
    flightState = PRELAUNCH;
    
    Serial.println("CanSat Flight Software Initialized");
    Serial.print("Team ID: ");
    Serial.println(TEAM_ID);
}

void loop() {
    uint32_t now_ms = millis();
    
    // High frequency main loop for responsiveness
    if (now_ms - lastLoopTime >= MAIN_LOOP_PERIOD_MS) {
        lastLoopTime = now_ms;
        
        // Update all subsystems
        updateTiming();
        updateSensors();
        updateXBee();
        updateCommands();  // Process commands frequently
        
        // Update flight state based on sensor readings
        updateFlightState(now_ms);
        
        // Update servos based on current flight state
        updateServos();
    }
    
    // Send telemetry at exactly 1 Hz (required: X4, C9)
    if (now_ms - lastTelemetryTime >= TELEMETRY_PERIOD_MS) {
        lastTelemetryTime = now_ms;
        
        // Update telemetry system
        updateTelemetry();
        
        // Send telemetry packet if enabled
        if (isTelemetryEnabled()) {
            sendTelemetry();
        }
    }
}
