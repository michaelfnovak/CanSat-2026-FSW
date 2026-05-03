#include "Commands.h"
#include "telemetry.h"
#include "Timing.h"
#include "Sensors.h"
#include "servos.h"
#include "XBee.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <EEPROM.h>

static uint16_t teamID = 0;
static bool simulationEnabled = false;
static bool simulationActive = false;

// EEPROM addresses for simulation mode configuration state
// Reserve addresses 50-51 for simulationEnabled and simulationActive flags.
const int EEPROM_SIM_ENABLED_ADDR = 50;
const int EEPROM_SIM_ACTIVE_ADDR  = 51;

void initCommands() {
    // Command buffer and line assembly are in XBee (xbeeReceive returns one line per call).
    // Parser state is stateless: parseCommand() handles each line.
    // Team ID is set by main via setTeamID(). Do not clear it here; setup currently
    // calls setTeamID() before initCommands().

    // Restore simulation configuration state (F8) so resets in simulation mode are handled.
    bool storedSimEnabled = (EEPROM.read(EEPROM_SIM_ENABLED_ADDR) == 1);
    bool storedSimActive  = (EEPROM.read(EEPROM_SIM_ACTIVE_ADDR)  == 1);

    if (storedSimEnabled) {
        // Reflect stored configuration in local flags
        // Note: we still require SIM,ENABLE before SIM,ACTIVATE in normal operation;
        // this simply re-applies the last configuration after a reset.
        simulationEnabled = storedSimEnabled;
        simulationActive  = storedSimActive;

        if (simulationActive) {
            // Re-enter simulation mode on reset
            setSimulationMode(true);
            setTelemetryMode(MODE_SIMULATION);
        } else {
            setSimulationMode(false);
            setTelemetryMode(MODE_FLIGHT);
        }
    }
}

void setTeamID(uint16_t id) {
    teamID = id;
}

uint16_t getTeamID() {
    return teamID;
}

bool processCXCommand(const char* onOff) {
    // CX - Telemetry On/Off: CMD,<TEAM_ID>,CX,<ON_OFF>
    if (onOff == nullptr) {
        return false;
    }
    
    if (strcmp(onOff, "ON") == 0) {
        setTelemetryEnabled(true);
        setCommandEcho("CXON");
        return true;
    } else if (strcmp(onOff, "OFF") == 0) {
        setTelemetryEnabled(false);
        setCommandEcho("CXOFF");
        return true;
    }
    
    return false;
}

bool processSTCommand(const char* timeStr) {
    // ST - Set Time: CMD,<TEAM_ID>,ST,<UTC_TIME>|GPS
    if (timeStr == nullptr) {
        return false;
    }
        
    if (setMissionTime(timeStr)) {
        char echo[32];
        snprintf(echo, sizeof(echo), "ST%s", timeStr);
        setCommandEcho(echo);
        return true;
    }
    
    return false;
}

bool processSIMCommand(const char* action) {
    // SIM - Simulation Mode: CMD,<TEAM_ID>,SIM,ENABLE|ACTIVATE|DISABLE
    if (action == nullptr) {
        return false;
    }
    
    if (strcmp(action, "ENABLE") == 0) {
        simulationEnabled = true;
        EEPROM.write(EEPROM_SIM_ENABLED_ADDR, 1);
        setCommandEcho("SIMENABLE");
        return true;
    } else if (strcmp(action, "ACTIVATE") == 0) {
        if (simulationEnabled) {
            simulationActive = true;
            setSimulationMode(true);
            setTelemetryMode(MODE_SIMULATION);
            EEPROM.write(EEPROM_SIM_ACTIVE_ADDR, 1);
            setCommandEcho("SIMACTIVATE");
            return true;
        }
        return false;
    } else if (strcmp(action, "DISABLE") == 0) {
        simulationEnabled = false;
        simulationActive = false;
        setSimulationMode(false);
        setTelemetryMode(MODE_FLIGHT);
        EEPROM.write(EEPROM_SIM_ENABLED_ADDR, 0);
        EEPROM.write(EEPROM_SIM_ACTIVE_ADDR, 0);
        setCommandEcho("SIMDISABLE");
        return true;
    }
    
    return false;
}

bool processSIMPCommand(uint32_t pressurePa) {
    // SIMP - Simulated Pressure: CMD,<TEAM_ID>,SIMP,<pressure_pascals>
    if (!simulationActive) {
        return false;  // Can only be used in simulation mode
    }
    
    setSimulatedPressure(pressurePa);
    char echo[32];
    snprintf(echo, sizeof(echo), "SIMP%lu", pressurePa);
    setCommandEcho(echo);
    return true;
}

bool processCALCommand() {
    // CAL - Calibrate Altitude: CMD,<TEAM_ID>,CAL
    zeroAltitude();
    resetPacketCount();  // Also reset packet count per requirements
    setCommandEcho("CAL");
    return true;
}

bool processMECCommand(const char* device, const char* onOff) {
    // MEC - Mechanism: CMD,<TEAM_ID>,MEC,<DEVICE>,<ON_OFF>
    if (device == nullptr || onOff == nullptr) {
        return false;
    }
    
    bool activate = (strcmp(onOff, "ON") == 0);
    
    if (strcmp(device, "PAYLOAD") == 0) {
        if (activate) {
            // PAYLOAD = canister separation hatch (servo on pin 2).
            // Nudge only (10°) — confirms servo is alive without straining the battery.
            // Full 90° release is triggered by the state machine at PROBE_RELEASE, not here.
            nudgeProbe();
        }
    } else if (strcmp(device, "EGG") == 0) {
        if (activate) {
            // EGG = egg drop mechanism (servo on pin 5).
            // Nudge only (10°) — same battery-safety rationale as PAYLOAD above.
            // Full 90° release is triggered by the state machine at PAYLOAD_RELEASE.
            nudgePayload();
        }
    } else if (strcmp(device, "FS1") == 0) {
        // Flight surface 1 test: ON -> 90 deg, OFF -> 0 deg
        setFlightSurface1Test(activate);
    } else if (strcmp(device, "FS2") == 0) {
        // Flight surface 2 test: ON -> 90 deg, OFF -> 0 deg
        setFlightSurface2Test(activate);
    }
    // Add more device mappings as needed
    
    char echo[64];
    snprintf(echo, sizeof(echo), "MEC%s%s", device, onOff);
    setCommandEcho(echo);
    return true;
}

bool parseCommand(const char* cmdString) {
    // Parse command string: CMD,<TEAM_ID>,<COMMAND>,<PARAMS>
    if (cmdString == nullptr) {
        return false;
    }
    
    // Check for "CMD," prefix
    if (strncmp(cmdString, "CMD,", 4) != 0) {
        return false;
    }
    
    // Parse team ID
    const char* ptr = cmdString + 4;
    char* endPtr;
    uint16_t cmdTeamID = (uint16_t)strtoul(ptr, &endPtr, 10);
    
    // Verify team ID matches
    if (cmdTeamID != teamID) {
        return false;
    }
    
    // Find command type
    ptr = endPtr + 1;  // Skip comma after team ID
    if (*ptr == '\0') {
        return false;
    }

    // Find the comma that separates the command token from its parameters.
    // If no comma exists the remainder IS the command with no parameters
    // (e.g. "CMD,1057,CAL" is valid; trailing comma is not required).
    const char* cmdStart = ptr;
    const char* cmdEnd = strchr(ptr, ',');

    char cmd[8];
    const char* params = "";
    if (cmdEnd == nullptr) {
        // No trailing comma — whole remainder is the command token, no params.
        size_t cmdLen = strlen(cmdStart);
        // Strip any trailing \r\n left by the line reader
        while (cmdLen > 0 && (cmdStart[cmdLen-1] == '\r' || cmdStart[cmdLen-1] == '\n')) {
            cmdLen--;
        }
        if (cmdLen == 0 || cmdLen >= sizeof(cmd)) return false;
        strncpy(cmd, cmdStart, cmdLen);
        cmd[cmdLen] = '\0';
    } else {
        // Normal case: comma found, params follow.
        size_t cmdLen = cmdEnd - cmdStart;
        if (cmdLen >= sizeof(cmd)) cmdLen = sizeof(cmd) - 1;
        strncpy(cmd, cmdStart, cmdLen);
        cmd[cmdLen] = '\0';
        params = cmdEnd + 1;
    }
    
    // Process command
    if (strcmp(cmd, "CX") == 0) {
        return processCXCommand(params);
    } else if (strcmp(cmd, "ST") == 0) {
        return processSTCommand(params);
    } else if (strcmp(cmd, "SIM") == 0) {
        return processSIMCommand(params);
    } else if (strcmp(cmd, "SIMP") == 0) {
        // Pressure in Pascals (e.g. 101325); strtoul stops at non-digit (\r, \n, etc.)
        uint32_t pressurePa = strtoul(params, nullptr, 10);
        return processSIMPCommand(pressurePa);
    } else if (strcmp(cmd, "CAL") == 0) {
        return processCALCommand();
    } else if (strcmp(cmd, "MEC") == 0) {
        // MEC has two parameters: device and on/off
        const char* deviceEnd = strchr(params, ',');
        if (deviceEnd == nullptr) {
            return false;
        }
        char device[32];
        size_t deviceLen = deviceEnd - params;
        if (deviceLen >= sizeof(device)) {
            deviceLen = sizeof(device) - 1;
        }
        strncpy(device, params, deviceLen);
        device[deviceLen] = '\0';
        const char* onOff = deviceEnd + 1;
        return processMECCommand(device, onOff);
    }
    
    return false;
}

void processCommands() {
    // Read one complete command line from XBee (if available) and parse it.
    uint8_t buffer[256];
    size_t length = 256;
    
    if (xbeeReceive(buffer, &length)) {
        buffer[length] = '\0';  // Null terminate
        parseCommand((const char*)buffer);
    }
}

void updateCommands() {
    processCommands();
}
