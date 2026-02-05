#include "Commands.h"
#include "telemetry.h"
#include "Timing.h"
#include "Sensors.h"
#include "servos.h"
#include "XBee.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

static uint16_t teamID = 0;

void initCommands() {
    // TODO: Initialize command processing system
    // Set up command buffer, parser, etc.
    teamID = 1057; 
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

static bool simulationEnabled = false;
static bool simulationActive = false;

bool processSIMCommand(const char* action) {
    // SIM - Simulation Mode: CMD,<TEAM_ID>,SIM,ENABLE|ACTIVATE|DISABLE
    if (action == nullptr) {
        return false;
    }
    
    if (strcmp(action, "ENABLE") == 0) {
        simulationEnabled = true;
        setCommandEcho("SIMENABLE");
        return true;
    } else if (strcmp(action, "ACTIVATE") == 0) {
        if (simulationEnabled) {
            simulationActive = true;
            setSimulationMode(true);
            setTelemetryMode(MODE_SIMULATION);
            setCommandEcho("SIMACTIVATE");
            return true;
        }
        return false;
    } else if (strcmp(action, "DISABLE") == 0) {
        simulationEnabled = false;
        simulationActive = false;
        setSimulationMode(false);
        setTelemetryMode(MODE_FLIGHT);
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
    
    // TODO: Map device string to actual mechanism
    // Examples: "PROBE", "PAYLOAD", "PARAGLIDER", etc.
    bool activate = (strcmp(onOff, "ON") == 0);
    
    if (strcmp(device, "PROBE") == 0) {
        if (activate) {
            releaseProbe();
        }
    } else if (strcmp(device, "PAYLOAD") == 0) {
        if (activate) {
            releasePayload();
        }
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
    
    // Find comma after command
    const char* cmdStart = ptr;
    const char* cmdEnd = strchr(ptr, ',');
    if (cmdEnd == nullptr) {
        return false;
    }
    
    // Extract command (CX, ST, SIM, SIMP, CAL, MEC)
    char cmd[8];
    size_t cmdLen = cmdEnd - cmdStart;
    if (cmdLen >= sizeof(cmd)) {
        cmdLen = sizeof(cmd) - 1;
    }
    strncpy(cmd, cmdStart, cmdLen);
    cmd[cmdLen] = '\0';
    
    // Get parameters
    const char* params = cmdEnd + 1;
    
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
    // TODO: Read from XBee and process commands
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
