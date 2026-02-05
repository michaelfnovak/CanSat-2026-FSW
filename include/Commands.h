#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

// Initialize command system
void initCommands();

// Set team ID (required for command parsing)
void setTeamID(uint16_t teamID);
uint16_t getTeamID();

// Command processing (required: Section 3.1.2)
// CX - Telemetry On/Off: CMD,<TEAM_ID>,CX,<ON_OFF>
bool processCXCommand(const char* onOff);

// ST - Set Time: CMD,<TEAM_ID>,ST,<UTC_TIME>|GPS
bool processSTCommand(const char* timeStr);

// SIM - Simulation Mode: CMD,<TEAM_ID>,SIM,ENABLE|ACTIVATE|DISABLE
bool processSIMCommand(const char* action);

// SIMP - Simulated Pressure: CMD,<TEAM_ID>,SIMP,<pressure_pascals>
bool processSIMPCommand(uint32_t pressurePa);

// CAL - Calibrate Altitude: CMD,<TEAM_ID>,CAL
bool processCALCommand();

// MEC - Mechanism: CMD,<TEAM_ID>,MEC,<DEVICE>,<ON_OFF>
bool processMECCommand(const char* device, const char* onOff);

// Parse and process command string
// Format: CMD,<TEAM_ID>,<COMMAND>,<PARAMS>
bool parseCommand(const char* cmdString);

// Process incoming commands
void processCommands();

// Update command system (call periodically)
void updateCommands();

#endif // COMMANDS_H
