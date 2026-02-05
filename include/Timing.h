#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

// Initialize timing system
void initTiming();

// Mission time management (required: F2, F3)
// Mission time must be maintained through processor resets
// Time must be set by ground command to within 1 second UTC
bool timeSetComplete();  // Check if time synchronization is complete

// Get mission time in UTC format hh:mm:ss (required: G3)
// Returns true if time is valid, false otherwise
bool getMissionTime(uint8_t& hour, uint8_t& minute, uint8_t& second);

// Set mission time from UTC string "hh:mm:ss" or GPS (required: ST command)
bool setMissionTime(const char* timeStr);  // Format: "13:35:59" or "GPS"
bool setMissionTimeFromGPS();

// Get current time in milliseconds since startup
uint32_t getCurrentTimeMs();

// Save/restore mission time to/from persistent storage (required: F2)
void saveMissionTime();  // Save to EEPROM or flash
void restoreMissionTime();  // Restore from EEPROM or flash

// Update timing system (call periodically)
void updateTiming();

#endif // TIMING_H
