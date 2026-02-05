#include "Timing.h"
#include <Arduino.h>
#include <EEPROM.h>  // Teensy 4.1 EEPROM library (emulated in flash)
#include <math.h>

// Mission time storage
static uint8_t missionHour = 0;
static uint8_t missionMinute = 0;
static uint8_t missionSecond = 0;
static bool timeSet = false;
static uint32_t missionTimeOffset = 0;  // Offset from system time

// EEPROM addresses for persistent storage (Teensy 4.1 has 8KB EEPROM emulation)
// Reserve addresses 0-15 for mission time data
const int EEPROM_MISSION_TIME_ADDR = 0;
const int EEPROM_TIME_SET_FLAG_ADDR = 10;

void initTiming() {
    // Restore mission time from persistent storage (required: F2 - maintain through resets)
    restoreMissionTime();
    // Mission time is set by ground command (ST) to within 1 second UTC (F3).
    // No RTC hardware required; time is derived from millis() + stored offset.
}

bool timeSetComplete() {
    return timeSet;
}

bool getMissionTime(uint8_t& hour, uint8_t& minute, uint8_t& second) {
    if (!timeSet) {
        return false;
    }
    
    // Calculate current mission time from offset
    uint32_t now = millis();
    uint32_t totalSeconds = missionTimeOffset + (now / 1000);
    
    uint32_t seconds = totalSeconds % 60;
    uint32_t minutes = (totalSeconds / 60) % 60;
    uint32_t hours = (totalSeconds / 3600) % 24;
    
    hour = (uint8_t)hours;
    minute = (uint8_t)minutes;
    second = (uint8_t)seconds;
    
    return true;
}

bool setMissionTime(const char* timeStr) {
    // Parse UTC time string "hh:mm:ss" (required: ST command)
    if (timeStr == nullptr) {
        return false;
    }
    
    // Check if it's "GPS" command
    if (strcmp(timeStr, "GPS") == 0) {
        return setMissionTimeFromGPS();
    }
    
    // Parse "hh:mm:ss" format
    uint8_t h, m, s;
    if (sscanf(timeStr, "%hhu:%hhu:%hhu", &h, &m, &s) == 3) {
        if (h < 24 && m < 60 && s < 60) {
            missionHour = h;
            missionMinute = m;
            missionSecond = s;
            
            // Calculate offset from current system time
            uint32_t now = millis();
            uint32_t currentSeconds = (now / 1000) % 86400;  // Seconds since midnight
            uint32_t missionSeconds = h * 3600 + m * 60 + s;
            
            // Handle wrap-around
            if (missionSeconds > currentSeconds) {
                missionTimeOffset = missionSeconds - currentSeconds;
            } else {
                missionTimeOffset = 86400 - (currentSeconds - missionSeconds);
            }
            
            timeSet = true;
            saveMissionTime();
            return true;
        }
    }
    
    return false;
}

bool setMissionTimeFromGPS() {
    // TODO: Get time from GPS module (required: ST GPS command)
    // This should read the current UTC time from GPS
    // For now, placeholder implementation
    return false;
}

uint32_t getCurrentTimeMs() {
    return millis();
}

void saveMissionTime() {
    // Save mission time to EEPROM (required: F2 - maintain through resets)
    // Teensy 4.1 EEPROM library automatically handles flash emulation
    // No need to call EEPROM.begin() on Teensy
    
    EEPROM.write(EEPROM_MISSION_TIME_ADDR, missionHour);
    EEPROM.write(EEPROM_MISSION_TIME_ADDR + 1, missionMinute);
    EEPROM.write(EEPROM_MISSION_TIME_ADDR + 2, missionSecond);
    EEPROM.write(EEPROM_TIME_SET_FLAG_ADDR, timeSet ? 1 : 0);
    
    // Note: EEPROM.commit() is not needed on Teensy - writes are immediate
    // However, writes are cached and may be delayed, so we can force a commit if needed
}

void restoreMissionTime() {
    // Restore mission time from EEPROM (required: F2)
    // Teensy 4.1 EEPROM library automatically handles flash emulation
    
    missionHour = EEPROM.read(EEPROM_MISSION_TIME_ADDR);
    missionMinute = EEPROM.read(EEPROM_MISSION_TIME_ADDR + 1);
    missionSecond = EEPROM.read(EEPROM_MISSION_TIME_ADDR + 2);
    timeSet = (EEPROM.read(EEPROM_TIME_SET_FLAG_ADDR) == 1);
    
    // Validate restored values
    if (missionHour >= 24 || missionMinute >= 60 || missionSecond >= 60) {
        // Invalid data, reset to defaults
        missionHour = 0;
        missionMinute = 0;
        missionSecond = 0;
        timeSet = false;
    }
    
    // Recalculate offset
    if (timeSet) {
        uint32_t now = millis();
        uint32_t currentSeconds = (now / 1000) % 86400;
        uint32_t missionSeconds = missionHour * 3600 + missionMinute * 60 + missionSecond;
        
        if (missionSeconds > currentSeconds) {
            missionTimeOffset = missionSeconds - currentSeconds;
        } else {
            missionTimeOffset = 86400 - (currentSeconds - missionSeconds);
        }
    }
}

void updateTiming() {
    // TODO: Update timing system
    // Check for GPS time updates, etc.
}
