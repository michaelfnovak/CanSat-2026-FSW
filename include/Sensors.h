#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

// Initialize all sensors
void initSensors();

// Altitude and pressure (required: SN1)
float getAltitude();  // Altitude in meters relative to ground level, resolution 0.1m
float getPressure();  // Air pressure in kPa, resolution 0.1 kPa

// Temperature (required: SN2)
float getTemperature();  // Internal temperature in degrees Celsius, resolution 0.1°C

// Battery monitoring (required: SN3, SN10)
float getBatteryVoltage();  // Voltage in volts, resolution 0.1V
float getBatteryCurrent();  // Current in amperes, resolution 0.01A

// Inertial measurement (required: SN5)
// Gyro readings in degrees per second
float getGyroRoll();   // GYRO_R
float getGyroPitch();  // GYRO_P
float getGyroYaw();    // GYRO_Y

// Accelerometer readings in degrees per second squared (or m/s²)
float getAccelRoll();   // ACCEL_R
float getAccelPitch();  // ACCEL_P
float getAccelYaw();    // ACCEL_Y

// GPS (required: SN4)
bool getGPSTime(uint8_t& hour, uint8_t& minute, uint8_t& second);  // UTC time, 1s resolution
float getGPSAltitude();  // Altitude in meters above MSL, resolution 0.1m
float getGPSLatitude();  // Latitude in decimal degrees, resolution 0.0001°N
float getGPSLongitude(); // Longitude in decimal degrees, resolution 0.0001°W
uint8_t getGPSSatellites();  // Number of GPS satellites tracked

// Derived values
float getVerticalVelocity();  // Vertical velocity in m/s

// Calibration (required: G1)
void zeroAltitude();  // Calibrate altitude to zero at launch pad

// Simulation mode (required: F4-F6)
void setSimulationMode(bool enabled);
bool isSimulationMode();
void setSimulatedPressure(float pressure_pa);  // Pressure in Pascals

// Update sensor readings (call periodically)
void updateSensors();

#endif // SENSORS_H
