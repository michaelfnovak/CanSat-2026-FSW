#include "Sensors.h"
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

// Adafruit sensor libraries (BMP390, INA219, BNO055)
#include <Adafruit_BMP3XX.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

// Unit conversion
static constexpr float DEG_PER_RAD = 57.2958f;

// Sensor state variables
static float currentAltitude = 0.0f;
static float currentPressure = 101.325f;  // kPa, sea level default
static float currentTemperature = 20.0f;  // °C
static float currentVoltage = 0.0f;       // Battery voltage (V)
static float currentCurrent = 0.0f;       // Battery current (A)
static float currentGyroR = 0.0f, currentGyroP = 0.0f, currentGyroY = 0.0f;
static float currentAccelR = 0.0f, currentAccelP = 0.0f, currentAccelY = 0.0f;
static float altitudeOffset = 0.0f;
static float previousAltitude = 0.0f;
static uint32_t lastUpdateTime = 0;

// GPS data
static uint8_t gpsHour = 0, gpsMinute = 0, gpsSecond = 0;
static float gpsAltitude = 0.0f;
static float gpsLatitude = 0.0f;
static float gpsLongitude = 0.0f;
static uint8_t gpsSatellites = 0;
static float gpsCourseDeg = 0.0f;
static float gpsGroundSpeedMps = 0.0f;
static bool gpsCourseValid = false;
static bool gpsSpeedValid = false;

static float imuHeadingDeg = 0.0f;
static bool bnoInitialized = false;

// Simulation mode (required: F4-F6)
static bool simulationModeEnabled = false;
static bool simulationModeActive = false;
static float simulatedPressure = 101325.0f;  // Pascals

// EEPROM addresses for persistent configuration state
// Reserve addresses 30-33 for altitude offset (float) and 34 for calibration flag.
const int EEPROM_ALT_OFFSET_ADDR = 30;
const int EEPROM_ALT_CAL_FLAG_ADDR = 34;

// Hardware sensor objects
// BMP390 barometric sensor (I2C). Uses I2C address 0x77 (see BNOO55_and_BMP390_test.ino).
static Adafruit_BMP3XX bmp;
static bool bmpInitialized = false;

// INA219 current/voltage sensor (I2C). Default address 0x40.
static Adafruit_INA219 ina219;

// BNO055 IMU (I2C). Common address is 0x28; change to 0x29 if your board is configured that way.
static Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// GPS module (UART). GPS uses Serial1 at 4800 baud (see gps_tester.ino).
static HardwareSerial& GPS_SERIAL = Serial1;
static TinyGPSPlus gpsParser;

void initSensors() {
    // Initialize I2C bus for BMP390, INA219, BNO055
    Wire.begin();

    // Initialize BMP390 barometric/temperature sensor
    // Uses I2C address 0x77. Returns false if not found.
    if (bmp.begin_I2C(0x77)) {
        bmpInitialized = true;
        // Configure oversampling / filter exactly as in BNOO55_and_BMP390_test.ino
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    } else {
        bmpInitialized = false;
    }

    // Initialize INA219 current/voltage monitor
    if (!ina219.begin()) {
        // Leave voltage/current at 0 on failure
    }

    // Initialize BNO055 IMU
    if (!bno.begin()) {
        bnoInitialized = false;
    } else {
        bnoInitialized = true;
        // Use external crystal for better accuracy if available
        bno.setExtCrystalUse(true);
    }

    // Initialize GPS UART (TinyGPSPlus parser in updateSensors)
    // BrainFPV GPS at 4800 baud to reduce UART frame errors (see gps_tester.ino).
    GPS_SERIAL.begin(4800);
    
    // Restore altitude calibration (zero-altitude offset) from EEPROM (F8)
    if (EEPROM.read(EEPROM_ALT_CAL_FLAG_ADDR) == 1) {
        EEPROM.get(EEPROM_ALT_OFFSET_ADDR, altitudeOffset);
    } else {
        altitudeOffset = 0.0f;
    }

    previousAltitude = 0.0f;
    lastUpdateTime = millis();
}

float getAltitude() {
    return currentAltitude;
}

float getPressure() {
    // In simulation mode, return simulated pressure converted to kPa
    if (simulationModeActive) {
        return simulatedPressure / 1000.0f;  // Convert Pa to kPa
    }
    return currentPressure;
}

float getTemperature() {
    return currentTemperature;
}

float getBatteryVoltage() {
    return currentVoltage;
}

float getBatteryCurrent() {
    return currentCurrent;
}

float getGyroRoll() {
    return currentGyroR;
}

float getGyroPitch() {
    return currentGyroP;
}

float getGyroYaw() {
    return currentGyroY;
}

float getAccelRoll() {
    return currentAccelR;
}

float getAccelPitch() {
    return currentAccelP;
}

float getAccelYaw() {
    return currentAccelY;
}

bool getGPSTime(uint8_t& hour, uint8_t& minute, uint8_t& second) {
    hour = gpsHour;
    minute = gpsMinute;
    second = gpsSecond;
    return gpsSatellites > 0;  // Return true if GPS has fix
}

float getGPSAltitude() {
    return gpsAltitude;
}

float getGPSLatitude() {
    return gpsLatitude;
}

float getGPSLongitude() {
    return gpsLongitude;
}

uint8_t getGPSSatellites() {
    return gpsSatellites;
}

float getVerticalVelocity() {
    // Calculate vertical velocity from altitude change
    uint32_t now = millis();
    if (lastUpdateTime > 0 && now > lastUpdateTime) {
        float dt = (now - lastUpdateTime) / 1000.0f;  // Convert to seconds
        if (dt > 0) {
            return (currentAltitude - previousAltitude) / dt;
        }
    }
    return 0.0f;
}

static float wrapAngle360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

bool getHeadingReferenceDeg(float* outHeadingDeg, uint8_t* source) {
    if (!outHeadingDeg || !source) {
        return false;
    }
    *source = 0;

    // Primary: GPS course-over-ground when moving fast enough for stable COG (typ. RMC).
    // COG aligns steering with actual ground track toward the target (C8).
    const float MIN_GROUND_SPEED_MPS = 1.2f;
    if (!simulationModeActive && gpsCourseValid && gpsSpeedValid &&
        gpsGroundSpeedMps >= MIN_GROUND_SPEED_MPS &&
        getGPSSatellites() >= 4) {
        *outHeadingDeg = wrapAngle360(gpsCourseDeg);
        *source = 1;
        return true;
    }

    // Fallback: BNO055 fusion heading when COG is stale or ground speed is too low.
    // Not used in simulation (Euler not updated in that path). Mag/pendulum sensitive.
    if (!simulationModeActive && bnoInitialized) {
        *outHeadingDeg = wrapAngle360(imuHeadingDeg);
        *source = 2;
        return true;
    }

    return false;
}

void zeroAltitude() {
    // Calibrate altitude to zero at launch pad (required: G1, CAL command)
    // Store current altitude reading as offset and persist it (F8).
    altitudeOffset = currentAltitude;
    currentAltitude = 0.0f;
    previousAltitude = 0.0f;

    // Save altitudeOffset and a calibration flag to EEPROM so it survives resets.
    EEPROM.put(EEPROM_ALT_OFFSET_ADDR, altitudeOffset);
    EEPROM.write(EEPROM_ALT_CAL_FLAG_ADDR, 1);
}

void setSimulationMode(bool enabled) {
    simulationModeEnabled = enabled;
    simulationModeActive = enabled;  // Both required so getPressure() and updateSensors() use simulated values
}

bool isSimulationMode() {
    return simulationModeActive;
}

void setSimulatedPressure(float pressure_pa) {
    simulatedPressure = pressure_pa;
    const float P0 = 101325.0f;  // Sea level pressure in Pa
    currentAltitude = 44330.0f * (1.0f - powf(pressure_pa / P0, 0.1903f)); // barometric formula
    currentPressure = pressure_pa / 1000.0f;  // Convert to kPa
}

void updateSensors() {
    // Read sensor data and update all sensor values.
    // This should be called periodically from main loop.
    
    uint32_t now = millis();
    
    if (!simulationModeActive) {
        // --- BMP390: temperature, pressure, altitude ---
        if (bmpInitialized && bmp.performReading()) {
            currentTemperature = bmp.temperature;            // °C
            // bmp.pressure is in Pascals
            float pressurePa = bmp.pressure;
            currentPressure = pressurePa / 1000.0f;         // kPa

            // Calculate altitude from pressure (barometric formula)
            // altitude = 44330 * (1 - (P/P0)^0.1903)
            const float P0 = 101325.0f;  // Sea level pressure in Pa
            float calculatedAltitude = 44330.0f * (1.0f - powf(pressurePa / P0, 0.1903f));
            currentAltitude = calculatedAltitude - altitudeOffset;
        }

        // --- INA219: battery voltage and current ---
        // Bus voltage in volts and current in milliamps
        float busVoltage_V = ina219.getBusVoltage_V();
        float current_mA = ina219.getCurrent_mA();
        currentVoltage = busVoltage_V;
        currentCurrent = current_mA / 1000.0f;  // Convert mA to A

        // --- BNO055: gyro, accelerometer, Euler heading (for descent steering fallback) ---
        sensors_event_t accelEvent, gyroEvent, orientEvent;

        if (bnoInitialized) {
            // Linear acceleration (m/s^2), gravity removed. Matches BNOO55_and_BMP390_test.ino.
            bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
            currentAccelR = accelEvent.acceleration.x;
            currentAccelP = accelEvent.acceleration.y;
            currentAccelY = accelEvent.acceleration.z;

            // Gyroscope (angular velocity). Library reports rad/s -> convert to deg/s for telemetry.
            bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
            currentGyroR = gyroEvent.gyro.x * DEG_PER_RAD;
            currentGyroP = gyroEvent.gyro.y * DEG_PER_RAD;
            currentGyroY = gyroEvent.gyro.z * DEG_PER_RAD;

            // Heading (degrees, 0–360). Axis mapping depends on PCB mount; tune sign in field.
            bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER);
            imuHeadingDeg = orientEvent.orientation.x;
        }

        // --- GPS: parse NMEA sentences from GPS_SERIAL using TinyGPSPlus ---
        while (GPS_SERIAL.available() > 0) {
            char c = (char)GPS_SERIAL.read();
            gpsParser.encode(c);
        }

        if (gpsParser.location.isValid()) {
            gpsLatitude  = gpsParser.location.lat();
            gpsLongitude = gpsParser.location.lng();
        }

        if (gpsParser.altitude.isValid()) {
            gpsAltitude = gpsParser.altitude.meters();
        }

        if (gpsParser.satellites.isValid()) {
            gpsSatellites = (uint8_t)gpsParser.satellites.value();
        }

        if (gpsParser.time.isValid()) {
            gpsHour   = (uint8_t)gpsParser.time.hour();
            gpsMinute = (uint8_t)gpsParser.time.minute();
            gpsSecond = (uint8_t)gpsParser.time.second();
        }

        gpsCourseValid = gpsParser.course.isValid();
        if (gpsCourseValid) {
            gpsCourseDeg = (float)gpsParser.course.deg();
        }
        gpsSpeedValid = gpsParser.speed.isValid();
        if (gpsSpeedValid) {
            gpsGroundSpeedMps = (float)gpsParser.speed.mps();
        }
    }
    
    // Update vertical velocity calculation
    if (lastUpdateTime > 0) {
        previousAltitude = currentAltitude;
    }
    lastUpdateTime = now;
}
