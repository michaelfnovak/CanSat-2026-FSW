#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>

/* ---------- Sensor Objects ---------- */
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/* ---------- Timing ---------- */
const uint32_t LOOP_PERIOD_MS = 1000;
uint32_t lastLoopTime = 0;

/* ---------- Sea Level Pressure ---------- */
const float SEA_LEVEL_PRESSURE_PA = 101325.0;

void setup() {
    Serial.begin(115200);
    delay(500); // allow Serial to stabilize

    Wire.begin();

    Serial.println("=== Teensy 4.1 Sensor Test ===");

    /* ---------- BMP390 Init ---------- */
    if (!bmp.begin_I2C(0x77)) { // BMP I2C address
        Serial.println("❌ BMP390 not detected");
        while (1);
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("✅ BMP390 initialized");

    /* ---------- BNO055 Init ---------- */
    if (!bno.begin()) {
        Serial.println("❌ BNO055 not detected");
        while (1);
    }

    // Use external crystal to stabilize gyro
    bno.setExtCrystalUse(true);

    // Set operation mode to NDOF for gyro + accel + mag fusion
    bno.setMode(OPERATION_MODE_NDOF);

    delay(1000); // let the sensor fully start
    Serial.println("✅ BNO055 initialized");
    Serial.println();
}

void loop() {
    uint32_t now = millis();
    if (now - lastLoopTime < LOOP_PERIOD_MS) return;
    lastLoopTime = now;

    /* ---------- BMP390 Read ---------- */
    if (bmp.performReading()) {
        float pressure_pa = bmp.pressure;
        float temperature_c = bmp.temperature;
        float altitude_m = 44330.0 * (1.0 - pow(pressure_pa / SEA_LEVEL_PRESSURE_PA, 0.1903));

        Serial.print("Pressure: ");
        Serial.print(pressure_pa, 1);
        Serial.print(" Pa | Temp: ");
        Serial.print(temperature_c, 1);
        Serial.print(" °C | Altitude: ");
        Serial.print(altitude_m, 2);
        Serial.println(" m");
    } else {
        Serial.println("❌ BMP390 read failed");
    }

    /* ---------- BNO055 Read ---------- */
    sensors_event_t accelEvent, gyroEvent;
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

    Serial.print("Accel (m/s^2) | X: ");
    Serial.print(accelEvent.acceleration.x, 2);
    Serial.print(" Y: ");
    Serial.print(accelEvent.acceleration.y, 2);
    Serial.print(" Z: ");
    Serial.println(accelEvent.acceleration.z, 2);

    Serial.print("Gyro (deg/s)  | X: ");
    Serial.print(gyroEvent.gyro.x * 57.2958, 2);
    Serial.print(" Y: ");
    Serial.print(gyroEvent.gyro.y * 57.2958, 2);
    Serial.print(" Z: ");
    Serial.println(gyroEvent.gyro.z * 57.2958, 2);

    Serial.println("--------------------------------");
}
