#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

const int pulsePin = 1; // Pin you are pulsing
const int pulseDelay = 500; // milliseconds

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(pulsePin, OUTPUT);

  Serial.println("INA219 Current Measurement Test");

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  Serial.println("INA219 Initialized");
}

void loop() {
  // Pulse Pin HIGH
  digitalWrite(pulsePin, HIGH);
  delay(pulseDelay);

  // Read current through resistor
  float current_mA = ina219.getCurrent_mA();

  Serial.print("Pin HIGH Current: ");
  Serial.print(current_mA);
  Serial.println(" mA");

  // Pulse Pin LOW
  digitalWrite(pulsePin, LOW);
  delay(pulseDelay);

  // Read current again (should be near 0)
  current_mA = ina219.getCurrent_mA();
  Serial.print("Pin LOW Current: ");
  Serial.print(current_mA);
  Serial.println(" mA");

  Serial.println("---------------------------");
}