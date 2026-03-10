#include <TinyGPS++.h>

/* ---------- GPS setup ---------- */
TinyGPSPlus gps;       // GPS parser
// Serial1 pins on Teensy 4.1: RX1 = pin 0, TX1 = pin 1

void setup() {
  Serial.begin(115200);    // Debug output to USB
  Serial1.begin(4800);     // BrainFPV GPS at 4800 baud to reduce UART frame errors
  Serial.println("=== GPS Test (4800 baud) ===");
}

void loop() {
  // Process any GPS characters
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    // Feed the character to TinyGPS++
    gps.encode(c);

    // Also print raw NMEA line for debugging
    Serial.write(c);
  }

  // Print interpreted GPS data every second
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    if (gps.location.isValid()) {
      Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
      Serial.print(" | Lon: "); Serial.print(gps.location.lng(), 6);
      Serial.print(" | Alt: "); Serial.println(gps.altitude.meters());
    } else {
      Serial.println("No valid GPS fix yet.");
    }

    if (gps.satellites.isUpdated()) {
      Serial.print("Satellites: "); Serial.println(gps.satellites.value());
    }
  }
}
