# CanSat 2026 FSW – Remaining Work

Areas where implementation is still needed (TODOs, placeholders, and incomplete logic).

---

## 1. **main.cpp**

| Line | Area | What to do |
|------|------|------------|
| 18 | Team ID | Confirm `TEAM_ID = 1057` is your assigned team number. |
| 42–44 | Target location | Set `setTargetLocation(lat, lon)` to your actual target landing coordinates (or load from config/command). |

---

## 2. **Sensors** (`include/Sensors.h`, `src/sensors/Sensors.cpp`)

| Area | What to do |
|------|------------|
| **initSensors()** ~L30 | Initialize barometer, temperature sensor, battery monitor, IMU (gyro/accel), GPS (I2C/SPI/Serial). |
| **zeroAltitude()** ~L127 | Implement CAL behavior: store current pressure/altitude as offset so reported altitude is 0 m at launch pad (required: G1). |
| **setSimulatedPressure()** ~L144 | Use barometric formula so altitude is computed from simulated pressure (for sim mode). |
| **updateSensors()** ~L153 | Read all hardware: pressure → altitude, temperature, voltage, current, gyro R/P/Y, accel R/P/Y, GPS (time, lat, lon, alt, sats). Update `current*` and GPS globals. |
| **Actual sensor reads** ~L159 | Replace placeholder with real reads for barometer, temp, battery, IMU, GPS. |

**Required hardware (from rules):** Barometer (SN1), temp (SN2), battery V/I (SN3, SN10), GPS (SN4), IMU – accel + gyro (SN5).

---

## 3. **Timing** (`include/Timing.h`, `src/utils/Timing.cpp`)

| Area | What to do |
|------|------------|
| **initTiming()** ~L19 | Any RTC or time-source setup; `restoreMissionTime()` is already called. |
| **setMissionTimeFromGPS()** ~L89 | Implement ST,GPS: read UTC time from GPS and call mission-time setter (required: F3). |
| **updateTiming()** ~L146 | Optional: periodic GPS time sync, RTC sync, or health checks. |

Mission time save/restore to EEPROM is implemented for Teensy 4.1.

---

## 4. **Telemetry** (`include/telemetry.h`, `src/telemetry/telemetry.cpp`)

| Area | What to do |
|------|------------|
| **updateTelemetry()** ~L163 | Implement connection checks (e.g. XBee link status, time since last ACK) and update any “telemetry active” flags used for PRELAUNCH/LAUNCH_READY. |

Packet count and mission-time persistence are implemented.

---

## 5. **XBee / Comms** (`include/XBee.h`, `src/comms/XBee.cpp`)

| Area | What to do |
|------|------------|
| **initXBee()** ~L9 | Configure Serial (or Serial1/2) for XBee, set baud, optionally configure NETID/PANID per team (X2). |
| **xbeeSend()** ~L19 | Send `data[0..length-1]` over the XBee serial port (e.g. `Serial1.write(data, length)`). |
| **xbeeReceive()** ~L24 | Non-blocking read: if bytes available, fill `buffer`, set `*length`, return true; else return false. |
| **updateXBee()** ~L31 | Replace “mark ready after 1 s” placeholder with real link/status handling if needed. |

Rules: XBee for telemetry (X1), 1 Hz transmit (X4), no broadcast (X3).

---

## 6. **Servos** (`include/servos.h`, `src/servos/servos.cpp`)

| Area | What to do |
|------|------------|
| **Pin constants** ~L7 | Set `SERVO_PROBE_PIN`, `SERVO_PAYLOAD_PIN`, `SERVO_PARAGLIDER_1_PIN`, `SERVO_PARAGLIDER_2_PIN` to your Teensy 4.1 pins. |
| **initServos()** ~L35 | Initialize PWM/servo library, set pins as output, move to safe/center positions. |
| **resetServos()** ~L61 | Command probe, payload, and paraglider servos to initial/center positions. |
| **releaseProbe()** ~L72 | Command probe-release servo to release at 80% altitude. |
| **releasePayload()** ~L82 | Command payload (egg) release servo at ~2 m altitude. |
| **calculateCurrentHeading()** ~L123 | Return current heading (e.g. from GPS course or IMU yaw); replace “return 0” placeholder. |
| **angleToPWM()** ~L150, L157 | Implement conversion from angle (0–180°) to your PWM (pulse width or analogWrite range) for your servos. |
| **Pitch/roll (optional)** ~L224 | Optionally use accel/IMU to drive servo 2 for pitch/roll during paraglider. |
| **PWM output** ~L236 | In `updateParagliderControl()`, call your PWM/servo API with `servo1Position` and `servo2Position` (e.g. via `angleToPWM()`). |

Paraglider logic (bearing to target, heading error, 10 Hz updates) is in place; it only needs real heading and PWM/servo output.

---

## 7. **Commands** (`include/Commands.h`, `src/commands/Commands.cpp`)

| Area | What to do |
|------|------------|
| **initCommands()** ~L14 | Set up command buffer, parser state, or RX queue if used. |
| **processMECCommand() device mapping** ~L123 | Extend the `device` string handling (e.g. "PROBE", "PAYLOAD", "PARAGLIDER1", "PARAGLIDER2") and call the right servo/mechanism functions (F7). |
| **processCommands() / updateCommands()** ~L223 | Replace “read from XBee” placeholder: get bytes from XBee (e.g. via `xbeeReceive` or a serial buffer), assemble full lines, then call `parseCommand(cmdString)`. |

CX, ST, SIM, SIMP, CAL parsing and behavior are implemented; MEC and XBee ingestion need wiring to your hardware.

---

## 8. **Summary by priority**

**Critical (needed for flight):**
- Sensors: all hardware init and reads (pressure, temp, voltage, current, IMU, GPS).
- XBee: init, send, receive, and integration with command processing.
- Servos: pin assignment, PWM/servo init, probe/payload release, paraglider PWM output and (if used) heading source.

**Important (rules / robustness):**
- Sensors: `zeroAltitude()` for CAL; simulated pressure → altitude for sim mode.
- Timing: `setMissionTimeFromGPS()` for ST,GPS.
- Commands: XBee RX → `parseCommand()`; MEC device mapping to mechanisms.

**Optional / tuning:**
- Paraglider: real heading (GPS/IMU), pitch/roll on servo 2, gains (STEERING_GAIN, MAX_SERVO_DEFLECTION).
- Telemetry: link status in `updateTelemetry()`.
- Timing: optional periodic time sync in `updateTiming()`.

---

*Generated from codebase TODOs and placeholders. Update this file as you complete items.*
