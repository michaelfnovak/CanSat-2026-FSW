# CanSat 2026 FSW → GCS integration (authoritative)

This document is produced from the **flight software (FSW)** repository for the **Ground Control Station (GCS)** team. It describes what the Teensy firmware actually sends and accepts on the radio link. If anything here disagrees with an older GCS note, **this file wins**.

**Team ID:** `1057` (compiled into FSW; `getTeamID()` in telemetry and command parser).

> **⚠ Test-flight notice — hardware actuation disabled:** For this launch, all servo mechanisms and paraglider flight-control surfaces are **software-disabled** (`src/servos/servos.cpp` — see file header). This CanSat separates from the rocket but takes **no physical action** afterward: no probe release, no egg release, no flight-surface deflection, no autonomous glide guidance. Only sensors and telemetry are being tested. The radio protocol below (telemetry fields, commands, `STATE` values) is **unchanged** — the GCS should not need any changes — but see §3.2 and §4 for the behavioral impact.

---

## 1. Physical link

| Item | FSW behavior |
|------|----------------|
| **Path** | Teensy 4.1 ↔ UART ↔ XBee (transparent serial). GCS typically uses USB ↔ XBee on the ground. |
| **UART** | XBee uses **`Serial5`** (RX5/TX5) on the Teensy at **`9600` baud** (`XBEE_BAUD` in `src/comms/XBee.cpp`). |
| **GCS baud** | Must match **9600** unless you reflash/reconfigure both ends to the same rate. |
| **Line ending** | **Inbound (GCS → FSW):** lines are assembled on `\r` or `\n` (`XBee.cpp`); typical use is **`CRLF` (`\r\n`)**. **Outbound (FSW → GCS):** each telemetry line ends with **`\r\n`**. |

---

## 2. Telemetry (FSW → GCS)

### 2.1 When packets are sent

- Telemetry is sent **only if** `CX,ON` has been processed (`setTelemetryEnabled(true)`).
- **Rate:** **1 Hz** (`TELEMETRY_PERIOD_MS = 1000` in `main.cpp`).
- `telemetryActive()` (used for PRELAUNCH → LAUNCH_PAD) requires **telemetry enabled** and **XBee ready** after init.

### 2.2 Framing

- One packet = **one ASCII line**, fields separated by **commas**, terminated by **`\r\n`**.
- **`snprintf` format** (exact field order) is in `src/telemetry/telemetry.cpp` `sendTelemetry()`:

`TEAM_ID, MISSION_TIME, PACKET_COUNT, MODE, STATE, ALTITUDE, TEMPERATURE, PRESSURE, VOLTAGE, CURRENT, GYRO_R, GYRO_P, GYRO_Y, ACCEL_R, ACCEL_P, ACCEL_Y, GPS_TIME, GPS_ALTITUDE, GPS_LATITUDE, GPS_LONGITUDE, GPS_SATS, CMD_ECHO`

- **Field count:** **22** fields (indices **0–21**). This build **does not** append optional trailing columns; if the GCS supports extras, they will not appear from this firmware unless extended later.

### 2.3 Field reference (0-based index)

| Index | Content | Notes |
|------:|---------|--------|
| 0 | TEAM_ID | Zero-padded 4 digits, e.g. `1057` |
| 1 | MISSION_TIME | `hh:mm:ss` or `00:00:00` if not set |
| 2 | PACKET_COUNT | `uint32_t`; see §2.5 |
| 3 | MODE | `F` = flight, `S` = simulation |
| 4 | STATE | See §2.4 |
| 5 | ALTITUDE | Baro AGL (m), `%.1f` |
| 6 | TEMPERATURE | `%.1f` |
| 7 | PRESSURE | `%.1f` (kPa in code comments) |
| 8 | VOLTAGE | `%.1f` |
| 9 | CURRENT | `%.2f` |
| 10–12 | GYRO_R, GYRO_P, GYRO_Y | `%.1f` each |
| 13–15 | ACCEL_R, ACCEL_P, ACCEL_Y | `%.1f` each |
| 16 | GPS_TIME | `hh:mm:ss` or `00:00:00` |
| 17 | GPS_ALTITUDE | `%.1f` |
| 18 | GPS_LATITUDE | `%.4f` |
| 19 | GPS_LONGITUDE | `%.4f` |
| 20 | GPS_SATS | integer |
| 21 | CMD_ECHO | Short echo of last handled command; **max 31 chars + NUL** (`commandEcho[32]`) — **avoid commas** in echoes |

### 2.4 STATE column (`flightStateToString`)

| String | Meaning |
|--------|--------|
| `PRELAUNCH` | Waiting for CX,ON + ST before pad ops |
| `LAUNCH_PAD` | On pad, ready to launch |
| `ASCENT` | Ascending |
| `APOGEE` | Transient; may appear rarely (logic often advances same tick) |
| `DESCENT` | Descending before probe release |
| `PROBE_RELEASE` | Probe-release regime (name only — **no physical probe release or paraglider guidance occurs this flight**, see notice above) |
| `PAYLOAD_RELEASE` | Payload-release regime (name only — **no physical egg release occurs this flight**) |
| `LANDED` | Landed |
| `UNKNOWN` | Invalid enum value (should not appear in normal operation) |

> These state names/transition triggers (altitude, vertical velocity) are unchanged from the full-mission FSW — GCS state-machine displays and altitude/velocity-based transition logic do not need updating. Only the physical consequence of entering `PROBE_RELEASE` / `PAYLOAD_RELEASE` is different for this flight (none).

### 2.5 PACKET_COUNT behavior

- Stored in **EEPROM** (persistence across reset).
- Each telemetry line contains **`packetCount` as it was before that send**; `incrementPacketCount()` runs **after** `xbeeSend()` for that line. So the counter in the CSV is the **sequence number of that packet** (then RAM/EEPROM advance for the next).
- **`CAL` command** calls `resetPacketCount()` → counter **0** and EEPROM updated.

---

## 3. Commands (GCS → FSW)

### 3.1 Framing

- Expected prefix: **`CMD,<TEAM_ID>,`**
- **Team ID must be `1057`** or the line is rejected.
- Trailing commas are **not required**. `CMD,1057,CAL` is valid; so is `CMD,1057,CAL,`.

### 3.2 Supported commands

| Command | Example | Effect |
|---------|---------|--------|
| **CX** | `CMD,1057,CX,ON\r\n` / `OFF` | Enable/disable telemetry transmission |
| **ST** | `CMD,1057,ST,12:34:56\r\n` | Set mission time from UTC string |
| **ST** | `CMD,1057,ST,GPS\r\n` | Set mission time from GPS time (if valid) |
| **SIM** | `CMD,1057,SIM,ENABLE\r\n` | Arm simulation (EEPROM); **must** precede ACTIVATE |
| **SIM** | `CMD,1057,SIM,ACTIVATE\r\n` | Enter simulation mode (requires ENABLE) |
| **SIM** | `CMD,1057,SIM,DISABLE\r\n` | Leave simulation, clear stored sim flags |
| **SIMP** | `CMD,1057,SIMP,101325\r\n` | Set simulated pressure (Pa); **only if simulation active** |
| **CAL** | `CMD,1057,CAL\r\n` | Zero altitude + reset packet count |
| **MEC** | `CMD,1057,MEC,PAYLOAD,ON\r\n` | ~~Nudge canister-separation hatch servo 10°~~ **Disabled for this flight — no-op, see notice above** |
| **MEC** | `CMD,1057,MEC,EGG,ON\r\n` | ~~Nudge egg-drop servo 10°~~ **Disabled for this flight — no-op** |
| **MEC** | `CMD,1057,MEC,FS1,ON\r\n` / `OFF` | ~~Flight surface 1 test angle~~ **Disabled for this flight — no-op** |
| **MEC** | `CMD,1057,MEC,FS2,ON\r\n` / `OFF` | ~~Flight surface 2 test angle~~ **Disabled for this flight — no-op** |

- **All `MEC` device actions are currently no-ops** (`src/servos/servos.cpp` functions are stubbed out for this test flight — sensors/telemetry only, no physical actuation). The command is still **parsed, accepted, and echoed** exactly as before (protocol-compatible), but **no servo will move**. GCS should not expect any physical/visual confirmation from `MEC` commands during this flight.
- **MEC** still sets an echo like `MEC<DEVICE><ON_OFF>` (e.g. `MECPAYLOADON`) regardless of the no-op — the echo alone does not confirm actuation occurred.
- Normally (hardware enabled), MEC nudge commands move the servo only **10°** (configurable via `MEC_NUDGE_DEG` in `servos.cpp`) to confirm the servo is alive without straining the battery, without setting the release flag. That behavior is preserved in code (commented out) and can be restored post-flight-test.

### 3.3 Simulation (for GCS test profiles)

1. Send **`SIM,ENABLE`**, then **`SIM,ACTIVATE`**.
2. Then **`SIMP,<Pa>`** is accepted.
3. FSW sensor fusion for heading may differ in simulation vs flight (see `getHeadingReferenceDeg` in `Sensors.cpp`).

---

## 4. Mission / pad flow (for GCS UI)

- Transition **PRELAUNCH → LAUNCH_PAD** requires **`telemetryActive()`** (CX on + XBee ready) **and** **`timeSetComplete()`** (successful **`ST`** once).
- Operators should plan: **CX,ON**, **ST** (or **ST,GPS**), then pad/arm logic as per your checklist.
- For this test flight, mission progression through all states (including `PROBE_RELEASE` and `PAYLOAD_RELEASE`) is expected to happen automatically from sensor data exactly as in a full mission — the only difference is **no servo/mechanism/flight-surface hardware will actuate** at any point. Do not expect probe separation, egg drop, or paraglider steering; this run is validating sensors + telemetry + state-machine timing only.

---

## 5. Checklist for GCS compatibility

1. **Baud 9600** on the serial port talking to the ground XBee (unless both ends changed).
2. **Parse telemetry lines on `\r\n`**; split **22** CSV fields minimum.
3. **Accept TEAM_ID `1057`** (or don’t hard-reject if you multi-team later).
4. **Send commands with `CMD,1057,...`** and **CRLF**.
5. **CAL:** `CMD,1057,CAL` works with or without a trailing comma.
6. **STATE `PRELAUNCH`** is now the explicit string for the pre-pad phase (no longer `UNKNOWN`).
7. **Lost-packet estimates:** `PACKET_COUNT` can jump after power cycle (EEPROM); **`CAL`** resets it.

---

## 6. Source map (FSW)

| Topic | File |
|--------|------|
| Telemetry line format | `src/telemetry/telemetry.cpp` |
| Commands | `src/commands/Commands.cpp` |
| XBee UART / line read | `src/comms/XBee.cpp` |
| Team ID constant | `src/main.cpp` (`TEAM_ID`) |
| Flight state strings | `src/flight/FlightState.cpp` |
| Servo/flight-surface disable (this flight) | `src/servos/servos.cpp` (real implementation preserved under `#if 0`, public API stubbed to no-ops), `include/servos.h` |

---

*Revision: generated from FSW tree; update when telemetry or command parser changes. Last updated to note servo/flight-control-surface hardware is disabled for the current test flight (sensors + telemetry validation only); revert this note when full-mission hardware is re-enabled.*
