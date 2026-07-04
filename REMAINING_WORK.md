# CanSat 2026 FSW – Remaining Work

Substantive items still on you before competition or flight test—not stylistic nits.

---

## 0. Current test flight: servo / flight-control-surface hardware is DISABLED

For the upcoming launch, all servo mechanisms and paraglider flight-control surfaces are software-disabled (`src/servos/servos.cpp` — real implementation preserved under `#if 0`, public API stubbed to no-ops; see `GCS_AGENT_INTEGRATION.md`). This flight validates **sensors + telemetry + state-machine timing only**; the CanSat separates from the rocket but takes no physical action afterward. Sections 1 and 2 below (servo pin alignment, throw/limit bench tests, paraglider gain tuning) are **moot for this flight** and deferred until hardware actuation is re-enabled for a full-mission flight. Re-verify them before that happens.

---

## 1. Hardware alignment (deferred — moot while servos are disabled, see §0)

| Item | Notes |
|------|--------|
| **Servo pins** | `SERVO_*_PIN` in `servos.cpp` must match Teensy 4.1 wiring. |
| **Throws and limits** | Bench-test probe/payload open/close angles and paraglider `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE` so the airframe cannot bind or over-travel. |

---

## 2. Flight test and tuning (`servos.cpp`) (deferred — moot while servos are disabled, see §0)

| Item | Notes |
|------|--------|
| **Gains and thresholds** | Stage distances, altitudes, `HEADING_KP_*`, `GYRO_DAMP_*`, vertical-rate PI, and `MIN_STAGE_DWELL_MS` need validation under real glide (and possibly adjustment for fast altitude changes vs dwell). |
| **Gyro damper sign** | Confirm `getGyroYaw()` sign vs the `- gyroDampGain * yawRate` term so yaw damping is stabilizing, not feeding back. |

---

## 3. EEPROM flight state vs mechanisms (moot while servos are disabled, see §0)

If `flightState` is restored from EEPROM after a reset **mid-mission**, RAM flags such as `probeReleased` / `payloadReleased` are reinitialized in `initServos()` and can disagree with the airframe. Not a concern while actuation is disabled (no physical state to disagree with). Define a team policy before re-enabling hardware: e.g. clear EEPROM for a fresh mission, or add an explicit sync after `initFlightState()` so software state matches assumed hardware state.

---

## 4. Telemetry cleanup (pre-competition)

| Item | Notes |
|------|--------|
| **`[GPS_RAW]` debug line** | `sendTelemetry()` in `telemetry.cpp` sends an extra, non-spec debug line (`[GPS_RAW] ...`) over the radio after every telemetry packet, plus a USB `Serial` mirror. Intentionally **kept for now** to help confirm GPS lock during this sensor/telemetry test flight. Per `rules.txt` §3.1.1.1, `OPTIONAL_DATA` must be additional comma-delimited fields on the **same** telemetry line, not a separate line — this extra line is not competition-format-compliant and should be removed (or converted to proper trailing `OPTIONAL_DATA` fields) before any scored competition flight. |

---

*Update this file when a category is fully verified in hardware or competition.*

---

Configured values already set in code:
- `TEAM_ID = 1057`
- `setTargetLocation(38.375961f, -79.607872f)` (from 38°22'33.66"N, 79°36'28.34"W)
