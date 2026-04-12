# CanSat 2026 FSW – Remaining Work

Substantive items still on you before competition or flight test—not stylistic nits.

---

## 1. Hardware alignment

| Item | Notes |
|------|--------|
| **Servo pins** | `SERVO_*_PIN` in `servos.cpp` must match Teensy 4.1 wiring. |
| **Throws and limits** | Bench-test probe/payload open/close angles and paraglider `SERVO_MIN_ANGLE` / `SERVO_MAX_ANGLE` so the airframe cannot bind or over-travel. |

---

## 2. Flight test and tuning (`servos.cpp`)

| Item | Notes |
|------|--------|
| **Gains and thresholds** | Stage distances, altitudes, `HEADING_KP_*`, `GYRO_DAMP_*`, vertical-rate PI, and `MIN_STAGE_DWELL_MS` need validation under real glide (and possibly adjustment for fast altitude changes vs dwell). |
| **Gyro damper sign** | Confirm `getGyroYaw()` sign vs the `- gyroDampGain * yawRate` term so yaw damping is stabilizing, not feeding back. |

---

## 3. EEPROM flight state vs mechanisms (only if you rely on state restore)

If `flightState` is restored from EEPROM after a reset **mid-mission**, RAM flags such as `probeReleased` / `payloadReleased` are reinitialized in `initServos()` and can disagree with the airframe. Define a team policy: e.g. clear EEPROM for a fresh mission, or add an explicit sync after `initFlightState()` so software state matches assumed hardware state.

---

*Update this file when a category is fully verified in hardware or competition.*

---

Configured values already set in code:
- `TEAM_ID = 1057`
- `setTargetLocation(38.375961f, -79.607872f)` (from 38°22'33.66"N, 79°36'28.34"W)
