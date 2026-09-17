# Phase 4: Full Mission & Failsafes — Hardware Test Plan

**Scope:** Verify mission state machine progression, failsafe injection handling (battery, link loss, geofence breach, companion halt, manual abort), autonomous return breadcrumb retracing, doorway exit transit, precision launch pad landing, and full integrated competition runs on the physical vehicle.

---

## The Core Rule

> **Phase 3 MUST be fully PASSED before starting Phase 4.**  
> Failsafe trigger testing MUST be performed initially on the bench with **PROPS REMOVED**, or in a tethered/netted flight area before free-flight deployment. A dedicated manual abort switch (`KILL` / `AUTO.LAND`) must be pre-tested on the RC transmitter.

---

## Code Import & Reuse Strategy

All Phase 4 verification scripts in `hardware/phase4/scripts/` import directly from existing workspace modules — zero new verification logic:

| Import Source | Functions / Modules Reused | Used In Test |
|---|---|---|
| [`catkin_ws/src/nidar_mission/scripts/entry_detection_module.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/catkin_ws/src/nidar_mission/scripts/entry_detection_module.py) | `MissionState`, `MultiCueEntryDetector` — state transitions & door cues | 4.1 State Machine, 4.4 Return/Exit |
| [`scripts/verify_full_flight.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_full_flight.py) | `Verifier` class — takeoff, altitude, entry, explore, return, landing validation | 4.1 State Machine, 4.4 Return, 4.5 Full Run |
| [`scripts/strict_monitor.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/strict_monitor.py) | `state_cb()`, `pose_cb()`, watchdog monitoring, anomaly detectors | 4.2 Battery & Link Loss, 4.5 Full Run |
| [`scripts/flight_envelope_guard.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/flight_envelope_guard.py) | `FlightEnvelopeGuard`, `euler_from_quaternion()`, `clamp()`, HOLD state stream | 4.3 Envelope & Abort |
| [`config/flight_envelope_guard.yaml`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/config/flight_envelope_guard.yaml) | Envelope boundaries, margins, vision/range watchdog timeouts | 4.3 Failsafe Config |
| [`scripts/verify_flight.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_flight.py) | `load_walls()`, `clearance_fn()`, `CRUISE_Z` — physical arena geometry | 4.4 Return & Exit |
| [`scripts/verify_components.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/verify_components.py) | Distance math, launch pad boundary validation | 4.4 Precision Landing |
| [`scripts/analyze_exploration.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/analyze_exploration.py) | `Analyzer` class — coverage metrics, revisit scoring, completion detection | 4.5 Full Run Scoring |
| [`scripts/mission_telemetry_logger.py`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/scripts/mission_telemetry_logger.py) | End-to-end mission telemetry recording and CSV generation | 4.5 Mission Telemetry |
| [`catkin_ws/src/nidar_mission/config/mission_config.yaml`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/catkin_ws/src/nidar_mission/config/mission_config.yaml) | Launch pad pose `(0, -9.5)`, south door `(0, -7.5)`, plateau window, max duration | 4.1, 4.4, 4.5 Mission Config |

## Persistent Execution & Implementation Logging

All execution runs and gating decisions are recorded in the common reports directory:
- [`hardware/reports/HARDWARE_STATUS_REPORT.md`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_STATUS_REPORT.md)
- [`hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md`](file:///media/satyam/OS/Users/ASUS/Desktop/ASTRA_AIR_MOUSE_/hardware/reports/HARDWARE_IMPLEMENTATION_LOG.md)
- Phase 4 persistent state: `hardware/reports/phase4_state.json`

---

## 1. Mission State Machine & Progression (Item 4.1)

### Objective
Verify that the autonomous state progression follows:
$$\text{PREFLIGHT} \longrightarrow \text{TAKEOFF} \longrightarrow \text{ENTRY\_SEARCH} \longrightarrow \text{ENTRY\_CONFIRMATION} \longrightarrow \text{EXPLORATION} \longrightarrow \text{RETURN} \longrightarrow \text{LAND}$$
and that each state transition is governed by physical sensor confirmation (e.g., baro/TFmini height $\ge 1.4\text{ m}$, doorway crossing confirmation via `MultiCueEntryDetector`, and coverage completion/plateau), rather than blind timers.

### Test Procedure
```bash
# 1. Start mission stack with bench simulation or tethered vehicle
roslaunch nidar_mission nidar_mission.launch

# 2. Observe state topic
rostopic echo /nidar/mission_state

# 3. Verify takeoff triggers at target height
# 4. Verify doorway entry confirmed with opening cues
# 5. Verify transition to EXPLORATION
```

### Imported Verification
```python
from catkin_ws.src.nidar_mission.scripts.entry_detection_module import MissionState, MultiCueEntryDetector
from scripts.verify_full_flight import Verifier
```

### Pass Criteria
- Strict ordered state transitions: `PREFLIGHT` $\to$ `TAKEOFF` $\to$ `ENTRY` $\to$ `EXPLORATION` $\to$ `RETURN` $\to$ `LAND`.
- No backward or invalid state jumps.
- Physical confirmations verified before each transition.

---

## 2. Failsafe System: Battery & Communications Loss (Item 4.2)

### Objective
Verify system resilience against low battery conditions and ground control station telemetry link loss:
1. **Low Battery:** Voltage drop below $14.2\text{ V}$ or remaining capacity $< 20\%$ triggers immediate abort and return home.
2. **Link Loss:** Loss of GCS heartbeat ($> 3.0\text{ s}$) triggers autonomous return without vehicle freezing.

### Test Procedure
```bash
# 1. Battery failsafe trigger test (Bench / Props OFF):
# Inject simulated battery state via MAVROS
rostopic pub /mavros/battery sensor_msgs/BatteryState "{voltage: 13.8, percentage: 0.15}"

# 2. Verify state machine transitions to RETURN
rostopic echo -n 1 /nidar/mission_state  # Expected: RETURN

# 3. GCS Link Loss test:
# Disconnect or terminate GCS telemetry link
# Observe telemetry watchdog trigger at t > 3.0 s
```

### Imported Verification
```python
from scripts.strict_monitor import state_cb, pose_cb
from scripts.verify_flight import load_walls
```

### Pass Criteria
- Low battery event ($< 20\%$ or $\le 14.2\text{ V}$) switches state to `RETURN` within $\le 0.5\text{ s}$.
- GCS link interruption for $> 3.0\text{ s}$ triggers autonomous return.
- No unhandled exceptions or thread lockups during failsafe execution.

---

## 3. Failsafe System: Geofence Breach, Companion Halt & Manual Abort (Item 4.3)

### Objective
Verify physical containment and emergency takeover capabilities:
1. **Geofence Breach:** Position command exceeding world envelope ($X \in [-7.0, 7.0]$, $Y \in [-10.5, 7.0]$, $Z \in [1.45, 1.55]$) is clamped and triggers `HOLD` mode (20 Hz zero-velocity stream).
2. **Companion Software Halt:** Interruption of offboard companion commands ($> 1.0\text{ s}$) triggers PX4 offboard failsafe (altitude hold / slow descent).
3. **Manual RC Abort:** Physical switch flips from `OFFBOARD` to `KILL` / `AUTO.LAND` instantly override companion setpoints.
4. **30-Minute Mission Timeout:** Remaining battery/time budget estimator orders return before flight limit.

### Test Procedure
```bash
# 1. Test Geofence Clamp via envelope guard:
rostopic pub /quadrotor_cmd quadrotor_msgs/PositionCommand "{position: {x: 10.0, y: 0.0, z: 1.5}}"
# Verify guard rejects out-of-bounds command and publishes safe clamped setpoint

# 2. Test Companion Timeout:
# Terminate offboard publisher; verify PX4 issues 'Offboard lost' warning within 1.0 s

# 3. Verify RC Manual Switch takeover on bench
```

### Imported Verification
```python
from scripts.flight_envelope_guard import FlightEnvelopeGuard, euler_from_quaternion, clamp
```

### Pass Criteria
- Out-of-bounds commands rejected; stationary hold commanded within $\le 100\text{ ms}$.
- Companion heartbeat loss detected within $\le 1.0\text{ s}$.
- Manual RC switch preempts offboard commands with 100% reliability.

---

## 4. Autonomous Return, Exit & Precision Landing (Item 4.4)

### Objective
Verify that following exploration completion, the vehicle:
1. Retraces its recorded breadcrumb path in reverse (guaranteeing collision-free egress).
2. Traverses the south doorway ($1.90\text{ m}$ opening at $Y = -7.50\text{ m}$) back to the external space.
3. Precisely touches down on the launch pad center `(0.0, -9.5)` with landing error $\le 1.0\text{ m}$.

### Test Procedure
```bash
# 1. Trigger exploration completion:
rostopic pub /exploration_completed std_msgs/Bool "data: true"

# 2. Track return trajectory waypoints:
rostopic echo /nidar/return_waypoints

# 3. Monitor landing precision on touchdown:
rostopic echo -n 1 /mavros/local_position/pose
```

### Imported Verification
```python
from scripts.verify_full_flight import Verifier
from scripts.verify_components import check
```

### Pass Criteria
- Reverse breadcrumb trail maintains $\ge 0.30\text{ m}$ wall clearance throughout.
- Successful south doorway passage without collision.
- Final landing position $\le 1.0\text{ m}$ from launch pad center `(0.0, -9.5)`.

---

## 5. Full Integrated Competition Mission Run (Item 4.5)

### Objective
Execute a complete, end-to-end competition profile:
1. Operator issues single "START" command.
2. Autonomous takeoff from pad $\to$ Doorway approach & entry $\to$ FUEL progressive exploration.
3. $\ge 98.0\%$ arena mapping coverage.
4. $\ge 5/6$ survivors localized and tagged in correct grid cells (A1–N14).
5. Continuous GCS telemetry and 2D map stream.
6. Autonomous return, exit through doorway, and touchdown on pad.
7. Total mission duration $\le 25\text{ minutes}$.

### Test Procedure
```bash
# Execute master full flight verifier
python3 scripts/verify_full_flight.py
python3 scripts/analyze_exploration.py
```

### Imported Verification
```python
from scripts.verify_full_flight import Verifier
from scripts.analyze_exploration import Analyzer
from scripts.mission_telemetry_logger import telemetry_logger
```

### Pass Criteria
- Mapping coverage $\ge 98.0\%$ verified against arena free-space polygon.
- At least 5 of 6 survivors tagged with 3D error $\le 0.30\text{ m}$.
- Landing within $\le 1.0\text{ m}$ of pad center.
- Total elapsed flight time $\le 25\text{ minutes}$.
- Zero manual operator interventions required during execution.

---

## Sign-Off Summary Sheet

| # | Verification Item | Script Path | Imported / Checked Symbols | Status |
|:---:|---|---|---|:---:|
| 1 | Mission State Machine & Progression | `hardware/phase4/scripts/check_mission_state_machine.py` | `catkin_ws/.../entry_detection_module.py` (`MissionState`, `MultiCueEntryDetector`), `scripts/verify_full_flight.py` (`Verifier`) | PENDING |
| 2 | Failsafe System: Battery & Link Loss | `hardware/phase4/scripts/check_failsafe_battery_link.py` | `scripts/strict_monitor.py` (`state_cb`, `pose_cb`), `scripts/verify_flight.py` (`load_walls`) | PENDING |
| 3 | Failsafe System: Geofence & Abort | `hardware/phase4/scripts/check_failsafe_abort_guard.py` | `scripts/flight_envelope_guard.py` (`FlightEnvelopeGuard`, `clamp`), `config/flight_envelope_guard.yaml` (`bounds`) | PENDING |
| 4 | Autonomous Return & Precision Landing | `hardware/phase4/scripts/check_return_landing.py` | `scripts/verify_full_flight.py` (`Verifier`), `scripts/verify_components.py` (`check`, pad distance) | PENDING |
| 5 | Full Integrated Competition Run | `hardware/phase4/scripts/check_full_integrated_mission.py` | `scripts/verify_full_flight.py` (`Verifier`), `scripts/analyze_exploration.py` (`Analyzer`), `scripts/mission_telemetry_logger.py` | PENDING |
