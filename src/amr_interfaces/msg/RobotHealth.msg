# Structured, timestamped per-robot operational-health state.
# Published by robot_health/health_monitor at a fixed rate on /<robot_ns>/health/state.
# An "agent" section is intentionally omitted here since it belongs to the separate
# agent-decision layer; adding placeholder fields for it now would be fabricated data.

string robot_id
builtin_interfaces/Time stamp

# --- energy ---
float32 battery_soc            # 0..1 state of charge
float32 battery_health         # 0..1, degrades permanently under BATTERY_CAPACITY_DEGRADATION
float32 discharge_rate         # soc fraction lost per second at current conditions

# --- mechanical_health ---
float32 drive_left_health      # 0..1
float32 drive_right_health     # 0..1
float32 caster_health          # 0..1

# --- sensors ---
float32 lidar_confidence       # 0..1
float32 camera_confidence      # 0..1
float32 imu_confidence         # 0..1
float32 ultrasonic_confidence  # 0..1

# --- navigation ---
float32 localization_confidence  # 0..1, derived from sensor confidences
bool path_blocked
uint32 nav_failures_recent        # NavigateToPose ABORTED count since node start

# --- communication ---
float32 packet_loss            # 0..1, only meaningful once peer-to-peer comms exist (Phase 8+)
bool central_manager_alive     # static true until Phase 4 wires this to a real heartbeat check

# --- workload ---
uint32 completed_tasks_recent  # NavigateToPose SUCCEEDED count since node start
float32 distance_recent_m      # odometry-integrated distance travelled since node start

string[] active_fault_types    # fault_type of every currently-active injected fault
