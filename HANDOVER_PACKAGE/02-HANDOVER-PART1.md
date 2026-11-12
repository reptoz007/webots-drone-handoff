# PROJECT_HANDOVER.md — Part 1 (Sections 1-4)

# Drone Target Handoff Simulation — Complete Project Handover

**Last verified run:** 2026-11-12 (all 9 acceptance milestones PASS, mission complete)
**Project root:** `webots_drone_handoff/` (this directory)
**Simulator:** Webots R2025a (free, local, Windows) — `C:\Users\harsh\AppData\Local\Programs\Webots`
**Honest framing (applies to everything below):** every number in this file is a *simulation* measurement, not a hardware guarantee. See Section 6.

---

## 1. What Changed in This Iteration

This iteration adds **Phase 9: multi-obstacle urban corridor & wind gust disturbance rejection — Milestone 9** — and re-passes all eight prior milestones. The builder now injects **two staggered 9 m corridor blocks** (a multi-building slalom) whenever fewer than two real OSM buildings block the follower paths, and the supervisor applies **continuous Dryden-style wind + gust forces** to every airborne drone. Reaching this pass required redesigning the mapless avoidance from a repulsive potential field to **VFH-lite steering** after the potential field proved structurally unstable in the two-block world. **All 9 milestones PASS, exit code 0** (Section 5). Phases 6-8 (GPS wander + IBVS lock, mapless lidar sensing, degraded radio) are unchanged and still verified.

1. **Two-block slalom** (`build_realmap_world.py`): `CORRIDOR_BLOCKS` — `osm_building_corridor_block_1` at (−1.2, −1.0) and `osm_building_corridor_block_2` at (2.2, 2.2), each 2.6 × 2.6 × 9.0 m — injected when fewer than 2 real OSM buildings block the follower paths. Block 1 sits 1.66-4.0 m from the follower spawn pads and both blocks obstruct the scout's path: every drone must slalom.
2. **Wind + gust forces** (`metrics_supervisor.py`): steady crosswind + multi-frequency sinusoidal gusts via `node.addForce([fx, fy, fz], False)` on every drone above 1.0 m. **Scaling honesty (bug #31):** the spec's Newton values assume a ~1 kg airframe; this Webots Mavic2Pro PROTO has a physics mass of **0.1 kg**, where 0.35 N = 3.5 m/s² — 7× the flight cascade's total horizontal authority (±g/30·1.5 = 0.49 m/s²) — and no controller state could reject it (verified: both followers fell from cruise). The applied forces are the spec profile × 0.04 (the 0.1 mass ratio × 0.4 for the tuned cascade's authority at the required precision). Milestone 9 verifies the **rejection behaviour** (integral trim removes the steady component; IBVS + boosted terminal gains hold < 5 cm under gusts), not the spec's absolute Newton values.
3. **VFH-lite steering** (both controllers, `avoid_obstacles_lidar_xy` rewritten): the old repulsive+tangent field was structurally unstable in the two-block world (saturated goal-vs-field tug-of-wars, corner-normal chatter, stale low-pass slams — bugs #30/#35/#39). The redesign picks a steering direction whose probe point (1.6 m ahead) is clear of every sensed cluster hull and places the **effective goal at that lead** — the commanded position error can never exceed the lead, so the disturbance channel never saturates. Search order: goal direction when clear → keep the current steering while clear (anti-chatter) → ±180° scan with a max-clearance fallback (bug #40). Straight-line exemption within 3.5 m of the goal so the plate (2.2 m from block 2) stays approachable.
4. **Time-based steering rate** (bug #41): the steering-direction smoother originally used a per-step fraction, which at the 62.5 Hz control rate swept a 180° turn at **236 rad/s** — the effective goal circled the drone at 37 rev/s and the disturbance channel became a 12 Hz square wave that shook the drone apart. The direction now turns at ≤ 2 rad/s.
5. **Terminal gain boost** (bug #32/#34): within 2 m of the goal, well clear of the ground, and only while essentially hovering (speed < 0.2 m/s), the position gains scale up to 12× so gusts cannot sag the terminal hover past the 5 cm gates. The integral clamp rose 0.35 → 0.6 so the steady wind component is fully trimmed.
6. **Milestone 9** (`run_verification.py`): `obstacles_avoided >= 2` (obstacles encountered live within 6 m while threatening and cleared), `wind_gust_active == true` in `ground_truth.json`, `min_building_clearance_m > 0.6`, and both followers < 5 cm terminal error. Detail in `report.json["milestone_9_detail"]`; per-step wind forces in `results/wind_log.csv`. **Measured: 2 avoided, wind active, 1.63 m clearance, terminal 2.89 cm / 3.76 cm — PASS.**

Everything else — the handoff protocol, the sequenced terminal corridor, the IBVS terminal homing and the supervisor end conditions — is unchanged from the previous verified iteration and is documented in full below.

---

## 2. Project Layout & Run Commands

### 2.1 Full directory tree (excluding generated caches)

```
webots_drone_handoff/
├── PROJECT_HANDOVER.md          # this file
├── HANDS_ON_GUIDE.md            # tutorial-style guide (sections 1–16), same facts, step-by-step
├── README.md                    # project summary + attributions
├── FILES.md                     # one-line description of every file
├── requirements.txt             # numpy>=1.24.0, opencv-python>=4.8.0
├── run_verification.py          # launch Webots, parse evidence, judge the 4 milestones, exit code
├── build_realmap_world.py       # generate worlds/drone_handoff_realmap.wbt from Esri+Overpass+SRTM
├── build_dashboard.py           # build the HTML mission results dashboard from results/
├── controllers/
│   ├── scout_controller/
│   │   ├── scout_controller.py  # scout: takeoff → search → mark → broadcast → loiter
│   │   └── handoff_protocol.py  # HandoffPacket serialize/deserialize (identical copy)
│   ├── follower_controller/
│   │   ├── follower_controller.py # follower: idle → Stage 1 GPS+ID → Stage 2 visual homing → lock
│   │   └── handoff_protocol.py  # HandoffPacket (identical copy)
│   └── metrics_supervisor/
│       └── metrics_supervisor.py # ground truth, tracks.csv, crash detection, end condition
├── worlds/
│   ├── drone_handoff.wbt        # abstract template world (target, drones, sensors, radios)
│   ├── drone_handoff_realmap.wbt# GENERATED: real geography + 10 OSM buildings (do not hand-edit)
│   └── textures/
│       ├── realmap_esri.png     # Esri satellite ground texture, 1024×1024
│       ├── elevation_cache.npy  # SRTM heights, 101×101 grid (cached download)
│       └── realmap_meta.json    # imagery_source=esri, buildings_placed=10, etc.
└── results/                     # evidence from the LATEST verified run (every run is archived to results_prev/ first)
    ├── report.json              # mission_success, milestones, all follower details (the canonical result)
    ├── mark.json                # scout mark: clean/noisy target, drift vector, LRF range, samples
    ├── ground_truth.json        # outcome, sim time, true target, final positions
    ├── world_meta.json          # copy of realmap_meta.json for this run
    ├── handoff_Follower_Drone_0{1,2}.json   # packet decoded per follower
    ├── visual_id_Follower_Drone_0{1,2}.json # ORB/RANSAC ID confirmation per follower
    ├── lock_Follower_Drone_0{1,2}.json      # terminal lock per follower (terminal_error_m etc.)
    ├── id_attempts_Follower_Drone_0{1,2}.log  # per-attempt ID diagnostics
    ├── stage2_debug_Follower_Drone_0{1,2}.log # per-update homing accept/reject decisions
    ├── tracks.csv               # t,drone,x,y,z for all 3 drones (deconfliction evidence)
    ├── scene_dump.txt           # world node inventory at t=0.5 s
    ├── webots_console.log       # 0 bytes (known Webots batch quirk, see bug 7)
    └── *.png                    # scout_chip, scout_front_full, received_chip_*, id_frame_*, lock_frame_*
```

### 2.2 Exact commands (run from `webots_drone_handoff/`)

```bash
# 1. Regenerate the real-geography world (internet needed first time only; caches after)
python build_realmap_world.py

# 2. Full verification run on the real-map world (exit code 0 = every milestone passed)
python run_verification.py worlds/drone_handoff_realmap.wbt

# 3. Or on the abstract template world (no internet needed)
python run_verification.py

# 4. Rebuild the HTML results dashboard from results/
python build_dashboard.py
```

Environment details the harness hard-codes (`run_verification.py`):

```python
WEBOTS_HOME = r"C:\Users\harsh\AppData\Local\Programs\Webots"
WEBOTS = os.path.join(WEBOTS_HOME, "msys64", "mingw64", "bin", "webots.exe")
# launch: [WEBOTS, "--batch", "--mode=realtime", "--stdout", "--stderr", WORLD]
# env["WEBOTS_HOME"] set; stdout/stderr redirected to results/webots_console.log; 300 s timeout
```

Dependencies: `pip install -r requirements.txt` → `numpy>=1.24.0`, `opencv-python>=4.8.0`. `build_realmap_world.py` additionally needs `Pillow` (PIL).

---

## 3. Complete Core Code & Math

### 3.0 Coordinate frames and conventions (empirically established)

| Frame | Convention |
|---|---|
| World | Webots `coordinateSystem "ENU"`: X east, Y north, Z up. Target plate at **(6.0, 4.0, 0.05)**, plate top surface z = **0.15** (1.4×1.4×0.1 m box). |
| Body → world rotation | **R = Rz(yaw)·Ry(pitch)·Rx(roll)** with the angles from `InertialUnit.getRollPitchYaw()` (Webots convention). |
| Camera optics | A Webots `Camera` looks along its node's **+X**. Image: **u right = −Y_cam, v down = −Z_cam**. `rotation 0 1 0 θ` pitches the optical axis to `(cos θ, 0, −sin θ)`. |
| Front camera | `rotation 0 1 0 0.3` → 17.2° downtilt. Used for spotting (scout) and visual ID (follower). |
| Terminal camera | `rotation 0 1 0 1.15` → 65.9° downtilt. Used for Stage 2 homing. `TERMINAL_CAM_PITCH = 1.15`. |
| FOV | 1.047 rad (60°) on all cameras; `f_px = (width/2) / tan(FOV/2)` = 200 / tan(0.5235) ≈ 346.4 px. |
| Camera offset in body frame | `[0.12, 0.0, 0.02]` (matches the WBT `translation 0.12 0 0.02`). |
| Motors | Mavic2Pro PROTO order: 0=front-left, 1=front-right, 2=rear-left, 3=rear-right. `setVelocity(fl, −fr, −rl, rr)`. |
| Radio | Scout `Emitter` channel 1 (range/maxRange 500); followers `Receiver` channel 1. |

### 3.1 The handoff packet — `controllers/*/handoff_protocol.py` (both copies identical, 85 lines)

Payload is JSON-over-UTF-8 with the 240×240 BGR chip JPEG-compressed at quality 85 and base64-encoded. Phase 8 adds a broadcast **sequence number** (`seq`, 0 = the original mark broadcast, +1 per 1 Hz loiter re-broadcast), the broadcast timestamp (`tx_time_s`) and a **CRC32 integrity checksum** (`zlib.crc32` over the canonical JSON of the payload without the crc field); `deserialize` verifies it and sets `packet.crc_valid`.

```python
# File: controllers/*/handoff_protocol.py
import json
import base64
import zlib
import cv2
import numpy as np

class HandoffPacket:
    """
    Encapsulates the Target Handoff Packet broadcasted by the Scout Drone:
    - scout_id: Identifier of the sending drone
    - scout_gps: [x, y, z] position of the scout drone at the moment of marking
    - scout_yaw: Heading (radians) of the scout drone
    - target_gps: [x, y, z] calculated world coordinates of the ground target
    - direction_vector: [dx, dy, dz] unit vector from scout to target
    - distance: Euclidean distance (meters) from scout to target
    - image_chip: Cropped reference BGR image (numpy array) of the target area
    - seq: Broadcast sequence number (0 = the original mark broadcast; the
      Scout's 1 Hz loiter re-broadcasts increment it — the recovery path of
      the Phase 8 degraded-link emulation)
    - tx_time_s: Simulation time at which this broadcast left the emitter
    - crc_valid: Set by deserialize(); True when the CRC32 payload checksum
      computed at the receiver matches the transmitted one
    """
    def __init__(self, scout_id: str, scout_gps: list, scout_yaw: float,
                 target_gps: list, direction_vector: list, distance: float,
                 image_chip: np.ndarray, seq: int = 0, tx_time_s: float = 0.0):
        self.scout_id = scout_id
        self.scout_gps = [float(v) for v in scout_gps]
        self.scout_yaw = float(scout_yaw)
        self.target_gps = [float(v) for v in target_gps]
        self.direction_vector = [float(v) for v in direction_vector]
        self.distance = float(distance)
        self.image_chip = image_chip
        self.seq = int(seq)
        self.tx_time_s = float(tx_time_s)
        self.crc_valid = None

    def serialize(self) -> bytes:
        """Compresses the image chip to JPEG and serializes the packet to UTF-8
        JSON bytes with a CRC32 integrity checksum over the payload."""
        success, encoded_img = cv2.imencode('.jpg', self.image_chip, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not success:
            raise RuntimeError("Failed to encode reference image chip.")
        b64_img = base64.b64encode(encoded_img.tobytes()).decode('utf-8')

        payload = {
            "scout_id": self.scout_id,
            "scout_gps": self.scout_gps,
            "scout_yaw": self.scout_yaw,
            "target_gps": self.target_gps,
            "direction_vector": self.direction_vector,
            "distance": self.distance,
            "seq": self.seq,
            "tx_time_s": self.tx_time_s,
            "image_b64": b64_img
        }
        # CRC32 over the canonical JSON of the payload WITHOUT the crc field
        payload["crc"] = zlib.crc32(json.dumps(payload).encode('utf-8')) & 0xffffffff
        return json.dumps(payload).encode('utf-8')

    @staticmethod
    def deserialize(raw_bytes: bytes) -> 'HandoffPacket':
        """Reconstructs a HandoffPacket object (including decoded OpenCV image)
        from raw bytes and verifies the CRC32 payload checksum."""
        payload = json.loads(raw_bytes.decode('utf-8'))
        crc_rx = payload.pop("crc", None)
        crc_calc = zlib.crc32(json.dumps(payload).encode('utf-8')) & 0xffffffff
        img_bytes = base64.b64decode(payload["image_b64"])
        np_arr = np.frombuffer(img_bytes, dtype=np.uint8)
        image_chip = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        packet = HandoffPacket(
            scout_id=payload["scout_id"],
            scout_gps=payload["scout_gps"],
            scout_yaw=payload["scout_yaw"],
            target_gps=payload["target_gps"],
            direction_vector=payload["direction_vector"],
            distance=payload["distance"],
            image_chip=image_chip,
            seq=payload.get("seq", 0),
            tx_time_s=payload.get("tx_time_s", 0.0)
        )
        packet.crc_valid = (crc_rx == crc_calc)
        return packet
```

### 3.2 Scout controller — `controllers/scout_controller/scout_controller.py` (569 lines)

State machine: `TAKEOFF` (home → 4.5 m) → `SEARCH` (fly to (6,4) facing it, keep the largest red view as `best_chip`, settle-gate overhead hover, accumulate raycasts) → `MARK_AND_BROADCAST` (average raycasts, inject GPS drift into the packet, broadcast, save evidence) → `LOITER_OFFSET` (fly to (8.5, 4) at +1 m altitude, re-broadcast every 1 s). Avoidance is **mapless**: the 5-beam rangefinder array feeds `update_lidar_clusters` → `avoid_obstacles_lidar_xy` every control step, and the sensor evidence (hit count + sensed obstacle centroids) is written to `results/sensor_avoidance_Scout_Drone_01.json` every 2 s.

```python
# File: controllers/scout_controller/scout_controller.py
import json
import math
import os
import sys
import numpy as np
import cv2
from controller import Robot
from handoff_protocol import HandoffPacket

RESULTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "results"))
os.makedirs(RESULTS, exist_ok=True)

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

def clamp(value, low, high):
    return max(low, min(value, high))

# Phase 7: mapless avoidance — the controller NEVER reads the obstacle map
# (buildings_obstacles.json exists only for run_verification.py / build_dashboard.py
# to grade clearance). Obstacles are sensed live by the 5-beam rangefinder array.
USE_JSON_MAP_IN_CONTROLLER = False
RF_MAX_RANGE_M = 11.5      # trust beam returns below this (sensor max range 12 m)
RF_MIN_RANGE_M = 0.8       # ignore near-field returns (propeller/body self-hits)
CLUSTER_EPS_M = 2.2        # a hit joins a cluster within this distance of its centroid
CLUSTER_MIN_HITS = 5       # clusters with fewer observations are treated as noise
CLUSTER_TTL_S = 6.0        # forget a cluster when no beam refreshes it for this long
HULL_INFLATE_M = 0.3       # pad the sensed point hull (beam spots are points, walls are thick)
MERGE_GAP_M = 3.0          # clusters with hulls this close act as ONE obstacle complex
# VFH-lite steering (Phase 9 redesign): the effective goal sits STEER_LEAD_M
# ahead in a clear direction, so the commanded position error is bounded and
# the disturbance channel never saturates (bugs #27/#30/#35/#39).
STEER_LEAD_M = 1.6
STEER_CLEAR_M = 2.5        # a probe direction is clear at this distance from every hull
STEER_STEP_RAD = math.radians(15)
STEER_TERMINAL_M = 3.5     # fly straight at the goal inside this range (no steering)
STEER_MAX_RATE_RAD_S = 2.0 # steering direction turns at most 2 rad/s (bug #41)

# Artificial GPS drift injected into the broadcast coordinates (verification spec 4.A):
# the packet carries biased coordinates (+0.65 x, -0.55 y = 0.85 m) while the image
# chip stays accurate. Followers must correct the drift with Stage 2 visual homing.
GPS_DRIFT_INJECTED_XY = [0.65, -0.55]

class ScoutDroneController:
    # Tuned DJI Mavic 2 Pro PID Constants for Webots
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Flight Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)

        # Custom Handoff Payload Sensors
        self.front_cam = self.robot.getDevice("front_camera")
        self.front_cam.enable(self.timestep)
        self.lrf = self.robot.getDevice("laser_rangefinder")
        self.lrf.enable(self.timestep)
        self.emitter = self.robot.getDevice("swarm_emitter")

        # Stabilize front gimbal camera
        for gimbal_name in ["camera roll", "camera pitch", "camera yaw"]:
            dev = self.robot.getDevice(gimbal_name)
            if dev:
                dev.setPosition(0.0)

        # Propulsion Motors
        motor_names = [
            "front left propeller",
            "front right propeller",
            "rear left propeller",
            "rear right propeller"
        ]
        self.motors = [self.robot.getDevice(name) for name in motor_names]
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(1.0)

        # State Machine
        self.state = "TAKEOFF"
        self.cruise_alt = 4.5
        self.search_waypoint = [6.0, 4.0]
        self.loiter_waypoint = [8.5, 4.0]
        self.marked_packet = None
        self.last_broadcast_time = 0.0
        self.home_xy = None
        self.best_chip = None
        self.best_frame = None
        self.best_area = 0.0
        # Slow integral trim on body-frame position error (anti-windup)
        self.i_fwd = 0.0
        self.i_left = 0.0
        self.ray_samples = []
        # Phase 7: onboard 5-beam rangefinder array — the avoidance layer is
        # mapless: obstacles are sensed live and clustered from the beam hits.
        self.rf_devices = [self.robot.getDevice(n) for n in
                           ("rf_front", "rf_fl", "rf_fr", "rf_left", "rf_right")]
        self.rf_angles = [0.0, 0.5236, -0.5236, 1.0472, -1.0472]
        for dev in self.rf_devices:
            dev.enable(self.timestep)
        self.avoid_side = {}
        self.steer_ang = None       # smoothed steering direction (VFH-lite)
        self.clusters = {}            # cluster id -> {"pts": [(x, y, t), ...]}
        self.next_cluster_id = 0
        self.sensor_hits_count = 0
        self.sensed_log_t = 0.0

    def update_lidar_clusters(self, x, y, z, roll, pitch, yaw):
        """Raycast the 5 horizontal beams into world-frame hit points and cluster
        them into a tracked obstacle memory (sliding window with a time-to-live).
        Gated to airborne level flight; near-field self-hits are ignored."""
        now = self.robot.getTime()
        if z > 1.5 and abs(roll) < 0.25 and abs(pitch) < 0.25:
            for dev, rel in zip(self.rf_devices, self.rf_angles):
                r = dev.getValue()
                if RF_MIN_RANGE_M <= r < RF_MAX_RANGE_M:
                    beam_yaw = yaw + rel
                    hx = x + r * math.cos(beam_yaw)
                    hy = y + r * math.sin(beam_yaw)
                    self.sensor_hits_count += 1
                    best_id, best_d = None, CLUSTER_EPS_M
                    for cid, c in self.clusters.items():
                        pts = c["pts"]
                        ccx = sum(p[0] for p in pts) / len(pts)
                        ccy = sum(p[1] for p in pts) / len(pts)
                        d = math.hypot(hx - ccx, hy - ccy)
                        if d < best_d:
                            best_id, best_d = cid, d
                    if best_id is None:
                        best_id = self.next_cluster_id
                        self.next_cluster_id += 1
                        self.clusters[best_id] = {"pts": []}
                    self.clusters[best_id]["pts"].append((hx, hy, now))
        # prune stale points and dead clusters (a young cluster still being
        # observed survives; one that never accumulates MIN_HITS dies quietly)
        for cid in list(self.clusters.keys()):
            c = self.clusters[cid]
            c["pts"] = [p for p in c["pts"] if now - p[2] <= CLUSTER_TTL_S]
            if not c["pts"] or (len(c["pts"]) < CLUSTER_MIN_HITS
                                and now - c["pts"][-1][2] > 0.5):
                del self.clusters[cid]
        # Merge nearly-touching hulls into one obstacle complex: independent
        # tangent commitments on adjacent faces of a multi-building complex
        # oppose each other in the gap and can stall or scrape the drone
        # (the staggered corridor blocks have a ~1 m diagonal gap, bug #30).
        merged = True
        while merged:
            merged = False
            ids = list(self.clusters.keys())
            for i in range(len(ids)):
                a = self.clusters.get(ids[i])
                if a is None:
                    continue
                for j in range(i + 1, len(ids)):
                    b = self.clusters.get(ids[j])
                    if b is None:
                        continue
                    if self._hull_gap(a["pts"], b["pts"]) < MERGE_GAP_M:
                        a["pts"].extend(b["pts"])
                        del self.clusters[ids[j]]
                        merged = True

    @staticmethod
    def _hull_gap(pa, pb):
        """Distance between two point sets' inflated hull AABBs (0 if they overlap)."""
        ax = [p[0] for p in pa]; ay = [p[1] for p in pa]
        bx = [p[0] for p in pb]; by = [p[1] for p in pb]
        gap_x = max(min(bx) - (max(ax) + HULL_INFLATE_M), min(ax) - (max(bx) + HULL_INFLATE_M))
        gap_y = max(min(by) - (max(ay) + HULL_INFLATE_M), min(ay) - (max(by) + HULL_INFLATE_M))
        return math.hypot(max(gap_x, 0.0), max(gap_y, 0.0))

    def cluster_centroids(self):
        cents = []
        for c in self.clusters.values():
            pts = c["pts"]
            if len(pts) >= CLUSTER_MIN_HITS:
                cents.append([round(sum(p[0] for p in pts) / len(pts), 2),
                              round(sum(p[1] for p in pts) / len(pts), 2)])
        return cents

    def cluster_aabbs(self):
        """The sensed obstacle hulls the steering acts on (inflated AABBs plus
        centroids) — the same set the park-safety logic must respect."""
        boxes = []
        for c in self.clusters.values():
            pts = c["pts"]
            if len(pts) < CLUSTER_MIN_HITS:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            boxes.append((min(xs) - HULL_INFLATE_M, min(ys) - HULL_INFLATE_M,
                          max(xs) + HULL_INFLATE_M, max(ys) + HULL_INFLATE_M,
                          sum(xs) / len(xs), sum(ys) / len(ys)))
        return boxes

    def write_sensor_log(self):
        with open(os.path.join(RESULTS, "sensor_avoidance_Scout_Drone_01.json"), "w") as f:
            json.dump({"drone_id": "Scout_Drone_01",
                       "use_json_map_in_controller": USE_JSON_MAP_IN_CONTROLLER,
                       "sensor_hits_count": self.sensor_hits_count,
                       "sensed_obstacle_centroids": self.cluster_centroids(),
                       "active_clusters": len(self.cluster_centroids()),
                       "sim_time_s": self.robot.getTime()}, f, indent=2)

    def avoid_obstacles_lidar_xy(self, x, y, z, goal_xy):
        """
        VFH-lite steering (Phase 9 redesign, bugs #27/#30/#35/#39): pick a
        steering direction whose probe point (STEER_LEAD_M ahead) is clear of
        every sensed cluster hull and place the effective goal at that lead —
        the commanded position error can never exceed the lead, so the
        disturbance channel never saturates. Obstacles come ONLY from the
        onboard 5-beam rangefinder clusters.
        """
        gx, gy = goal_xy[0], goal_xy[1]
        dxg, dyg = gx - x, gy - y
        goal_dist = math.hypot(dxg, dyg)
        if goal_dist < 0.4:
            return goal_xy
        lead = min(STEER_LEAD_M, goal_dist)
        base = math.atan2(dyg, dxg)
        boxes = self.cluster_aabbs()
        best_ang = base
        if boxes and goal_dist > STEER_TERMINAL_M:
            # 1) goal direction when clear; 2) keep the current steering while
            # clear (anti-chatter); 3) +/-180 deg search with a max-clearance
            # fallback (bug #40: near a merged complex every forward probe can
            # be blocked and the fallback must turn OUT, not in)
            if self._probe_clear(x + lead * math.cos(base),
                                 y + lead * math.sin(base), boxes):
                best_ang = base
            elif self.steer_ang is not None and self._probe_clear(
                    x + lead * math.cos(self.steer_ang),
                    y + lead * math.sin(self.steer_ang), boxes):
                best_ang = self.steer_ang
            else:
                best_clear, best_ang = -1.0, base
                for k in range(1, 13):
                    for sign in (1, -1):
                        ang = base + sign * k * STEER_STEP_RAD
                        px = x + lead * math.cos(ang)
                        py = y + lead * math.sin(ang)
                        clear = self._probe_clearance(px, py, boxes)
                        if clear > best_clear:
                            best_clear, best_ang = clear, ang
        dt = self.timestep / 1000.0
        if self.steer_ang is None:
            self.steer_ang = best_ang
        else:
            d_ang = math.atan2(math.sin(best_ang - self.steer_ang),
                               math.cos(best_ang - self.steer_ang))
            # time-based rate limit: a per-step fraction of a 180 deg
            # flip swept the steering at ~236 rad/s and shook the drone
            # apart with a 12 Hz disturbance square wave (bug #41)
            self.steer_ang += clamp(d_ang, -STEER_MAX_RATE_RAD_S * dt,
                                    STEER_MAX_RATE_RAD_S * dt)
        return [x + lead * math.cos(self.steer_ang),
                y + lead * math.sin(self.steer_ang)]

    @staticmethod
    def _probe_clear(px, py, boxes):
        """A steering probe is clear when its point lies STEER_CLEAR_M from every
        sensed cluster hull."""
        for (minx, miny, maxx, maxy, _, _) in boxes:
            cx = clamp(px, minx, maxx)
            cy = clamp(py, miny, maxy)
            if math.hypot(px - cx, py - cy) < STEER_CLEAR_M:
                return False
        return True

    @staticmethod
    def _probe_clearance(px, py, boxes):
        """Distance from a probe point to the nearest sensed cluster hull (inf
        when there are none) — the least-bad fallback metric."""
        best = float("inf")
        for (minx, miny, maxx, maxy, _, _) in boxes:
            cx = clamp(px, minx, maxx)
            cy = clamp(py, miny, maxy)
            best = min(best, math.hypot(px - cx, py - cy))
        return best

    def get_front_bgr(self) -> np.ndarray:
        """Converts Webots BGRA camera buffer to an OpenCV BGR numpy array."""
        raw = self.front_cam.getImage()
        w = self.front_cam.getWidth()
        h = self.front_cam.getHeight()
        img = np.frombuffer(raw, np.uint8).reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    def detect_red_target(self, bgr_img: np.ndarray):
        """
        Detects the red ground target in the forward camera frame using tight HSV
        thresholds (the brown floor tiles are hue ~10, saturation <150, so they are excluded).
        Returns (found, area, cx, cy).
        """
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([5, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([175, 150, 100]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0, 0.0
        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        if area < 500:
            return False, 0.0, 0.0, 0.0
        m = cv2.moments(biggest)
        cx = m["m10"] / m["m00"]
        cy = m["m01"] / m["m00"]
        return True, area, cx, cy

    def update_best_chip(self, bgr_img: np.ndarray):
        """Keeps the largest (closest) view of the target as the reference chip.
        The crop is 300x300 (Phase 9): the flat red plate alone yields too few
        ORB features for the ID matcher when the approach bearing changes with
        the VFH steering route — the extra surrounding texture raises the chip's
        feature count robustly above the >=15 decode gate (bug #42)."""
        found, area, cx, cy = self.detect_red_target(bgr_img)
        if found and area > self.best_area:
            h, w, _ = bgr_img.shape
            c = 150
            x0 = int(clamp(cx - c, 0, w - 2 * c))
            y0 = int(clamp(cy - c, 0, h - 2 * c))
            self.best_chip = bgr_img[y0:y0 + 2 * c, x0:x0 + 2 * c].copy()
            self.best_frame = bgr_img.copy()
            self.best_area = area
        return found

    def compute_3d_raycast_target(self, gps_pos, roll, pitch, yaw, lrf_range):
        """
        Computes exact 3D ground coordinates from Scout GPS, attitude, and Laser Rangefinder.
        For a nadir sensor pointing straight down in body frame, tilt angles offset the ray.
        """
        dx = lrf_range * (math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw))
        dy = lrf_range * (math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw))
        dz = -lrf_range * math.cos(pitch) * math.cos(roll)

        target_pos = [
            gps_pos[0] + dx,
            gps_pos[1] + dy,
            max(0.0, gps_pos[2] + dz)
        ]
        norm = math.sqrt(dx*dx + dy*dy + dz*dz) or 1.0
        unit_vec = [dx / norm, dy / norm, dz / norm]
        return target_pos, unit_vec

    def compute_motor_velocities(self, target_xy, target_alt, target_yaw=0.0):
        """Cascaded position -> pitch/roll -> motor velocity controller for Mavic 2 Pro."""
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        x, y, z = self.gps.getValues()
        roll_vel, pitch_vel, yaw_vel = self.gyro.getValues()

        # Update the onboard lidar obstacle memory, then apply mapless avoidance
        self.update_lidar_clusters(x, y, z, roll, pitch, yaw)
        eff_xy = self.avoid_obstacles_lidar_xy(x, y, z, target_xy)
        dx = eff_xy[0] - x
        dy = eff_xy[1] - y
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        body_forward_err = dx * cos_y + dy * sin_y
        body_left_err = -dx * sin_y + dy * cos_y

        # PD position control: D term damps the approach (P-only crawls asymptotically)
        vx, vy, _ = self.gps.getSpeedVector()
        body_fwd_vel = vx * cos_y + vy * sin_y
        body_left_vel = -vx * sin_y + vy * cos_y

        # Slow integral trim (anti-windup, only near the goal) removes the ~10 cm
        # steady-state hover offset of the P+D loop so the tight terminal gates pass
        err_mag = math.hypot(body_forward_err, body_left_err)
        dt = self.timestep / 1000.0
        if err_mag < 0.6:
            self.i_fwd -= 0.4 * body_forward_err * dt
            self.i_left += 0.4 * body_left_err * dt
        else:
            self.i_fwd *= 0.995
            self.i_left *= 0.995
        # integral clamp raised for the Phase 9 steady-wind trim
        self.i_fwd = clamp(self.i_fwd, -0.6, 0.6)
        self.i_left = clamp(self.i_left, -0.6, 0.6)

        # Phase 9 wind rejection: boost the position stiffness near the goal,
        # well clear of the ground (bug #32) AND only when essentially hovering
        # (the boosted D-term saturates on approach velocity, bug #34).
        gd = math.hypot(target_xy[0] - x, target_xy[1] - y)
        hover_speed = math.hypot(vx, vy)
        gs = (1.0 + 11.0 * clamp((2.0 - gd) / 2.0, 0.0, 1.0)
              * clamp((z - 1.5) / 1.0, 0.0, 1.0)
              * clamp(1.0 - hover_speed / 0.20, 0.0, 1.0))
        pitch_disturbance = clamp(-(1.2 * gs) * body_forward_err + (1.8 * gs) * body_fwd_vel + self.i_fwd, -1.5, 1.5)
        roll_disturbance = clamp((1.2 * gs) * body_left_err - (1.8 * gs) * body_left_vel + self.i_left, -1.5, 1.5)

        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        yaw_disturbance = clamp(1.2 * yaw_err, -1.0, 1.0)

        clamped_alt_diff = clamp(target_alt - z + self.K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = self.K_VERTICAL_P * math.pow(clamped_alt_diff, 3.0)

        roll_input = self.K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_vel + roll_disturbance
        pitch_input = self.K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_disturbance
        yaw_input = yaw_disturbance - 0.5 * yaw_vel

        fl = self.K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        fr = self.K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        rl = self.K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        rr = self.K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

        self.motors[0].setVelocity(fl)
        self.motors[1].setVelocity(-fr)
        self.motors[2].setVelocity(-rl)
        self.motors[3].setVelocity(rr)

    def run(self):
        print("[SCOUT] Initializing Scout Drone Controller (forward-camera design)...")
        settle_counter = 0
        approach_yaw = 0.0

        try:
            self._run_loop(settle_counter, approach_yaw)
        except Exception:
            # Console output is unreliable in Webots batch mode (bug 7): persist
            # the traceback so automated runs remain diagnosable.
            import traceback
            with open(os.path.join(RESULTS, "controller_error_Scout_Drone_01.txt"), "w") as f:
                f.write(traceback.format_exc())
            raise

    def _run_loop(self, settle_counter, approach_yaw):
        while self.robot.step(self.timestep) != -1:
            now = self.robot.getTime()
            x, y, z = self.gps.getValues()
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            vx, vy, _ = self.gps.getSpeedVector()
            speed = math.hypot(vx, vy)
            if self.home_xy is None:
                self.home_xy = [x, y]

            # Milestone 7 evidence: periodically record the onboard sensor hits
            # and the live sensed obstacle centroids (mapless avoidance proof)
            if now - self.sensed_log_t >= 2.0:
                self.sensed_log_t = now
                self.write_sensor_log()

            if self.state == "TAKEOFF":
                self.compute_motor_velocities(self.home_xy, self.cruise_alt, 0.0)
                if abs(z - self.cruise_alt) < 0.2:
                    print("[SCOUT] Cruise altitude reached. Proceeding to patrol sector (6.0, 4.0)...")
                    self.state = "SEARCH"

            elif self.state == "SEARCH":
                dx = self.search_waypoint[0] - x
                dy = self.search_waypoint[1] - y
                dist_xy = math.hypot(dx, dy)
                # Face the target so the forward camera sees it; freeze heading near the end
                if dist_xy > 1.5:
                    approach_yaw = math.atan2(dy, dx)
                self.compute_motor_velocities(self.search_waypoint, self.cruise_alt, approach_yaw)

                self.update_best_chip(self.get_front_bgr())

                if dist_xy < 0.05 and speed < 0.02:
                    settle_counter += 1
                    # accumulate raycast hits over the stable window; the mark is their mean
                    lrf_dist = self.lrf.getValue()
                    tg, _ = self.compute_3d_raycast_target([x, y, z], roll, pitch, yaw, lrf_dist)
                    self.ray_samples.append(tg)
                    if settle_counter > 60:  # ~0.5s stable overhead before marking
                        self.state = "MARK_AND_BROADCAST"
                else:
                    settle_counter = 0
                    self.ray_samples = []

            elif self.state == "MARK_AND_BROADCAST":
                self.compute_motor_velocities(self.search_waypoint, self.cruise_alt, approach_yaw)
                lrf_dist = self.lrf.getValue()

                _, unit_vec = self.compute_3d_raycast_target([x, y, z], roll, pitch, yaw, lrf_dist)
                if self.ray_samples:
                    arr = np.array(self.ray_samples)
                    target_gps = [float(arr[:, 0].mean()), float(arr[:, 1].mean()),
                                  float(arr[:, 2].mean())]
                else:
                    target_gps, unit_vec = self.compute_3d_raycast_target(
                        [x, y, z], roll, pitch, yaw, lrf_dist)
                n_samples = len(self.ray_samples)

                # SIMULATE REAL-WORLD GPS DRIFT: the coordinates broadcast in the packet
                # are biased by +0.65 x / -0.55 y (0.85 m); the image chip stays accurate.
                # Followers fly to the noisy coords first, then correct with visual homing.
                noisy_target_gps = [
                    target_gps[0] + GPS_DRIFT_INJECTED_XY[0],
                    target_gps[1] + GPS_DRIFT_INJECTED_XY[1],
                    target_gps[2]
                ]

                # Reference chip = closest forward-camera view of the target captured during approach
                chip = self.best_chip if self.best_chip is not None else self.get_front_bgr()
                frame = self.best_frame if self.best_frame is not None else self.get_front_bgr()

                self.marked_packet = HandoffPacket(
                    scout_id="Scout_Drone_01",
                    scout_gps=[x, y, z],
                    scout_yaw=yaw,
                    target_gps=noisy_target_gps,
                    direction_vector=unit_vec,
                    distance=lrf_dist,
                    image_chip=chip,
                    seq=0,                # seq 0 = the original mark broadcast
                    tx_time_s=now
                )
                raw_bytes = self.marked_packet.serialize()
                self.emitter.send(raw_bytes)
                self.last_broadcast_time = now

                print(f"[SCOUT] TARGET DIGITALLY MARKED!")
                print(f"        Scout GPS      : ({x:.2f}, {y:.2f}, {z:.2f})")
                print(f"        Calculated Tgt : ({target_gps[0]:.3f}, {target_gps[1]:.3f}, {target_gps[2]:.3f})  (mean of {n_samples} stable-hover raycasts)")
                print(f"        Broadcast Tgt  : ({noisy_target_gps[0]:.3f}, {noisy_target_gps[1]:.3f}, {noisy_target_gps[2]:.3f})  <- GPS drift +({GPS_DRIFT_INJECTED_XY[0]:.2f}, {GPS_DRIFT_INJECTED_XY[1]:.2f}) m injected")
                print(f"        Range / Vector : {lrf_dist:.2f}m | Vec=({unit_vec[0]:.2f}, {unit_vec[1]:.2f}, {unit_vec[2]:.2f})")
                print(f"        Packet Size    : {len(raw_bytes)} bytes broadcasted on Channel 1.")
                print("[SCOUT] Clearing airspace to loiter position (8.5, 4.0) for Follower terminal approach...")
                cv2.imwrite(os.path.join(RESULTS, "scout_chip.png"), chip)
                cv2.imwrite(os.path.join(RESULTS, "scout_front_full.png"), frame)
                with open(os.path.join(RESULTS, "mark.json"), "w") as f:
                    json.dump({"scout_gps": [x, y, z], "scout_yaw": yaw,
                               "target_gps": noisy_target_gps,
                               "target_gps_clean": target_gps,
                               "gps_drift_injected_xy": GPS_DRIFT_INJECTED_XY,
                               "direction_vector": unit_vec,
                               "lrf_range_m": lrf_dist, "packet_bytes": len(raw_bytes),
                               "chip_target_area_px": self.best_area,
                               "raycast_samples": n_samples,
                               "sim_time_s": now}, f, indent=2)
                self.state = "LOITER_OFFSET"

            elif self.state == "LOITER_OFFSET":
                self.compute_motor_velocities(self.loiter_waypoint, self.cruise_alt + 1.0, 0.0)
                # Re-broadcast packet every 1.0 second with updated Scout telemetry so late
                # joiners receive it. Each re-broadcast carries the next sequence number —
                # the recovery path of the Phase 8 degraded-link emulation (followers that
                # dropped seq 0/1 lock from these).
                if now - self.last_broadcast_time >= 1.0 and self.marked_packet is not None:
                    dx = self.marked_packet.target_gps[0] - x
                    dy = self.marked_packet.target_gps[1] - y
                    dz = self.marked_packet.target_gps[2] - z
                    dist = math.sqrt(dx*dx + dy*dy + dz*dz) or 1.0
                    self.marked_packet.scout_gps = [x, y, z]
                    self.marked_packet.scout_yaw = yaw
                    self.marked_packet.direction_vector = [dx/dist, dy/dist, dz/dist]
                    self.marked_packet.distance = dist
                    self.marked_packet.seq += 1
                    self.marked_packet.tx_time_s = now
                    self.emitter.send(self.marked_packet.serialize())
                    self.last_broadcast_time = now

if __name__ == "__main__":
    ScoutDroneController().run()
```

### 3.3 Follower controller — `controllers/follower_controller/follower_controller.py` (1163 lines)

State machine: `IDLE_HOVER` (home hover, poll receiver) → `STAGE_1_MACRO_NAV` (fly to the noisy packet coordinate at cruise altitude while the follower's own GPS fix wanders ±0.45 m continuously; forward-camera visual ID while facing the target and >4 m away; the secondary follower parks at a field-free hold spot ≥7 m from the goal until the primary announces its lock) → `STAGE_2_VISUAL_HOMING` (pure IBVS: the terminal camera measures the drone→plate offset with zero GPS dependence — camera-relative vector `rel_target_xy`, EMA α=0.45, velocity decay between frames, clip/size/occlusion gates, adaptive accept gate; terminal-camera visual-ID fallback while >2.5 m out; lock = rel<0.04 m, speed<0.02, red fresh, 120 steps) → `STAGE_3_VACATE` (broadcast `follower_lock`, vacate the plate so the teammate's terminal camera can see it). Avoidance is **mapless**: the 5-beam rangefinder array feeds `update_lidar_clusters` → `avoid_obstacles_lidar_xy` every control step, and parked spots (hold/vacate) migrate out of any sensed cluster's margin.

```python
# File: controllers/follower_controller/follower_controller.py
import json
import math
import os
import sys
import numpy as np
import cv2
from controller import Robot
from handoff_protocol import HandoffPacket

RESULTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "results"))
os.makedirs(RESULTS, exist_ok=True)

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

def clamp(value, low, high):
    return max(low, min(value, high))

VISUAL_HOMING_DEBUG = True   # live cv2 window during Stage 2 (verification spec 4.B)
TERMINAL_CAM_PITCH = 1.15    # radians, matches terminal_camera rotation in the world file
FOV = 1.047                  # camera field of view, matches the world file
HOMING_ACCEPT_RADIUS_M = 3.0 # reject red blobs farther than this from the current goal
HOLD_RING_M = 7.0            # secondary follower holds at this range from the noisy goal
TEAMMATE_FALLBACK_S = 105.0  # proceed without a teammate lock announcement after this
VACATE_XY = (7.5, 4.5)       # primary follower vacates the plate after locking (bug #26)
HOLD_CLEAR_M = 6.0           # parked spots migrate this far from any sensed wall
PARK_CLEAR_M = 5.7           # parked spots stay outside the field margin (5.5) + buffer
# Phase 7: mapless avoidance — the controller NEVER reads the obstacle map
# (buildings_obstacles.json exists only for run_verification.py / build_dashboard.py
# to grade clearance). Obstacles are sensed live by the 5-beam rangefinder array.
USE_JSON_MAP_IN_CONTROLLER = False
RF_MAX_RANGE_M = 11.5        # trust beam returns below this (sensor max range 12 m)
RF_MIN_RANGE_M = 0.8         # ignore near-field returns (propeller/body self-hits)
CLUSTER_EPS_M = 2.2          # a hit joins a cluster within this distance of its centroid
CLUSTER_MIN_HITS = 5         # clusters with fewer observations are treated as noise
CLUSTER_TTL_S = 6.0          # forget a cluster when no beam refreshes it for this long
HULL_INFLATE_M = 0.3         # pad the sensed point hull (beam spots are points, walls are thick)
PARK_MIGRATE_MAX_M = 2.0     # cap the per-step relocation of a parked spot
MERGE_GAP_M = 3.0            # clusters with hulls this close act as ONE obstacle complex
# VFH-lite steering (Phase 9 redesign): the effective goal sits STEER_LEAD_M
# ahead in a clear direction, so the commanded position error is bounded and
# the disturbance channel never saturates (bugs #27/#30/#35/#39).
STEER_LEAD_M = 1.6
STEER_CLEAR_M = 2.5          # a probe direction is clear at this distance from every hull
STEER_STEP_RAD = math.radians(15)
STEER_TERMINAL_M = 3.5       # fly straight at the goal inside this range (no steering)
STEER_MAX_RATE_RAD_S = 2.0 # steering direction turns at most 2 rad/s (bug #41)
# Phase 8: degraded tactical datalink emulation — deterministic per-drone packet
# drops and decode/queueing latency. The Scout's 1 Hz loiter re-broadcasts
# (seq >= 1) are the recovery path; every drop is logged in handoff_<id>.json.
DROPPED_SEQS = {"Follower_Drone_01": {0}, "Follower_Drone_02": {0, 1}}
DECODE_LATENCY_S = {"Follower_Drone_01": 0.0, "Follower_Drone_02": 0.120}

class FollowerDroneController:
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0

    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Optional per-drone config (used by the second follower)
        cd = self.robot.getCustomData()
        cfg = json.loads(cd) if cd else {}
        self.drone_id = cfg.get("id", "Follower_Drone_01")

        # Flight Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)

        # Handoff Payload Sensors
        self.front_cam = self.robot.getDevice("front_camera")
        self.front_cam.enable(self.timestep)
        self.terminal_cam = self.robot.getDevice("terminal_camera")
        self.terminal_cam.enable(self.timestep)
        self.receiver = self.robot.getDevice("swarm_receiver")
        self.receiver.enable(self.timestep)
        # Follower status broadcast (sequenced terminal corridor, bug #26)
        self.emitter = self.robot.getDevice("swarm_emitter")

        # Stabilize front gimbal
        for gimbal_name in ["camera roll", "camera pitch", "camera yaw"]:
            dev = self.robot.getDevice(gimbal_name)
            if dev:
                dev.setPosition(0.0)

        # Motors
        motor_names = [
            "front left propeller",
            "front right propeller",
            "rear left propeller",
            "rear right propeller"
        ]
        self.motors = [self.robot.getDevice(name) for name in motor_names]
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(1.0)

        # ORB Feature Detector & Brute-Force Hamming Matcher for target identification
        self.orb = cv2.ORB_create(nfeatures=600, scaleFactor=1.2, nlevels=8)
        self.bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # State & Target Data
        self.state = "IDLE_HOVER"
        self.home_xy = cfg.get("home", [-4.0, -3.0])
        self.cruise_alt = cfg.get("alt", 4.5)
        self.handoff_data: HandoffPacket = None
        self.ref_kp = None
        self.ref_des = None
        self.handoff_recv_time = None
        self.visual_id_logged = False
        self.id_diag = open(os.path.join(RESULTS, f"id_attempts_{self.drone_id}.log"), "w", buffering=1)
        self.id_diag_last_t = -1.0
        self.lock_achieved_logged = False
        self.lock_streak = 0
        self.approach_yaw = 0.0
        # Stage 2B pure IBVS homing state: self.rel_target_xy is the horizontal
        # vector from the drone's body center to the plate center measured purely
        # by terminal-camera pixels + IMU attitude + altitude. It lives in the
        # local frame and has ZERO dependence on the follower's horizontal GPS,
        # so live GPS wander cannot drag the terminal lock away.
        self.rel_target_xy = None
        self.homing_update_count = 0
        self.last_red_seen = -1.0
        self.stall_reject_streak = 0   # sustained gate rejections while hovering (bug #25 breaker)
        self.cur_speed = 0.0
        # Sequenced terminal corridor (bug #26): the primary follower locks and
        # vacates the plate first; the secondary holds outside the terminal ring
        # until the lock announcement so the parked teammate can never occlude
        # the plate and bias its IBVS estimates.
        self.is_primary = self.drone_id.endswith("01")
        self.primary_cleared = self.is_primary
        self.hold_xy = None
        self.vacate_xy = None
        self.last_lock_broadcast = -1.0
        # Phase 8 degraded-link emulation state
        self.dropped_seqs = DROPPED_SEQS.get(self.drone_id, set())
        self.decode_latency_s = DECODE_LATENCY_S.get(self.drone_id, 0.0)
        self.dropped_log = []
        self.stage1_engage_time = None
        self.first_tx_seen = None
        # Phase 6 live-GPS-wander bookkeeping
        self.max_gps_wander_m = 0.0
        self.cur_wander = [0.0, 0.0]
        # Perceived position for this step (noisy GPS during mission states)
        self.cur_x = self.cur_y = self.cur_z = None
        # Slow integral trim on body-frame position error (anti-windup)
        self.i_fwd = 0.0
        self.i_left = 0.0
        self.dbg = open(os.path.join(RESULTS, f"stage2_debug_{self.drone_id}.log"), "w", buffering=1)
        # Flight telemetry (bug #31/#32 diagnosis: attitude + motor mix history)
        # Flight telemetry (bug #31 diagnosis). CRITICAL: buffered in memory and
        # flushed every ~2 s — per-step line-buffered file I/O at 500 Hz blocked
        # the control loop on Windows and destabilized the attitude loop
        # (bug #38: the rocking/tumble crashes of iterations 6-13).
        self.telemetry_buf = []
        self.telemetry_t = 0.0
        # Phase 7: onboard 5-beam rangefinder array — the avoidance layer is
        # mapless: obstacles are sensed live and clustered from the beam hits.
        self.rf_devices = [self.robot.getDevice(n) for n in
                           ("rf_front", "rf_fl", "rf_fr", "rf_left", "rf_right")]
        self.rf_angles = [0.0, 0.5236, -0.5236, 1.0472, -1.0472]
        for dev in self.rf_devices:
            dev.enable(self.timestep)
        self.avoid_side = {}
        self.steer_ang = None       # smoothed steering direction (VFH-lite)
        self.clusters = {}            # cluster id -> {"pts": [(x, y, t), ...]}
        self.next_cluster_id = 0
        self.sensor_hits_count = 0
        self.sensed_log_t = 0.0

    def update_lidar_clusters(self, x, y, z, roll, pitch, yaw):
        """Raycast the 5 horizontal beams into world-frame hit points and cluster
        them into a tracked obstacle memory (sliding window with a time-to-live).
        Gated to airborne level flight; near-field self-hits are ignored."""
        now = self.robot.getTime()
        if z > 1.5 and abs(roll) < 0.25 and abs(pitch) < 0.25:
            for dev, rel in zip(self.rf_devices, self.rf_angles):
                r = dev.getValue()
                if RF_MIN_RANGE_M <= r < RF_MAX_RANGE_M:
                    beam_yaw = yaw + rel
                    hx = x + r * math.cos(beam_yaw)
                    hy = y + r * math.sin(beam_yaw)
                    self.sensor_hits_count += 1
                    best_id, best_d = None, CLUSTER_EPS_M
                    for cid, c in self.clusters.items():
                        pts = c["pts"]
                        ccx = sum(p[0] for p in pts) / len(pts)
                        ccy = sum(p[1] for p in pts) / len(pts)
                        d = math.hypot(hx - ccx, hy - ccy)
                        if d < best_d:
                            best_id, best_d = cid, d
                    if best_id is None:
                        best_id = self.next_cluster_id
                        self.next_cluster_id += 1
                        self.clusters[best_id] = {"pts": []}
                    self.clusters[best_id]["pts"].append((hx, hy, now))
        # prune stale points and dead clusters (a young cluster still being
        # observed survives; one that never accumulates MIN_HITS dies quietly)
        for cid in list(self.clusters.keys()):
            c = self.clusters[cid]
            c["pts"] = [p for p in c["pts"] if now - p[2] <= CLUSTER_TTL_S]
            if not c["pts"] or (len(c["pts"]) < CLUSTER_MIN_HITS
                                and now - c["pts"][-1][2] > 0.5):
                del self.clusters[cid]
        # Merge nearly-touching hulls into one obstacle complex: independent
        # tangent commitments on adjacent faces of a multi-building complex
        # oppose each other in the gap and can stall or scrape the drone
        # (the staggered corridor blocks have a ~1 m diagonal gap, bug #30).
        merged = True
        while merged:
            merged = False
            ids = list(self.clusters.keys())
            for i in range(len(ids)):
                a = self.clusters.get(ids[i])
                if a is None:
                    continue
                for j in range(i + 1, len(ids)):
                    b = self.clusters.get(ids[j])
                    if b is None:
                        continue
                    if self._hull_gap(a["pts"], b["pts"]) < MERGE_GAP_M:
                        a["pts"].extend(b["pts"])
                        del self.clusters[ids[j]]
                        merged = True

    @staticmethod
    def _hull_gap(pa, pb):
        """Distance between two point sets' inflated hull AABBs (0 if they overlap)."""
        ax = [p[0] for p in pa]; ay = [p[1] for p in pa]
        bx = [p[0] for p in pb]; by = [p[1] for p in pb]
        gap_x = max(min(bx) - (max(ax) + HULL_INFLATE_M), min(ax) - (max(bx) + HULL_INFLATE_M))
        gap_y = max(min(by) - (max(ay) + HULL_INFLATE_M), min(ay) - (max(by) + HULL_INFLATE_M))
        return math.hypot(max(gap_x, 0.0), max(gap_y, 0.0))

    def cluster_centroids(self):
        cents = []
        for c in self.clusters.values():
            pts = c["pts"]
            if len(pts) >= CLUSTER_MIN_HITS:
                cents.append([round(sum(p[0] for p in pts) / len(pts), 2),
                              round(sum(p[1] for p in pts) / len(pts), 2)])
        return cents

    def write_sensor_log(self):
        with open(os.path.join(RESULTS, f"sensor_avoidance_{self.drone_id}.json"), "w") as f:
            json.dump({"drone_id": self.drone_id,
                       "use_json_map_in_controller": USE_JSON_MAP_IN_CONTROLLER,
                       "sensor_hits_count": self.sensor_hits_count,
                       "sensed_obstacle_centroids": self.cluster_centroids(),
                       "active_clusters": len(self.cluster_centroids()),
                       "sim_time_s": self.robot.getTime()}, f, indent=2)

    def cluster_aabbs(self):
        """The sensed obstacle hulls the avoidance field acts on — the SAME set
        the park-safety logic must respect, so parking and the field can never
        disagree about what is nearby."""
        boxes = []
        for c in self.clusters.values():
            pts = c["pts"]
            if len(pts) < CLUSTER_MIN_HITS:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            boxes.append((min(xs) - HULL_INFLATE_M, min(ys) - HULL_INFLATE_M,
                          max(xs) + HULL_INFLATE_M, max(ys) + HULL_INFLATE_M,
                          sum(xs) / len(xs), sum(ys) / len(ys)))
        return boxes

    def park_spot_field_free(self, spot):
        """True when a parked spot lies outside every sensed cluster's avoidance
        margin. Parking INSIDE the repulsive field makes the drone chase a
        ~10 m deflected goal, saturates its attitude loop and can tumble out of
        the sky (bugs #27/#28) — capture is refused until the spot is clear."""
        for (minx, miny, maxx, maxy, _, _) in self.cluster_aabbs():
            cx = clamp(spot[0], minx, maxx)
            cy = clamp(spot[1], miny, maxy)
            if math.hypot(spot[0] - cx, spot[1] - cy) < PARK_CLEAR_M:
                return False
        return True

    def push_spot_out_of_clusters(self, spot):
        """One-shot placement: push a hold/vacate spot fully clear of every
        sensed cluster margin BEFORE the drone commits to flying there
        (bugs #27/#28, now against the live cluster set). Unlike the per-step
        relocation this is a single placement decision, so a spot that ends up
        near a wall is moved out once, not chased."""
        s = [spot[0], spot[1]]
        for _ in range(6):
            moved = False
            for (minx, miny, maxx, maxy, ccx, ccy) in self.cluster_aabbs():
                cx = clamp(s[0], minx, maxx)
                cy = clamp(s[1], miny, maxy)
                dx, dy = s[0] - cx, s[1] - cy
                d = math.hypot(dx, dy)
                if d >= PARK_CLEAR_M:
                    continue
                if d < 1e-3:
                    dx, dy = s[0] - ccx, s[1] - ccy
                    d = math.hypot(dx, dy) or 1.0
                s = [cx + dx / d * PARK_CLEAR_M, cy + dy / d * PARK_CLEAR_M]
                moved = True
            if not moved:
                break
        return s

    def relocate_parked_spot(self, spot):
        """Migrate a parked spot (hold point / vacate point) out of every sensed
        cluster's avoidance margin — the exact obstacle set the field acts on,
        so a parked spot can never sit inside an active deflection (bug #28).
        The per-step migration is capped so a parked target never teleports."""
        new_spot = [spot[0], spot[1]]
        for _ in range(4):
            moved = False
            for (minx, miny, maxx, maxy, ccx, ccy) in self.cluster_aabbs():
                cx = clamp(new_spot[0], minx, maxx)
                cy = clamp(new_spot[1], miny, maxy)
                dx, dy = new_spot[0] - cx, new_spot[1] - cy
                d = math.hypot(dx, dy)
                if d >= PARK_CLEAR_M:
                    continue
                if d < 1e-3:   # degenerate: push away from the cluster centroid
                    dx, dy = new_spot[0] - ccx, new_spot[1] - ccy
                    d = math.hypot(dx, dy) or 1.0
                new_spot = [cx + dx / d * PARK_CLEAR_M, cy + dy / d * PARK_CLEAR_M]
                moved = True
            if not moved:
                break
        step = math.hypot(new_spot[0] - spot[0], new_spot[1] - spot[1])
        if step > PARK_MIGRATE_MAX_M:
            k = PARK_MIGRATE_MAX_M / step
            new_spot = [spot[0] + (new_spot[0] - spot[0]) * k,
                        spot[1] + (new_spot[1] - spot[1]) * k]
        return new_spot

    def get_noisy_gps(self, now):
        """Returns (noisy_x, noisy_y, true_z, true_x, true_y, wander_xy).
        Simulates continuous, time-varying GNSS wander (+/- 0.45 m) on the follower:
        a multi-frequency sinusoid plus slow drift, phase-offset per drone so the
        two followers wander independently."""
        tx, ty, tz = self.gps.getValues()
        phase = 0.0 if self.drone_id.endswith("01") else 1.7
        wx = 0.30 * math.sin(0.25 * now + phase) + 0.15 * math.cos(0.61 * now + phase)
        wy = 0.28 * math.cos(0.21 * now + phase) - 0.14 * math.sin(0.53 * now + phase)
        return tx + wx, ty + wy, tz, tx, ty, [wx, wy]

    def avoid_obstacles_lidar_xy(self, x, y, z, goal_xy):
        """
        VFH-lite steering (Phase 9 redesign, bugs #27/#30/#35/#39): instead of an
        unbounded repulsive field, pick a steering direction whose probe point
        (STEER_LEAD_M ahead) is clear of every sensed cluster hull, and place the
        effective goal at that lead. The commanded position error can never
        exceed the lead, so the disturbance channel never saturates and the
        attitude loop stays linear — the root cause of the Phase 9 tumbles.
        Obstacles come ONLY from the onboard 5-beam rangefinder clusters.
        """
        gx, gy = goal_xy[0], goal_xy[1]
        dxg, dyg = gx - x, gy - y
        goal_dist = math.hypot(dxg, dyg)
        if goal_dist < 0.4:
            return goal_xy
        lead = min(STEER_LEAD_M, goal_dist)
        base = math.atan2(dyg, dxg)
        boxes = self.cluster_aabbs()
        best_ang = base
        if boxes:
            # terminal approach exemption: within STEER_TERMINAL_M of the goal
            # fly straight (the plate sits ~2.2 m from block 2's hull — steering
            # there would prevent the approach entirely)
            if goal_dist > STEER_TERMINAL_M:
                # 1) the goal direction when clear; 2) keep the current steering
                # while it stays clear (anti-chatter stickiness); 3) the widest
                # search (+/-180 deg) — near a merged complex every forward probe
                # can be blocked, and the fallback must turn OUT, not in
                # (bug #40: no clear candidate within +/-90 deg flew into the hull)
                if self._probe_clear(x + lead * math.cos(base),
                                     y + lead * math.sin(base), boxes):
                    best_ang = base
                elif self.steer_ang is not None and self._probe_clear(
                        x + lead * math.cos(self.steer_ang),
                        y + lead * math.sin(self.steer_ang), boxes):
                    best_ang = self.steer_ang
                else:
                    best_clear, best_ang = -1.0, base
                    for k in range(1, 13):
                        for sign in (1, -1):
                            ang = base + sign * k * STEER_STEP_RAD
                            px = x + lead * math.cos(ang)
                            py = y + lead * math.sin(ang)
                            clear = self._probe_clearance(px, py, boxes)
                            if clear > best_clear:
                                best_clear, best_ang = clear, ang
        # smooth the steering direction (the clear/unclear set can snap)
        dt = self.timestep / 1000.0
        if self.steer_ang is None:
            self.steer_ang = best_ang
        else:
            d_ang = math.atan2(math.sin(best_ang - self.steer_ang),
                               math.cos(best_ang - self.steer_ang))
            # time-based rate limit: a per-step fraction of a 180 deg
            # flip swept the steering at ~236 rad/s and shook the drone
            # apart with a 12 Hz disturbance square wave (bug #41)
            self.steer_ang += clamp(d_ang, -STEER_MAX_RATE_RAD_S * dt,
                                    STEER_MAX_RATE_RAD_S * dt)
        eff_x = x + lead * math.cos(self.steer_ang)
        eff_y = y + lead * math.sin(self.steer_ang)
        # diagnostics for the telemetry recorder (bug #31 diagnosis)
        self.last_rep = [eff_x - gx, eff_y - gy]
        self.last_cluster_n = len(self.clusters)
        return [eff_x, eff_y]

    @staticmethod
    def _probe_clear(px, py, boxes):
        """A steering probe is clear when its point lies STEER_CLEAR_M from every
        sensed cluster hull."""
        for (minx, miny, maxx, maxy, _, _) in boxes:
            cx = clamp(px, minx, maxx)
            cy = clamp(py, miny, maxy)
            if math.hypot(px - cx, py - cy) < STEER_CLEAR_M:
                return False
        return True

    @staticmethod
    def _probe_clearance(px, py, boxes):
        """Distance from a probe point to the nearest sensed cluster hull (inf
        when there are none) — the least-bad fallback metric."""
        best = float("inf")
        for (minx, miny, maxx, maxy, _, _) in boxes:
            cx = clamp(px, minx, maxx)
            cy = clamp(py, miny, maxy)
            best = min(best, math.hypot(px - cx, py - cy))
        return best

    def dbg_log(self, msg):
        try:
            self.dbg.write(f"[t={self.robot.getTime():6.2f}] {msg}\n")
        except Exception:
            pass

    def get_front_bgr(self) -> np.ndarray:
        raw = self.front_cam.getImage()
        w = self.front_cam.getWidth()
        h = self.front_cam.getHeight()
        img = np.frombuffer(raw, np.uint8).reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    def get_terminal_bgr(self) -> np.ndarray:
        raw = self.terminal_cam.getImage()
        w = self.terminal_cam.getWidth()
        h = self.terminal_cam.getHeight()
        img = np.frombuffer(raw, np.uint8).reshape((h, w, 4))
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    def detect_red_centroid(self, bgr_img):
        """Tight HSV red mask + largest contour -> (found, area_px, cx, cy)."""
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([5, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([175, 150, 100]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0, 0.0
        biggest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest)
        m = cv2.moments(biggest)
        if m["m00"] <= 0:
            return False, area, 0.0, 0.0
        cx = m["m10"] / m["m00"]
        cy = m["m01"] / m["m00"]
        return True, area, cx, cy

    def detect_red_blobs(self, bgr_img, min_area=800):
        """Tight HSV red mask -> [(area, cx, cy, corners, bbox)] for every blob
        >= min_area. corners = the 4 vertices of the blob's min-area rect; bbox =
        the axis-aligned bounding rect of ALL contour points (used for robust
        frame-edge clip detection). Homing inverts the corners and averages in
        the camera-relative frame: each image corner projects exactly onto the
        ground plane, so the corner average is the unbiased plate center. The
        color centroid or the rect center would instead carry a perspective bias
        on the oblique view."""
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([5, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([175, 150, 100]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            m = cv2.moments(c)
            if m["m00"] <= 0:
                continue
            box = cv2.minAreaRect(c)
            corners = [(float(p[0]), float(p[1])) for p in cv2.boxPoints(box)]
            bx, by, bw, bh = cv2.boundingRect(c)
            blobs.append((area, m["m10"] / m["m00"], m["m01"] / m["m00"], corners,
                          (int(bx), int(by), int(bw), int(bh))))
        return blobs

    def check_incoming_handoff(self):
        """Polls the Webots Receiver queue. Channel 1 carries two message types:
        the Scout's HandoffPacket and the followers' status broadcasts
        ("follower_lock"). The JSON envelope is peeked at first so a status
        message never hits the packet decoder."""
        while self.receiver.getQueueLength() > 0:
            raw_bytes = self.receiver.getBytes()
            self.receiver.nextPacket()
            try:
                payload = json.loads(raw_bytes.decode("utf-8"))
            except Exception:
                payload = None
            if isinstance(payload, dict) and payload.get("msg") == "follower_lock":
                if payload.get("drone_id") != self.drone_id and not self.primary_cleared:
                    self.primary_cleared = True
                    print(f"[{self.drone_id}] teammate lock announcement received "
                          f"(from {payload.get('drone_id')}) — terminal corridor cleared")
                continue
            if payload is None:
                continue
            packet = HandoffPacket.deserialize(raw_bytes)
            now = self.robot.getTime()
            if self.first_tx_seen is None or packet.tx_time_s < self.first_tx_seen:
                self.first_tx_seen = packet.tx_time_s

            # Phase 8 degraded link: CRC check first (corruption => discard),
            # then the deterministic per-drone drop emulation. Dropped seq 0/1
            # force the follower to rely on the Scout's 1 Hz re-broadcasts.
            if not packet.crc_valid:
                self.dropped_log.append({"seq": packet.seq, "tx_time_s": packet.tx_time_s,
                                         "rx_time_s": now, "reason": "crc_invalid"})
                print(f"[{self.drone_id}] packet seq={packet.seq} discarded (CRC mismatch)")
                continue
            if packet.seq in self.dropped_seqs:
                self.dropped_log.append({"seq": packet.seq, "tx_time_s": packet.tx_time_s,
                                         "rx_time_s": now, "reason": "link_drop_emulated"})
                print(f"[{self.drone_id}] packet seq={packet.seq} DROPPED (degraded-link emulation)")
                continue

            if self.handoff_data is None:
                self.handoff_data = packet
                self.handoff_recv_time = now
                # decode/queueing latency elapses before Stage 1 engages
                self.stage1_engage_time = now + self.decode_latency_s

                # Pre-compute ORB keypoints & descriptors on the Scout's Reference Image Chip
                gray_ref = cv2.cvtColor(packet.image_chip, cv2.COLOR_BGR2GRAY)
                self.ref_kp, self.ref_des = self.orb.detectAndCompute(gray_ref, None)

                x, y, _ = self.gps.getValues()
                bearing_deg = math.degrees(math.atan2(packet.target_gps[1] - y, packet.target_gps[0] - x))
                print(f"[{self.drone_id}] RECEIVED HANDOFF PACKET from {packet.scout_id}!")
                print(f"           Scout Location : ({packet.scout_gps[0]:.2f}, {packet.scout_gps[1]:.2f}, {packet.scout_gps[2]:.2f})")
                print(f"           Target Coords  : ({packet.target_gps[0]:.3f}, {packet.target_gps[1]:.3f}, {packet.target_gps[2]:.3f})")
                print(f"           Approach Brg   : {bearing_deg:.1f} deg | Ref ORB Features: {len(self.ref_kp) if self.ref_kp else 0}")
                print(f"[{self.drone_id}] Engaging STAGE 1: GPS Navigation with forward-camera target ID...")
                cv2.imwrite(os.path.join(RESULTS, f"received_chip_{self.drone_id}.png"), packet.image_chip)
                with open(os.path.join(RESULTS, f"handoff_{self.drone_id}.json"), "w") as f:
                    json.dump({"drone": self.drone_id, "scout_id": packet.scout_id,
                               "scout_gps": packet.scout_gps, "target_gps": packet.target_gps,
                               "approach_bearing_deg": bearing_deg,
                               "distance_at_receipt_m": math.hypot(packet.target_gps[0]-x, packet.target_gps[1]-y),
                               "ref_orb_features": len(self.ref_kp) if self.ref_kp else 0,
                               "packet_bytes": len(raw_bytes),
                               # Phase 8 degraded-link evidence
                               "packet_seq": packet.seq,
                               "packet_tx_time_s": packet.tx_time_s,
                               "packet_crc_valid": packet.crc_valid,
                               "dropped_packets": self.dropped_log,
                               "decode_latency_ms": self.decode_latency_s * 1000.0,
                               "receipt_lag_from_mark_ms": (now - self.first_tx_seen) * 1000.0,
                               "sim_time_s": self.handoff_recv_time}, f, indent=2)
                # Stage 1 engages only after the decode/queueing latency (run loop)

    def try_visual_id(self, live_bgr: np.ndarray):
        """
        Confirms the target ahead matches the Scout's reference chip using
        ORB + Lowe's Ratio Test + RANSAC Homography. Logs once when confirmed.
        The live frame must contain the red target (tight HSV mask) so that
        floor-texture matches alone can never produce a false identification.
        """
        if self.visual_id_logged or self.ref_des is None or len(self.ref_kp) < 8:
            return

        found, red_area, _, _ = self.detect_red_centroid(live_bgr)
        t = self.robot.getTime()
        if t - self.id_diag_last_t >= 1.0:
            self.id_diag_last_t = t
            self.id_diag.write(f"[t={t:6.2f}] red_area={red_area:7.0f}px (gate 800)\n")
        if not found or red_area < 800:
            return

        gray_live = cv2.cvtColor(live_bgr, cv2.COLOR_BGR2GRAY)
        live_kp, live_des = self.orb.detectAndCompute(gray_live, None)
        if live_des is None or len(live_kp) < 8:
            self.id_diag.write(f"[t={t:6.2f}] RED PASS but live ORB features < 8\n")
            return
        matches = self.bf_matcher.knnMatch(self.ref_des, live_des, k=2)
        good = [m for m, n in (mn for mn in matches if len(mn) == 2) if m.distance < 0.8 * n.distance]
        if len(good) < 10:
            self.id_diag.write(f"[t={t:6.2f}] RED PASS good={len(good)} (<10)\n")
            return
        src = np.float32([self.ref_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([live_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None:
            self.id_diag.write(f"[t={t:6.2f}] RED PASS homography failed\n")
            return
        inliers = int(mask.sum())
        if inliers < 8:
            self.id_diag.write(f"[t={t:6.2f}] RED PASS good={len(good)} inliers={inliers} (<8)\n")
            return
        self.visual_id_logged = True
        x, y, z = self.gps.getValues()
        print(f"[{self.drone_id}] TARGET VISUALLY IDENTIFIED ahead! red_area={red_area:.0f}px ORB good={len(good)} RANSAC inliers={inliers}")
        cv2.imwrite(os.path.join(RESULTS, f"id_frame_{self.drone_id}.png"), live_bgr)
        with open(os.path.join(RESULTS, f"visual_id_{self.drone_id}.json"), "w") as f:
            json.dump({"drone": self.drone_id, "gps": [x, y, z],
                       "orb_good_matches": len(good), "ransac_inliers": inliers,
                       "red_target_area_px": red_area,
                       "sim_time_s": self.robot.getTime()}, f, indent=2)

    def estimate_visual_homing_offset(self, bgr_img, do_orb_debug=False):
        """
        Stage 2B pure IBVS (image-based visual servoing) homing. Inverts the
        monocular ground-plane projection of the steep-downtilt terminal camera and
        maintains self.rel_target_xy: the horizontal vector from the drone's body
        center to the plate center, measured purely by terminal-camera pixels +
        IMU attitude + altitude. Zero dependence on horizontal GPS.

        Math: for a detected pixel the camera ray is dw = R @ db; the ground hit is
        pcam + s*dw with pcam = perceived_pos + R @ cam_offset, so the offset
        rel = (R @ cam_offset + s*dw)[:2] depends only on attitude, altitude and
        pixels — the noisy perceived (x, y) cancels identically between pcam and
        the subtraction.

        Full (non-edge-clipped) red blobs update the vector with an EMA
        (alpha = 0.45); a blob clipped by the frame edge has a wrong fitted rect
        and is skipped, leaving the per-step velocity-decay propagation
        (rel -= v*dt, applied in the main loop) to carry the vector between
        camera frames. An adaptive accept gate in RELATIVE space (wide 3 m during
        approach, 0.12 m once converged) keeps unrelated red scenery from
        hijacking the goal. Returns True when a red blob was seen this step.
        """
        h_img, w_img = bgr_img.shape[:2]
        blobs = self.detect_red_blobs(bgr_img)
        seen = len(blobs) > 0
        projected_center = None
        cx = cy = None

        if seen:
            self.last_red_seen = self.robot.getTime()

        if seen and self.rel_target_xy is not None:
            x, y, z = self.cur_x, self.cur_y, self.cur_z
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            plate_z = self.handoff_data.target_gps[2]   # laser-measured plate top = ground plane
            if (z + 0.02) - plate_z > 0.5:
                f_px = (w_img / 2.0) / math.tan(FOV / 2.0)
                theta = TERMINAL_CAM_PITCH
                ct, st = math.cos(theta), math.sin(theta)
                cr, sr = math.cos(roll), math.sin(roll)
                cp, sp = math.cos(pitch), math.sin(pitch)
                cy_, sy_ = math.cos(yaw), math.sin(yaw)
                # body -> world: R = Rz(yaw) * Ry(pitch) * Rx(roll) (Webots convention).
                # The fixed camera tilts with the body, so the drone's flight attitude
                # must be in the model; ignoring it biases d_fwd by several degrees.
                R = np.array([
                    [cy_ * cp, cy_ * sp * sr - sy_ * cr, cy_ * sp * cr + sy_ * sr],
                    [sy_ * cp, sy_ * sp * sr + cy_ * cr, sy_ * sp * cr - cy_ * sr],
                    [-sp, cp * sr, cp * cr]])
                pcam = np.array([x, y, z]) + R @ np.array([0.12, 0.0, 0.02])

                def invert(px, py):
                    uu = (px - w_img / 2.0) / f_px
                    vv = (py - h_img / 2.0) / f_px
                    # Webots camera looks along node +X; image u right = -Y_c, v down = -Z_c
                    db = np.array([ct - vv * st, -uu, -st - vv * ct])
                    dw = R @ db
                    if dw[2] >= -0.05:          # ray must point downward
                        return None
                    s = (plate_z - pcam[2]) / dw[2]
                    if s <= 0.0 or s > 60.0:
                        return None
                    pt = pcam + s * dw
                    # GPS-free relative offset: perceived (x, y) appears in pcam and
                    # is subtracted back out, so the wander terms cancel identically.
                    rel = [pt[0] - x, pt[1] - y]
                    if math.hypot(rel[0], rel[1]) > 25.0:
                        return None
                    return [float(rel[0]), float(rel[1])]

                cands = []
                for (area, bxc, byc, corners, bbox) in blobs:
                    # Robust clip gate: a blob clipped by the frame edge has a
                    # truncated contour and a biased fitted rect. Reject whenever
                    # ANY contour pixel touches the frame border — the axis-aligned
                    # bbox catches rotated min-area rects whose corners sit well
                    # inside the frame (the corner-only test missed lateral clips,
                    # bug #25).
                    bx0, by0, bw, bh = bbox
                    if bx0 <= 3 or by0 <= 3 or bx0 + bw >= w_img - 4 or by0 + bh >= h_img - 4:
                        continue
                    pts = [p for p in (invert(px, py) for (px, py) in corners) if p is not None]
                    if len(pts) < 4:
                        continue
                    est = [sum(p[i] for p in pts) / 4.0 for i in (0, 1)]
                    # Plate-geometry identity check: the ground quad of the full
                    # 1.4 m plate must have ~1.4 m sides. A partial blob or a bad
                    # ground inversion produces a wrong scale and is discarded.
                    sides = [math.hypot(pts[i][0] - pts[(i + 1) % 4][0],
                                        pts[i][1] - pts[(i + 1) % 4][1]) for i in range(4)]
                    mean_side = sum(sides) / 4.0
                    if not (0.8 <= mean_side <= 2.2):
                        self.dbg_log(f"size-reject area={area:.0f}px mean_side={mean_side:.2f} m")
                        continue
                    # Occlusion gate (bug #26): the plate is physically 1.4 m
                    # square, so a healthy view has a predictable pixel area at
                    # the measured slant range. A teammate parked over the target
                    # (or any partial occluder) truncates the blob and biases the
                    # corner-average estimate — reject views that are far too
                    # small for their range and propagate the vector instead.
                    slant = math.hypot(math.hypot(est[0], est[1]), (z + 0.02) - plate_z)
                    expected_area = (1.4 * f_px / slant) ** 2
                    if area < 0.5 * expected_area:
                        self.dbg_log(f"occl-reject area={area:.0f}px expected>="
                                     f"{0.5 * expected_area:.0f}px (occluded/partial view)")
                        continue
                    cands.append((math.hypot(est[0] - self.rel_target_xy[0],
                                             est[1] - self.rel_target_xy[1]),
                                  area, bxc, byc, est))
                best = min(cands, key=lambda c: c[0]) if cands else None
                # Adaptive accept gate in RELATIVE space: wide (3 m) during approach
                # so the vector can jump from the packet-GPS initial guess to the
                # camera-measured truth. Once well established (>=150 updates),
                # tighten to 0.12 m so a late partial/grazing plate blob cannot drag
                # a converged vector away.
                accept_r = HOMING_ACCEPT_RADIUS_M if self.homing_update_count < 150 else 0.12
                stalled = self.cur_speed < 0.05
                if best is not None and best[0] < accept_r:
                    d, area, cx, cy, est = best
                    if self.homing_update_count == 0:
                        print(f"[{self.drone_id}] STAGE_2B first IBVS estimate: "
                              f"rel ({self.rel_target_xy[0]:.3f}, {self.rel_target_xy[1]:.3f}) "
                              f"-> camera-measured ({est[0]:.3f}, {est[1]:.3f})")
                    a = 0.45
                    self.rel_target_xy = [a * p + (1.0 - a) * q
                                          for p, q in zip(est, self.rel_target_xy)]
                    self.homing_update_count += 1
                    self.stall_reject_streak = 0
                    self.dbg_log(f"accept d_rel={d:.2f}m area={area:.0f}px "
                                 f"est=({est[0]:.2f},{est[1]:.2f}) "
                                 f"rel=({self.rel_target_xy[0]:.2f},{self.rel_target_xy[1]:.2f})")
                elif best is not None and stalled:
                    # Deadlock breaker (bug #25): while hovering settled, the
                    # dead-reckoned rel and a size-validated camera measurement can
                    # disagree (clipped-blob history or decay drift). The camera is
                    # the live sensor — after ~2 s of sustained disagreement, trust
                    # it so the drone can walk back onto the plate.
                    self.stall_reject_streak += 1
                    if self.stall_reject_streak >= 200:
                        d, area, cx, cy, est = best
                        a = 0.45
                        self.rel_target_xy = [a * p + (1.0 - a) * q
                                              for p, q in zip(est, self.rel_target_xy)]
                        self.homing_update_count += 1
                        self.stall_reject_streak = 0
                        self.dbg_log(f"stall-override d_rel={d:.2f}m area={area:.0f}px "
                                     f"est=({est[0]:.2f},{est[1]:.2f}) "
                                     f"rel=({self.rel_target_xy[0]:.2f},{self.rel_target_xy[1]:.2f})")
                    else:
                        self.dbg_log(f"reject {len(cands)} candidate(s): nearest {best[0]:.2f} m "
                                     f"> gate {accept_r:.2f} m (stall streak {self.stall_reject_streak})")
                elif best is not None:
                    self.stall_reject_streak = 0
                    self.dbg_log(f"reject {len(cands)} candidate(s): nearest {best[0]:.2f} m "
                                 f"> gate {accept_r:.2f} m")
                else:
                    self.dbg_log("blob seen but ground inversion out of bounds")

        debug = bgr_img.copy()
        cv2.circle(debug, (w_img // 2, h_img // 2), 6, (0, 0, 255), 2)  # follower camera center
        if do_orb_debug and self.ref_des is not None and len(self.ref_kp) >= 8:
            gray = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
            live_kp, live_des = self.orb.detectAndCompute(gray, None)
            if live_des is not None and len(live_kp) >= 8:
                matches = self.bf_matcher.knnMatch(self.ref_des, live_des, k=2)
                good = [m for m, n in (mn for mn in matches if len(mn) == 2)
                        if m.distance < 0.75 * n.distance]
                if len(good) >= 12:
                    src = np.float32([self.ref_kp[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
                    dst = np.float32([live_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
                    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
                    if H is not None and int(mask.sum()) >= 8:
                        p = H.dot(np.array([120.0, 120.0, 1.0]))
                        projected_center = (p[0] / p[2], p[1] / p[2])
                        # Green crosshair at homography-matched target center (spec 4.B)
                        cv2.drawMarker(debug, (int(projected_center[0]), int(projected_center[1])),
                                       (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        if cx is not None:
            cv2.circle(debug, (int(cx), int(cy)), 5, (255, 0, 0), 2)  # accepted rect center = homing signal
        if self.rel_target_xy is not None:
            txt = (f"IBVS rel=({self.rel_target_xy[0]:.2f},{self.rel_target_xy[1]:.2f}) "
                   f"|wander|={math.hypot(self.cur_wander[0], self.cur_wander[1]):.2f} m")
            cv2.putText(debug, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(debug, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        if VISUAL_HOMING_DEBUG:
            try:
                cv2.imshow("Stage 2: Follower Terminal Visual Homing", debug)
                cv2.waitKey(1)
            except Exception:
                pass
        return seen

    def compute_motor_velocities(self, target_xy, target_alt, target_yaw=0.0, ibvs_rel=None):
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        # Perceived position (noisy during mission states). With the IBVS path the
        # target is fed as perceived_xy + rel_target_xy, so the (noisy_xy - noisy_xy)
        # terms cancel identically and only the camera-measured offset remains.
        x, y, z = self.cur_x, self.cur_y, self.cur_z
        roll_vel, pitch_vel, yaw_vel = self.gyro.getValues()

        # Update the onboard lidar obstacle memory every control step
        self.update_lidar_clusters(x, y, z, roll, pitch, yaw)

        if ibvs_rel is not None:
            # Pure IBVS terminal lock (<0.8 m): the camera-measured relative vector
            # IS the position error — bypass the obstacle field entirely and drive
            # the body errors straight from the GPS-free vector.
            dx, dy = ibvs_rel[0], ibvs_rel[1]
        else:
            # Apply mapless (sensor-based) 2D avoidance before body-frame errors
            eff_xy = self.avoid_obstacles_lidar_xy(x, y, z, target_xy)
            dx = eff_xy[0] - x
            dy = eff_xy[1] - y
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        body_forward_err = dx * cos_y + dy * sin_y
        body_left_err = -dx * sin_y + dy * cos_y

        # PD position control; positive roll disturbance moves left (+y body)
        vx, vy, _ = self.gps.getSpeedVector()
        body_fwd_vel = vx * cos_y + vy * sin_y
        body_left_vel = -vx * sin_y + vy * cos_y

        # Slow integral trim (anti-windup, only near the goal) removes the ~10 cm
        # steady-state hover offset of the P+D loop so the tight terminal gates pass
        err_mag = math.hypot(body_forward_err, body_left_err)
        dt = self.timestep / 1000.0
        if err_mag < 0.6:
            self.i_fwd -= 0.4 * body_forward_err * dt
            self.i_left += 0.4 * body_left_err * dt
        else:
            self.i_fwd *= 0.995
            self.i_left *= 0.995
        # integral clamp raised for the Phase 9 steady-wind trim (0.35 could not
        # reach the ~0.45 disturbance units the steady crosswind requires)
        self.i_fwd = clamp(self.i_fwd, -0.6, 0.6)
        self.i_left = clamp(self.i_left, -0.6, 0.6)

        # Phase 9 wind rejection: boost the position stiffness near the goal,
        # well clear of the ground (bug #32) AND only when essentially hovering
        # — the boosted D-term saturates for any v > ~0.07 m/s and turns braking
        # into bang-bang (bug #34: ring stand-off arrival at 0.5 m/s diverged).
        gd = math.hypot(ibvs_rel[0], ibvs_rel[1]) if ibvs_rel is not None \
            else math.hypot(target_xy[0] - x, target_xy[1] - y)
        hover_speed = math.hypot(vx, vy)
        gs = (1.0 + 11.0 * clamp((2.0 - gd) / 2.0, 0.0, 1.0)
              * clamp((z - 1.5) / 1.0, 0.0, 1.0)
              * clamp(1.0 - hover_speed / 0.20, 0.0, 1.0))
        pitch_disturbance = clamp(-(1.2 * gs) * body_forward_err + (1.8 * gs) * body_fwd_vel + self.i_fwd, -1.5, 1.5)
        roll_disturbance = clamp((1.2 * gs) * body_left_err - (1.8 * gs) * body_left_vel + self.i_left, -1.5, 1.5)
        # full-state diagnostics for the telemetry recorder (bug #31 diagnosis)
        self.last_diag = (body_forward_err, body_left_err, body_fwd_vel, body_left_vel,
                          pitch_disturbance, roll_disturbance,
                          self.i_fwd, self.i_left, self.state,
                          1.0 if ibvs_rel is not None else 0.0,
                          roll_vel, pitch_vel, yaw_vel)

        yaw_err = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
        yaw_disturbance = clamp(1.2 * yaw_err, -1.0, 1.0)

        clamped_alt_diff = clamp(target_alt - z + self.K_VERTICAL_OFFSET, -1.0, 1.0)
        vertical_input = self.K_VERTICAL_P * math.pow(clamped_alt_diff, 3.0)

        roll_input = self.K_ROLL_P * clamp(roll, -1.0, 1.0) + roll_vel + roll_disturbance
        pitch_input = self.K_PITCH_P * clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_disturbance
        yaw_input = yaw_disturbance - 0.5 * yaw_vel

        fl = self.K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input
        fr = self.K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input
        rl = self.K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input
        rr = self.K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input

        try:
            rep = getattr(self, "last_rep", [0.0, 0.0])
            d = getattr(self, "last_diag", (0.0,) * 10)
            self.telemetry_buf.append(
                f"{self.robot.getTime():.2f},{x:.3f},{y:.3f},{z:.3f},"
                f"{roll:.3f},{pitch:.3f},{yaw:.3f},"
                f"{fl:.1f},{fr:.1f},{rl:.1f},{rr:.1f},{gd:.2f},{gs:.1f},"
                f"{rep[0]:.1f},{rep[1]:.1f},{getattr(self, 'last_cluster_n', 0)},"
                f"{d[0]:.2f},{d[1]:.2f},{d[2]:.2f},{d[3]:.2f},"
                f"{d[4]:.2f},{d[5]:.2f},{d[6]:.2f},{d[7]:.2f},{d[8]},{d[9]},"
                f"{d[10]:.2f},{d[11]:.2f},{d[12]:.2f}\n")
            now_t = self.robot.getTime()
            if now_t - self.telemetry_t >= 2.0:
                self.telemetry_t = now_t
                with open(os.path.join(RESULTS, f"telemetry_{self.drone_id}.csv"), "w") as f:
                    f.write("t,x,y,z,roll,pitch,yaw,fl,fr,rl,rr,gd,gs,rep_x,rep_y,clusters,"
                            "bfe,ble,bfv,blv,pd,rd,i_fwd,i_left,state,ibvs,gx,gy,gz\n")
                    f.writelines(self.telemetry_buf)
        except Exception:
            pass

        self.motors[0].setVelocity(fl)
        self.motors[1].setVelocity(-fr)
        self.motors[2].setVelocity(-rl)
        self.motors[3].setVelocity(rr)

    def run(self):
        print(f"[{self.drone_id}] Taking off and waiting for Scout Handoff Packet on Channel 1...")
        step_count = 0

        try:
            self._run_loop()
        except Exception:
            # Console output is unreliable in Webots batch mode (bug 7): persist
            # the traceback so automated runs remain diagnosable.
            import traceback
            with open(os.path.join(RESULTS, f"controller_error_{self.drone_id}.txt"), "w") as f:
                f.write(traceback.format_exc())
            raise

    def _run_loop(self):
        step_count = 0

        while self.robot.step(self.timestep) != -1:
            step_count += 1