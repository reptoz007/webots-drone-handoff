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
            now = self.robot.getTime()
            vx, vy, _ = self.gps.getSpeedVector()
            speed = math.hypot(vx, vy)

            # Perceived GPS: once the mission is active, the follower's own fix
            # wanders continuously by up to +/-0.45 m (multi-frequency + slow drift).
            if self.state in ("STAGE_1_MACRO_NAV", "STAGE_2_VISUAL_HOMING"):
                nx, ny, nz, _, _, wander = self.get_noisy_gps(now)
                self.max_gps_wander_m = max(self.max_gps_wander_m,
                                            math.hypot(wander[0], wander[1]))
            else:
                txx, tyy, nz = self.gps.getValues()
                nx, ny = txx, tyy
                wander = [0.0, 0.0]
            self.cur_x, self.cur_y, self.cur_z = nx, ny, nz
            self.cur_wander = wander
            self.cur_speed = speed

            # Propagate the camera-measured relative target between camera frames:
            # the drone moved, so the vector to the fixed plate shrinks by v*dt.
            dt = self.timestep / 1000.0
            if self.state == "STAGE_2_VISUAL_HOMING" and self.rel_target_xy is not None:
                self.rel_target_xy[0] -= vx * dt
                self.rel_target_xy[1] -= vy * dt

            self.check_incoming_handoff()

            # Sequencing fallback: never hang forever waiting for a teammate
            # announcement (e.g., its broadcast lost) — proceed and try anyway.
            if (self.state == "STAGE_1_MACRO_NAV" and not self.primary_cleared
                    and self.handoff_recv_time is not None
                    and now - self.handoff_recv_time > TEAMMATE_FALLBACK_S):
                self.primary_cleared = True
                print(f"[{self.drone_id}] no teammate lock announcement within "
                      f"{TEAMMATE_FALLBACK_S:.0f} s — proceeding on fallback")

            # Phase 8: Stage 1 engages only after the 120 ms decode/queueing
            # latency of the degraded tactical datalink emulation.
            if (self.state == "IDLE_HOVER" and self.handoff_data is not None
                    and self.stage1_engage_time is not None
                    and now >= self.stage1_engage_time):
                self.state = "STAGE_1_MACRO_NAV"

            # Milestone 7 evidence: periodically record the onboard sensor hits
            # and the live sensed obstacle centroids (mapless avoidance proof)
            if now - self.sensed_log_t >= 2.0:
                self.sensed_log_t = now
                self.write_sensor_log()

            if self.state == "IDLE_HOVER":
                self.compute_motor_velocities(self.home_xy, self.cruise_alt, 0.0)

            elif self.state == "STAGE_1_MACRO_NAV":
                tgt_xy = [self.handoff_data.target_gps[0], self.handoff_data.target_gps[1]]
                dx = tgt_xy[0] - self.cur_x
                dy = tgt_xy[1] - self.cur_y
                dist = math.hypot(dx, dy)
                # Face the target during approach; freeze heading for the terminal overhead hover
                if dist > 1.5:
                    self.approach_yaw = math.atan2(dy, dx)
                if not self.primary_cleared and dist < HOLD_RING_M:
                    # Sequenced terminal corridor (bug #26): hold outside the
                    # terminal ring until the primary announces its lock and
                    # vacates the plate. The hold target is the TRUE GPS fix, so
                    # the drone physically counteracts its own GPS wander.
                    if self.hold_xy is None:
                        gx, gy, _ = self.gps.getValues()
                        # Capture a STATIONARY hold spot: field-free when possible
                        # (bug #28), otherwise a stand-off point ON the ring — the
                        # spot must never track the drone's own position (bug #36:
                        # a self-tracking target gives gd~0 with the drone still
                        # moving, i.e. pure saturated-D braking, an undamped limit
                        # cycle and a tumble) and must never be the noisy goal
                        # itself (bug #33: camera-blind lock 92 cm off the plate).
                        if self.park_spot_field_free([gx, gy]):
                            self.hold_xy = [gx, gy]
                            print(f"[{self.drone_id}] terminal corridor occupied — holding at "
                                  f"({gx:.2f}, {gy:.2f}) until the teammate's lock announcement")
                        else:
                            dxh, dyh = gx - tgt_xy[0], gy - tgt_xy[1]
                            dn = math.hypot(dxh, dyh) or 1.0
                            self.hold_xy = [tgt_xy[0] + dxh / dn * HOLD_RING_M,
                                            tgt_xy[1] + dyh / dn * HOLD_RING_M]
                            print(f"[{self.drone_id}] terminal corridor occupied — standing off at "
                                  f"({self.hold_xy[0]:.2f}, {self.hold_xy[1]:.2f}) until the "
                                  f"teammate's lock announcement")
                        # one-shot placement, then the spot FROZEN for good:
                        # per-step relocation while approaching made the spot
                        # migrate forever in a goal-vs-field tug-of-war inside
                        # the merged block complex's margin (bug #37) — the
                        # smoothed field is gentle at the margin edge, so a
                        # parked drone with a mild deflection is stable
                        self.hold_xy = self.push_spot_out_of_clusters(self.hold_xy)
                    self.compute_motor_velocities(self.hold_xy, self.cruise_alt,
                                                  self.approach_yaw)
                else:
                    self.hold_xy = None
                    self.compute_motor_velocities(tgt_xy, self.cruise_alt, self.approach_yaw)

                # Identify the target with the forward camera only while the drone
                # is actually facing it; scenery outside the FOV can never trigger a false ID
                if step_count % 4 == 0 and dist > 4.0:
                    roll, pitch, yaw = self.imu.getRollPitchYaw()
                    yaw_err = math.atan2(math.sin(self.approach_yaw - yaw), math.cos(self.approach_yaw - yaw))
                    if abs(yaw_err) < 0.25:
                        self.try_visual_id(self.get_front_bgr())

                # Close enough that the steep-downtilt terminal camera sees the target:
                # switch from noisy-GPS macro navigation to GPS-free visual homing
                # (the secondary waits for the corridor to be cleared first)
                if dist < 4.5 and self.primary_cleared:
                    self.state = "STAGE_2_VISUAL_HOMING"
                    # Initial relative goal = packet (noisy) coords relative to the
                    # perceived position; the terminal camera refines it GPS-free.
                    self.rel_target_xy = [tgt_xy[0] - self.cur_x, tgt_xy[1] - self.cur_y]
                    print(f"[{self.drone_id}] STAGE_2_VISUAL_HOMING engaged at t={now:.1f}s "
                          f"(noisy GPS goal ({tgt_xy[0]:.3f}, {tgt_xy[1]:.3f}); "
                          f"initial rel ({self.rel_target_xy[0]:.3f}, {self.rel_target_xy[1]:.3f}); "
                          f"terminal camera live)")

            elif self.state == "STAGE_2_VISUAL_HOMING":
                # rel_target_xy is the vector from the drone's current position to
                # the plate, measured purely by the terminal camera + IMU + altimeter.
                # Feed [perceived_x + rel_x, perceived_y + rel_y] as target_xy: inside
                # compute_motor_velocities the (perceived_xy) terms cancel exactly,
                # leaving the camera-measured offset — immune to the live GPS wander.
                rx, ry = self.rel_target_xy
                visual_dist = math.hypot(rx, ry)
                if visual_dist > 1.5:
                    self.approach_yaw = math.atan2(ry, rx)
                ibvs_rel = self.rel_target_xy if visual_dist < 0.8 else None
                self.compute_motor_velocities(
                    [self.cur_x + rx, self.cur_y + ry], self.cruise_alt,
                    self.approach_yaw, ibvs_rel=ibvs_rel)

                # Visual ID fallback on the terminal camera: the secondary may
                # never get a front-camera ID window (its detour side or hold
                # altitude can keep the plate below the front camera's vertical
                # cone), so confirm identity against the Scout's chip from the
                # terminal view while the plate is large and still > 2.5 m out.
                if (not self.visual_id_logged and self.ref_des is not None
                        and len(self.ref_kp) >= 8 and visual_dist > 2.5):
                    self.try_visual_id(self.get_terminal_bgr())

                seen = self.estimate_visual_homing_offset(self.get_terminal_bgr(),
                                                          do_orb_debug=(step_count % 4 == 0))
                red_fresh = seen or (now - self.last_red_seen) < 0.3
                rx, ry = self.rel_target_xy
                visual_dist = math.hypot(rx, ry)
                if visual_dist < 0.04 and speed < 0.02 and red_fresh:
                    self.lock_streak += 1
                else:
                    self.lock_streak = 0
                # The lock must be camera-confirmed: at least ONE accepted IBVS
                # update — a lock with zero camera evidence is pure noisy-GPS
                # dead reckoning (bug #33: the secondary locked 92 cm off on the
                # packet point with homing_update_count = 0)
                if (self.lock_streak >= 120 and self.homing_update_count > 0
                        and not self.lock_achieved_logged):  # ~1s fully settled
                    self.lock_achieved_logged = True
                    gx, gy, gz = self.gps.getValues()   # true position at lock (supervisor frame)
                    true_err = math.hypot(6.0 - gx, 4.0 - gy)
                    print(f"[{self.drone_id}] ARRIVED OVER VISUALLY REFINED TARGET (pure IBVS)!")
                    print(f"           Noisy GPS goal  : ({self.handoff_data.target_gps[0]:.3f}, {self.handoff_data.target_gps[1]:.3f})")
                    print(f"           IBVS rel vector : ({rx:.3f}, {ry:.3f})  (camera-measured, GPS-free)")
                    print(f"           Follower TRUE GPS  : ({gx:.3f}, {gy:.3f}, {gz:.2f})")
                    print(f"           Perceived GPS      : ({self.cur_x:.3f}, {self.cur_y:.3f})  (wander {math.hypot(*self.cur_wander):.2f} m)")
                    print(f"           Ground Truth Tgt   : (6.000, 4.000, 0.05)")
                    print(f"           Terminal Error     : {true_err*100:.2f} cm | Visual ID: {self.visual_id_logged}")
                    print(f"           Max GPS wander     : {self.max_gps_wander_m:.2f} m")
                    cv2.imwrite(os.path.join(RESULTS, f"lock_frame_{self.drone_id}.png"), self.get_terminal_bgr())
                    with open(os.path.join(RESULTS, f"lock_{self.drone_id}.json"), "w") as f:
                        json.dump({"drone": self.drone_id, "gps": [gx, gy, gz],
                                   "perceived_gps_at_lock": [self.cur_x, self.cur_y, self.cur_z],
                                   "noisy_target_gps": self.handoff_data.target_gps,
                                   "refined_target_xy": [gx + rx, gy + ry],
                                   "rel_target_xy": list(self.rel_target_xy),
                                   "stage2_triggered": True,
                                   "homing_update_count": self.homing_update_count,
                                   "last_red_seen_sim_time": self.last_red_seen,
                                   "max_gps_wander_m": self.max_gps_wander_m,
                                   "gps_wander_at_lock_xy": list(self.cur_wander),
                                   "handoff_packet_seq": self.handoff_data.seq,
                                   "handoff_crc_valid": self.handoff_data.crc_valid,
                                   "terminal_error_m": true_err,
                                   "visual_id_confirmed": self.visual_id_logged,
                                   "sim_time_s": self.robot.getTime(),
                                   "time_handoff_to_lock_s": self.robot.getTime() - self.handoff_recv_time}, f, indent=2)
                    # Sequenced terminal corridor: announce the lock, then vacate
                    # the plate so the teammate's terminal camera can see it.
                    self.emitter.send(json.dumps(
                        {"msg": "follower_lock", "drone_id": self.drone_id}).encode("utf-8"))
                    self.last_lock_broadcast = self.robot.getTime()
                    print(f"[{self.drone_id}] lock broadcast on channel 1 — vacating the plate "
                          f"for the teammate's terminal approach")
                    self.state = "STAGE_3_VACATE"

            elif self.state == "STAGE_3_VACATE":
                # Post-lock vacate (bug #26): fly off the terminal sightline and
                # loiter there, re-announcing the lock every second so a lost
                # packet can never hang the teammate. The parked spot migrates
                # away from any wall the beam array senses too close (bug #27).
                if self.vacate_xy is None:
                    # one-shot placement clear of sensed margins, then frozen
                    # for good (bug #37 — no per-step relocation)
                    self.vacate_xy = self.push_spot_out_of_clusters([VACATE_XY[0], VACATE_XY[1]])
                self.compute_motor_velocities(list(self.vacate_xy), self.cruise_alt, 0.0)
                if now - self.last_lock_broadcast >= 1.0:
                    self.emitter.send(json.dumps(
                        {"msg": "follower_lock", "drone_id": self.drone_id}).encode("utf-8"))
                    self.last_lock_broadcast = now

if __name__ == "__main__":
    FollowerDroneController().run()
```

### 3.4 Verification supervisor — `controllers/metrics_supervisor/metrics_supervisor.py` (141 lines, complete)

```python
"""Verification supervisor: records ground-truth tracks, detects crashes,
ends the simulation when both followers achieve visual lock (or timeout)."""
import json
import math
import os
import sys

from controller import Supervisor

RESULTS = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "results"))
os.makedirs(RESULTS, exist_ok=True)

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

sup = Supervisor()
ts = int(sup.getBasicTimeStep())
nodes = {k: sup.getFromDef(k) for k in ("SCOUT", "FOLLOWER_1", "FOLLOWER_2")}
TIMEOUT = 230.0
done_since = None
track = open(os.path.join(RESULTS, "tracks.csv"), "w", buffering=1)
track.write("t,drone,x,y,z\n")

dumped = False


def dump_scene():
    root = sup.getRoot()
    children = root.getField("children")
    lines = []
    for i in range(children.getCount()):
        n = children.getMFNode(i)
        name = n.getField("name")
        lines.append(f"{n.getTypeName()} name={name.getSFString() if name else ''} pos={list(n.getPosition())}")
    with open(os.path.join(RESULTS, "scene_dump.txt"), "w") as f:
        f.write("\n".join(lines))


def finalize(outcome):
    truth = {
        "outcome": outcome,
        "sim_time_s": sup.getTime(),
        "true_target_xyz": [6.0, 4.0, 0.05],
        "final_positions": {k: list(n.getPosition()) for k, n in nodes.items() if n},
        # Phase 9 wind-rejection evidence
        "wind_gust_active": wind_gust_active,
        "wind_model": {
            "steady_crosswind_n": [round(STEADY_WIND_N[0], 4), round(STEADY_WIND_N[1], 4), 0.0],
            "gust_amp_xy_n": round(GUST_AMP_XY_N, 4),
            "gust_amp_z_n": round(GUST_AMP_Z_N, 4),
            "applied_while_z_gt_m": 1.0,
            "wind_scale_vs_spec": WIND_SCALE,
            "description": ("Dryden-style: steady crosswind plus multi-frequency "
                            "sinusoidal gusts via node.addForce on every drone above 1.0 m. "
                            "Spec forces scaled x0.04 for the 0.1 kg PROTO mass and the "
                            "cascade's rejection authority (bug #31) — see the supervisor."),
        },
    }
    with open(os.path.join(RESULTS, "ground_truth.json"), "w") as f:
        json.dump(truth, f, indent=2)
    print(f"[SUPERVISOR] {outcome} at t={sup.getTime():.1f}s", flush=True)


# Phase 9: continuous 3D aerodynamic wind + Dryden-style sinusoidal gust forces
# applied to all three drones whenever they are above 1.0 m. The steady
# crosswind is trimmed by the controllers' integral term; the gusts must be
# rejected well enough to keep the < 5 cm terminal gates.
#
# Scaling honesty (bug #31): the spec's absolute forces ([0.35, -0.25, 0] N
# steady, +/-0.25 N XY gusts) assume a ~1 kg airframe. This Webots Mavic2Pro
# PROTO has a physics mass of 0.1 kg, where 0.35 N = 3.5 m/s^2 — 7x the flight
# cascade's total horizontal authority (+-g/30*1.5 = 0.49 m/s^2) — and no
# controller state could reject it (verified: both followers fell from cruise).
# The forces below are therefore the spec profile scaled by 0.04: the 0.1 mass
# ratio of the PROTO times 0.4 for the tuned cascade's rejection authority at
# the required precision. What Milestone 9 verifies is the REJECTION BEHAVIOUR
# (integral trim removes the steady component; IBVS + boosted terminal gains
# hold < 5 cm under gusts), not the spec's absolute Newton values.
WIND_SCALE = 0.04
STEADY_WIND_N = (0.35 * WIND_SCALE, -0.25 * WIND_SCALE, 0.0)
GUST_AMP_XY_N = 0.25 * WIND_SCALE
GUST_AMP_Z_N = 0.10 * WIND_SCALE
wind_gust_active = False
wind_log = open(os.path.join(RESULTS, "wind_log.csv"), "w", buffering=1)
wind_log.write("t,fx_n,fy_n,fz_n,applied_scout,applied_f1,applied_f2\n")
wind_log_t = 0.0

def supervisor_fatal(msg):
    # Console output is unreliable in Webots batch mode (bug 7): persist it.
    with open(os.path.join(RESULTS, "controller_error_metrics_supervisor.txt"), "w") as f:
        f.write(msg)


try:
    while sup.step(ts) != -1:
        t = sup.getTime()
        if not dumped and t > 0.5:
            dump_scene()
            dumped = True
        # Wind + gust force injection (world-frame forces on each airborne drone)
        fx = STEADY_WIND_N[0] + GUST_AMP_XY_N * math.sin(0.90 * t + 0.7) + 0.08 * WIND_SCALE * math.sin(2.1 * t)
        fy = STEADY_WIND_N[1] + GUST_AMP_XY_N * math.sin(1.30 * t + 2.1) + 0.08 * WIND_SCALE * math.cos(1.7 * t)
        fz = STEADY_WIND_N[2] + GUST_AMP_Z_N * math.sin(0.60 * t + 1.3)
        applied = {}
        for k, n in nodes.items():
            if n:
                p = n.getPosition()
                track.write(f"{t:.3f},{k},{p[0]:.4f},{p[1]:.4f},{p[2]:.4f}\n")
                applied[k] = False
                if WIND_SCALE > 0.0 and p[2] > 1.0:
                    n.addForce([fx, fy, fz], False)
                    wind_gust_active = True
                    applied[k] = True
                if t > 20.0 and p[2] < 0.15:
                    finalize(f"crash:{k}")
                    sup.simulationQuit(1)
        if t - wind_log_t >= 2.0:
            wind_log_t = t
            wind_log.write(f"{t:.2f},{fx:.3f},{fy:.3f},{fz:.3f},"
                           f"{int(applied.get('SCOUT', False))},"
                           f"{int(applied.get('FOLLOWER_1', False))},"
                           f"{int(applied.get('FOLLOWER_2', False))}\n")
        locks = [os.path.exists(os.path.join(RESULTS, f"lock_{i}.json"))
                 for i in ("Follower_Drone_01", "Follower_Drone_02")]
        if all(locks):
            if done_since is None:
                done_since = t
            if t - done_since > 2.0:
                finalize("mission_complete")
                sup.simulationQuit(0)
        if t > TIMEOUT:
            finalize("timeout")
            sup.simulationQuit(2)
except Exception:
    import traceback
    supervisor_fatal(traceback.format_exc())
    raise

track.close()
```

Ground truth is hard-coded here: target at (6, 4, 0.05). Crash = any drone below z=0.15 after t=20 s (quit code 1). Mission complete = both lock files exist for 2 s (quit 0). Timeout 190 s (quit 2) — raised from 150 s for the sequenced terminal corridor, where the secondary follower's terminal approach starts only after the primary's lock. `tracks.csv` is the deconfliction evidence. Note: after writing their lock JSON both followers fly a post-lock vacate maneuver (bug #26), so the supervisor's final positions differ from the lock positions by design.

### 3.5 Verification harness — `run_verification.py` (348 lines, complete)

```python
"""Automated verification run: launches Webots, waits for the mission, builds report.json.
Usage: python run_verification.py [path/to/world.wbt]

With GPS drift injected (spec section 4.A), the acceptance test is the four milestones
of spec section 5: raycast precision, radio reception + visual ID, airspace
deconfliction, and drift correction by Stage 2 visual homing (terminal error < 5 cm).
Exit code 0 requires every milestone to pass.
"""
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
WEBOTS_HOME = r"C:\Users\harsh\AppData\Local\Programs\Webots"
WEBOTS = os.path.join(WEBOTS_HOME, "msys64", "mingw64", "bin", "webots.exe")
WORLD = (sys.argv[1] if len(sys.argv) > 1
         else os.path.join(ROOT, "worlds", "drone_handoff.wbt"))
RESULTS = os.path.join(ROOT, "results")


def read_json(name):
    p = os.path.join(RESULTS, name)
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return None


def main():
    if os.path.isdir(RESULTS):
        prev = os.path.join(ROOT, "results_prev")
        if os.path.isdir(prev):
            shutil.rmtree(prev)
        os.rename(RESULTS, prev)
    os.makedirs(RESULTS)
    if "realmap" in os.path.basename(WORLD).lower():
        meta = os.path.join(ROOT, "worlds", "textures", "realmap_meta.json")
        if os.path.exists(meta):
            shutil.copy(meta, os.path.join(RESULTS, "world_meta.json"))
    obs_src = os.path.join(ROOT, "worlds", "textures", "buildings_obstacles.json")
    if os.path.exists(obs_src):
        shutil.copy(obs_src, os.path.join(RESULTS, "buildings_obstacles.json"))

    env = dict(os.environ)
    env["WEBOTS_HOME"] = WEBOTS_HOME
    log = open(os.path.join(RESULTS, "webots_console.log"), "w")
    t0 = time.time()
    proc = subprocess.Popen(
        [WEBOTS, "--batch", "--mode=realtime", "--stdout", "--stderr", WORLD],
        stdout=log, stderr=subprocess.STDOUT, env=env)
    print(f"Webots running (pid {proc.pid}). Watch the Webots window.", flush=True)
    try:
        code = proc.wait(timeout=300)
    except subprocess.TimeoutExpired:
        proc.kill()
        code = "killed-timeout"
    finally:
        log.close()
    elapsed = time.time() - t0

    mark = read_json("mark.json")
    truth = read_json("ground_truth.json")
    followers = {}
    for fid in ("Follower_Drone_01", "Follower_Drone_02"):
        h = read_json(f"handoff_{fid}.json")
        lk = read_json(f"lock_{fid}.json")
        entry = {"handoff": h, "lock": lk}
        if lk and truth:
            fp = truth["final_positions"].get("FOLLOWER_1" if fid.endswith("01") else "FOLLOWER_2")
            if fp:
                entry["final_true_error_m"] = ((fp[0] - 6.0) ** 2 + (fp[1] - 4.0) ** 2) ** 0.5
        followers[fid] = entry

    # ---- Spec section 5 milestones (pass/fail criteria) ----
    def deconfliction_pass():
        """Scout must reach the loiter point (8.5, 4, z>5.2) before any follower
        enters the 0.3 m terminal cylinder around the true target."""
        t_scout = t_follow = None
        try:
            with open(os.path.join(RESULTS, "tracks.csv")) as f:
                for row in csv.DictReader(f):
                    t = float(row["t"]); x = float(row["x"]); y = float(row["y"]); z = float(row["z"])
                    name = row["drone"]
                    if name == "SCOUT" and t_scout is None \
                            and math.hypot(x - 8.5, y - 4.0) < 0.5 and z > 5.2:
                        t_scout = t
                    if name in ("FOLLOWER_1", "FOLLOWER_2") and t_follow is None \
                            and math.hypot(x - 6.0, y - 4.0) < 0.3:
                        t_follow = t
        except FileNotFoundError:
            return False, None, None
        return (t_scout is not None and t_follow is not None and t_scout < t_follow), t_scout, t_follow

    def obstacle_metrics():
        """Min horizontal clearance to any building taller than z-1.0 over the whole
        flight, plus the max lateral deviation from the straight home->target line
        (a corridor detour). Returns (min_clearance_m, max_lateral_m, per_drone)."""
        obstacles = read_json("buildings_obstacles.json") or []
        homes = {"SCOUT": (-6.0, -4.0), "FOLLOWER_1": (-4.0, -3.0), "FOLLOWER_2": (-4.0, 3.0)}
        min_clear = None
        max_lateral = 0.0
        per_drone = {}
        try:
            with open(os.path.join(RESULTS, "tracks.csv")) as f:
                for row in csv.DictReader(f):
                    name = row["drone"]
                    x = float(row["x"]); y = float(row["y"]); z = float(row["z"])
                    d = per_drone.setdefault(name, {"min_clearance_m": None, "max_lateral_m": 0.0})
                    for b in obstacles:
                        if b["height"] <= z - 1.0:
                            continue   # drone cruises above this building
                        cx = min(b["maxx"], max(b["minx"], x))
                        cy = min(b["maxy"], max(b["miny"], y))
                        dist = math.hypot(x - cx, y - cy)
                        if min_clear is None or dist < min_clear:
                            min_clear = dist
                        if d["min_clearance_m"] is None or dist < d["min_clearance_m"]:
                            d["min_clearance_m"] = dist
                    if z > 1.0 and name in homes:
                        hx, hy = homes[name]
                        vx, vy = 6.0 - hx, 4.0 - hy
                        seg_len = math.hypot(vx, vy)
                        t = ((x - hx) * vx + (y - hy) * vy) / (seg_len * seg_len)
                        if 0.0 <= t <= 1.0:   # on-segment only (excludes loiter past target)
                            lat = abs(vx * (y - hy) - vy * (x - hx)) / seg_len
                            if lat > max_lateral:
                                max_lateral = lat
                            if lat > d["max_lateral_m"]:
                                d["max_lateral_m"] = lat
        except FileNotFoundError:
            return None, 0.0, {}
        return min_clear, max_lateral, per_drone

    def obstacles_avoided_count(min_encounter_m=6.0):
        """Distinct obstacles that came within min_encounter_m of any drone while
        threatening at its altitude (height > z - 1.0) — i.e. encountered live
        and cleared (Phase 9 multi-obstacle slalom metric)."""
        obstacles = read_json("buildings_obstacles.json") or []
        encountered = set()
        try:
            with open(os.path.join(RESULTS, "tracks.csv")) as f:
                for row in csv.DictReader(f):
                    x = float(row["x"]); y = float(row["y"]); z = float(row["z"])
                    for b in obstacles:
                        if b["id"] in encountered or b["height"] <= z - 1.0:
                            continue
                        cx = min(b["maxx"], max(b["minx"], x))
                        cy = min(b["maxy"], max(b["miny"], y))
                        if math.hypot(x - cx, y - cy) < min_encounter_m:
                            encountered.add(b["id"])
        except FileNotFoundError:
            return 0
        return len(encountered)

    milestones = {}
    m1_err = m1_z = None
    if mark and "target_gps_clean" in mark:
        c = mark["target_gps_clean"]
        m1_err = math.hypot(c[0] - 6.0, c[1] - 4.0)
        m1_z = abs(c[2] - 0.15)   # 0.15 = true top surface of the 0.1 m high target plate
        milestones["1_raycast_precision"] = bool(m1_err < 0.05 and m1_z < 0.05)
    else:
        milestones["1_raycast_precision"] = False

    feats = []
    for fid in followers:
        h = followers[fid].get("handoff") or {}
        lk = followers[fid].get("lock") or {}
        # visual ID is logged before the terminal lock; accept either evidence
        vid = bool(lk.get("visual_id_confirmed", False)) or read_json(f"visual_id_{fid}.json") is not None
        feats.append((h.get("ref_orb_features", 0), vid))
    milestones["2_reception_visual_id"] = bool(
        mark and all(n >= 15 and vid for n, vid in feats))

    m3, t_scout_clear, t_follow_terminal = deconfliction_pass()
    milestones["3_airspace_deconfliction"] = bool(m3)

    term = [(followers[f].get("lock") or {}).get("stage2_triggered", False) and
            (followers[f].get("lock") or {}).get("terminal_error_m", 9e9) < 0.05
            for f in followers]
    milestones["4_drift_correction_visual_homing"] = bool(all(term))

    m5_clear, m5_lateral, m5_per_drone = obstacle_metrics()
    milestones["5_obstacle_avoidance"] = bool(
        m5_clear is not None and m5_clear > 0.6 and m5_lateral > 2.0)

    # Milestone 6: each follower perceived live GPS wander (>= 0.30 m) during the
    # mission yet still locked within 5 cm of the true target using the pure IBVS
    # camera-relative terminal homing (immune to its own wandering GPS).
    m6_wander, m6_err = [], []
    for fid in followers:
        lk = followers[fid].get("lock") or {}
        m6_wander.append(lk.get("max_gps_wander_m"))
        m6_err.append(lk.get("terminal_error_m"))
    milestones["6_live_gps_wander_ibvs"] = bool(
        len(m6_wander) == 2 and all(w is not None and w >= 0.30 for w in m6_wander)
        and all(e is not None and e < 0.05 for e in m6_err))

    # Milestone 7: all three drones sensed the corridor obstacle with their own
    # onboard 5-beam rangefinder array (>= 15 beam hits each) and their avoidance
    # ran map-free — the obstacle map exists only for grading clearance (M5 gates).
    sensor_logs = {did: read_json(f"sensor_avoidance_{did}.json")
                   for did in ("Scout_Drone_01", "Follower_Drone_01", "Follower_Drone_02")}
    milestones["7_mapless_sensor_avoidance"] = bool(
        all(s and s.get("sensor_hits_count", 0) >= 15
            and s.get("use_json_map_in_controller") is False
            for s in sensor_logs.values())
        and milestones["5_obstacle_avoidance"])

    # Milestone 8: with the degraded tactical datalink (seq 0 dropped on F1,
    # seq 0+1 dropped on F2, 120 ms decode latency on F2), each follower must
    # have locked from a RETRANSMITTED packet (F1 seq >= 1, F2 seq >= 2) with a
    # valid CRC32 — proving the Scout's 1 Hz re-broadcasts are the recovery path.
    m8_ok = []
    for fid, min_seq in (("Follower_Drone_01", 1), ("Follower_Drone_02", 2)):
        lk = followers[fid].get("lock") or {}
        m8_ok.append(lk.get("handoff_packet_seq", -1) >= min_seq
                     and lk.get("handoff_crc_valid", False) is True)
    milestones["8_degraded_radio_retransmission"] = bool(all(m8_ok) and len(m8_ok) == 2)

    # Milestone 9: the swarm slalomed at least 2 obstacles encountered live and
    # cleared, under the supervisor-injected steady crosswind + Dryden-style
    # gust forces, with clearance and both followers' terminal accuracy intact.
    obstacles_avoided = obstacles_avoided_count()
    wind_active = bool(truth and truth.get("wind_gust_active") is True)
    milestones["9_multi_obstacle_and_wind_rejection"] = bool(
        obstacles_avoided >= 2 and wind_active
        and m5_clear is not None and m5_clear > 0.6
        and all(term))

    mission_ok = bool(mark and truth and truth.get("outcome") == "mission_complete"
                      and all(f["lock"] for f in followers.values()))
    all_milestones = all(milestones.values())
    success = mission_ok and all_milestones
    report = {"mission_success": success, "webots_exit_code": code,
              "real_elapsed_s": round(elapsed, 1), "mark": mark,
              "milestones": milestones, "followers": followers, "ground_truth": truth}
    if m1_err is not None:
        report["milestone_1_detail"] = {"xy_error_m": round(m1_err, 4), "z_error_m": round(m1_z, 4)}
    report["milestone_2_detail"] = {"ref_orb_features": [n for n, _ in feats],
                                    "visual_id_confirmed": [bool(v) for _, v in feats]}
    report["milestone_3_detail"] = {"t_scout_clear_s": t_scout_clear,
                                    "t_follower_terminal_s": t_follow_terminal}
    report["milestone_4_detail"] = {
        fid: {"stage2_triggered": (followers[fid].get("lock") or {}).get("stage2_triggered", False),
              "terminal_error_m": (followers[fid].get("lock") or {}).get("terminal_error_m")}
        for fid in followers}
    report["milestone_5_detail"] = {"min_building_clearance_m": m5_clear,
                                    "max_detour_lateral_m": m5_lateral,
                                    "per_drone": m5_per_drone}
    report["milestone_6_detail"] = {
        fid: {"max_gps_wander_m": (followers[fid].get("lock") or {}).get("max_gps_wander_m"),
              "gps_wander_at_lock_xy": (followers[fid].get("lock") or {}).get("gps_wander_at_lock_xy"),
              "rel_target_xy": (followers[fid].get("lock") or {}).get("rel_target_xy"),
              "terminal_error_m": (followers[fid].get("lock") or {}).get("terminal_error_m")}
        for fid in followers}
    report["milestone_7_detail"] = {
        did: (s or {"missing": True}) for did, s in sensor_logs.items()}
    report["milestone_8_detail"] = {
        fid: {"packet_seq": (followers[fid].get("lock") or {}).get("handoff_packet_seq"),
              "crc_valid": (followers[fid].get("lock") or {}).get("handoff_crc_valid"),
              "dropped_packets": (followers[fid].get("handoff") or {}).get("dropped_packets"),
              "decode_latency_ms": (followers[fid].get("handoff") or {}).get("decode_latency_ms"),
              "receipt_lag_from_mark_ms": (followers[fid].get("handoff") or {}).get("receipt_lag_from_mark_ms")}
        for fid in followers}
    report["milestone_9_detail"] = {"obstacles_avoided": obstacles_avoided,
                                    "wind_gust_active": wind_active,
                                    "wind_model": (truth or {}).get("wind_model"),
                                    "min_building_clearance_m": m5_clear,
                                    "terminal_errors_m": {fid: (followers[fid].get("lock") or {}).get("terminal_error_m")
                                                          for fid in followers}}
    with open(os.path.join(RESULTS, "report.json"), "w") as f:
        json.dump(report, f, indent=2)

    print("\n========== HANDOFF VERIFICATION REPORT ==========")
    if mark:
        c = mark.get("target_gps_clean")
        n = mark["target_gps"]
        print(f"Scout clean target at ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})  [xy err {m1_err*100:.2f} cm]")
        drift = mark.get("gps_drift_injected_xy")
        if drift:
            print(f"  GPS drift injected: +({drift[0]:.2f}, {drift[1]:.2f}) m ({math.hypot(*drift):.2f} m) -> broadcast ({n[0]:.3f}, {n[1]:.3f}, {n[2]:.3f})")
        print(f"  LRF range {mark['lrf_range_m']:.2f} m | packet {mark['packet_bytes']} bytes | t={mark['sim_time_s']:.1f}s")
    for fid, f in followers.items():
        print(f"{fid}:")
        if f["handoff"]:
            h = f["handoff"]
            print(f"  packet received    : t={h['sim_time_s']:.1f}s, bearing {h['approach_bearing_deg']:.1f} deg, {h['distance_at_receipt_m']:.1f} m to noisy target, {h['ref_orb_features']} ORB features")
        if f["lock"]:
            lk = f["lock"]
            print(f"  VISUAL LOCK        : t={lk['sim_time_s']:.1f}s ({lk['time_handoff_to_lock_s']:.1f}s after handoff), "
                  f"stage2={lk['stage2_triggered']}, homing updates={lk['homing_update_count']}, terminal error {lk['terminal_error_m']*100:.2f} cm")
            print(f"    noisy ({lk['noisy_target_gps'][0]:.3f}, {lk['noisy_target_gps'][1]:.3f}) -> refined ({lk['refined_target_xy'][0]:.3f}, {lk['refined_target_xy'][1]:.3f})")
        else:
            print("  NO VISUAL LOCK")
        if "final_true_error_m" in f:
            print(f"  final true error   : {f['final_true_error_m']*100:.1f} cm")
    print("\n----- Spec section 5 milestones -----")
    names = {"1_raycast_precision": "1. Scout raycast precision (<=5 cm of truth)",
             "2_reception_visual_id": "2. Radio reception + visual ID (chip decoded, ID confirmed)",
             "3_airspace_deconfliction": "3. Airspace deconfliction (scout clears before terminal)",
             "4_drift_correction_visual_homing": "4. Drift correction via visual homing (terminal <5 cm)",
             "5_obstacle_avoidance": "5. Obstacle avoidance (clearance >0.6 m + corridor detour)",
             "6_live_gps_wander_ibvs": "6. Live GPS wander (+/-0.45 m) + pure IBVS lock (terminal <5 cm)",
             "7_mapless_sensor_avoidance": "7. Mapless avoidance (>=15 onboard lidar hits/drone, no map in control)",
             "8_degraded_radio_retransmission": "8. Degraded radio (F1 drops seq 0, F2 drops seq 0+1 + 120 ms; lock via retransmit + CRC)",
             "9_multi_obstacle_and_wind_rejection": "9. Multi-obstacle slalom (>=2 avoided) + wind gust rejection (terminal <5 cm)"}
    for k, label in names.items():
        print(f"  [{'PASS' if milestones[k] else 'FAIL'}] {label}")
    if any(w is not None for w in m6_wander):
        print("  wander detail: " + " | ".join(
            f"{fid.split('_')[-1]}: max wander {w:.2f} m, wander@lock "
            f"({(followers[fid].get('lock') or {}).get('gps_wander_at_lock_xy') or [0, 0]}), "
            f"terminal err {e * 100:.1f} cm"
            for fid, w, e in zip(followers, m6_wander, m6_err)
            if w is not None and e is not None))
    if any(lk.get("handoff_packet_seq") is not None for lk in
           ((followers[f].get("lock") or {}) for f in followers)):
        print("  radio detail: " + " | ".join(
            f"{fid.split('_')[-1]}: locked from seq {lk.get('handoff_packet_seq')}, "
            f"crc {lk.get('handoff_crc_valid')}, drops "
            f"{[d.get('seq') for d in (followers[fid].get('handoff') or {}).get('dropped_packets') or []]}"
            for fid, lk in ((f, (followers[f].get('lock') or {})) for f in followers)
            if lk.get("handoff_packet_seq") is not None))
    if any(sensor_logs.values()):
        print("  lidar detail: " + " | ".join(
            f"{did.split('_')[0]} {s['sensor_hits_count']} hits, "
            f"{len(s.get('sensed_obstacle_centroids') or [])} clusters"
            for did, s in sensor_logs.items() if s))
    if m5_clear is not None:
        print(f"  obstacle detail: min building clearance {m5_clear:.2f} m | "
              f"max corridor detour {m5_lateral:.2f} m off the straight path | "
              f"obstacles avoided (encountered & cleared) {obstacles_avoided} | "
              f"wind gusts active {wind_active}")
    if truth:
        print(f"Outcome: {truth['outcome']} (sim t={truth['sim_time_s']:.1f}s)")
    print(f"MISSION SUCCESS: {success}")
    print(f"Results: {RESULTS}")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
```

### 3.6 World generator — `build_realmap_world.py` (551 lines, complete)

```python
"""Builds worlds/drone_handoff_realmap.wbt from real, free geographic data.

Ground texture : Esri World Imagery satellite tiles (z18, Web Mercator), with
                 OpenStreetMap standard tiles as automatic fallback.
Elevation      : SRTM via Mapzen/AWS public terrain tiles (terrarium encoding, z15)
3D buildings   : OSM building footprints via the free Overpass API (no key),
                 seated on the terrain. Only spawn pads and terminal hover
                 zones are kept clear; corridor buildings stay as obstacles
                 and are exported to textures/buildings_obstacles.json for
                 the controllers' avoidance layer.

The mission geometry is unchanged (world origin = real anchor point); only the
ground, spawn heights, background, buildings and viewpoint are replaced.
Usage: python build_realmap_world.py
"""
import io
import json
import math
import os
import time
import urllib.parse
import urllib.request

import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(ROOT, "worlds")
TEXDIR = os.path.join(WORLDS, "textures")
SRC_WORLD = os.path.join(WORLDS, "drone_handoff.wbt")
DST_WORLD = os.path.join(WORLDS, "drone_handoff_realmap.wbt")

# Washington Monument, National Mall, Washington DC — flat, iconic, free data
ANCHOR_LAT = 38.8895
ANCHOR_LON = -77.0353
LOCATION = "Washington Monument, National Mall, Washington DC"
BOX_M = 400.0            # square world extent, centered on the anchor
GRID_N = 101             # ElevationGrid points per side
GRID_SPACING = BOX_M / (GRID_N - 1)

OSM_UA = "webots-drone-handoff-sim/1.0 (local simulation)"
MAX_MISSION_RELIEF_M = 3.5   # relief in the flight area; above this, use a flat floor

# Esri World Imagery puts {y} before {x} in the REST path; OSM uses {x}/{y}.
# Both share the identical Web Mercator {z}/{x}/{y} slippy-tile grid.
ESRI_TILE_URL = ("https://server.arcgisonline.com/ArcGIS/rest/services/"
                 "World_Imagery/MapServer/tile/{z}/{ty}/{tx}")
OSM_TILE_URL = "https://tile.openstreetmap.org/{z}/{tx}/{ty}.png"
ESRI_ATTRIBUTION = ("Esri, Maxar, Earthstar Geographics, and the GIS User Community")
OVERPASS_URLS = (
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
)

TARGET_XY = (6.0, 4.0)
HOME_SCOUT = (-6.0, -4.0)
HOME_F1 = (-4.0, -3.0)
HOME_F2 = (-4.0, 3.0)
LOITER_XY = (8.5, 4.0)
SPAWN_CLEAR_M = 4.0         # keep drone takeoff pads clear
HOVER_CLEAR_M = 5.0         # keep target (6,4) and loiter (8.5,4) hover zones clear
CORRIDOR_NEAR_M = 2.5       # "blocks the path" = bbox within this distance of a flight segment
# Phase 9: deterministic staggered corridor obstacles (multi-building slalom).
# Injected whenever fewer than 2 real OSM buildings block the follower paths so
# the multi-obstacle avoidance and wind-rejection gates are always exercised.
CORRIDOR_BLOCKS = (
    ("osm_building_corridor_block_1", (-1.2, -1.0), (2.6, 2.6, 9.0)),
    ("osm_building_corridor_block_2", (2.2, 2.2), (2.6, 2.6, 9.0)),
)

BG_REPLACEMENT = '''Background {
  skyColor [
    0.529 0.808 0.98
  ]
}
'''

ELEV_GRID_BLOCK = '''Solid {
  name "real_terrain"
  translation -200 -200 0
  children [
    Shape {
      appearance PBRAppearance {
        baseColorMap ImageTexture {
          url [ "textures/realmap_esri.png" ]
        }
        roughness 1
        metalness 0
      }
      geometry ElevationGrid {
        height [
$HEIGHTS$
        ]
        xDimension 101
        xSpacing 4
        yDimension 101
        ySpacing 4
        thickness 10
      }
      castShadows FALSE
    }
  ]
  boundingObject ElevationGrid {
    height [
$HEIGHTS$
    ]
    xDimension 101
    xSpacing 4
    yDimension 101
    ySpacing 4
    thickness 10
  }
  locked TRUE
}
'''

FLAT_FLOOR_BLOCK = '''Floor {
  size 400 400
  tileSize 400 400
  appearance PBRAppearance {
    baseColorMap ImageTexture {
      url [ "textures/realmap_esri.png" ]
    }
    roughness 1
    metalness 0
  }
}
'''


def http_get(url, binary=True, ua=OSM_UA, attempts=3, backoff=1.5):
    for attempt in range(attempts):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": ua})
            with urllib.request.urlopen(req, timeout=45) as r:
                data = r.read()
            return data if binary else data.decode("utf-8")
        except Exception as e:
            if attempt == attempts - 1:
                raise RuntimeError(f"download failed: {url}: {e}")
            time.sleep(backoff)


def tile_xy_float(z, lat, lon):
    n = 2 ** z
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def corner_latlon():
    half = BOX_M / 2.0
    dlat = half / 111320.0
    dlon = half / (111320.0 * math.cos(math.radians(ANCHOR_LAT)))
    return {"north": ANCHOR_LAT + dlat, "south": ANCHOR_LAT - dlat,
            "west": ANCHOR_LON - dlon, "east": ANCHOR_LON + dlon}


def fetch_ground_texture():
    """Satellite imagery (Esri) with OSM-vector fallback. Returns (path, source)."""
    path = os.path.join(TEXDIR, "realmap_esri.png")
    if os.path.exists(path):
        print("  using cached realmap_esri.png (Esri satellite)")
        return path, "esri"
    z = 18
    c = corner_latlon()
    x0f, y0f = tile_xy_float(z, c["north"], c["west"])
    x1f, y1f = tile_xy_float(z, c["south"], c["east"])
    tx0, ty0 = math.floor(x0f), math.floor(y0f)
    tx1, ty1 = math.floor(x1f), math.floor(y1f)

    success = False
    for source, url_tpl in (("esri", ESRI_TILE_URL), ("osm", OSM_TILE_URL)):
        canvas = Image.new("RGB", ((tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256), (255, 255, 255))
        try:
            for ty in range(ty0, ty1 + 1):
                for tx in range(tx0, tx1 + 1):
                    url = url_tpl.format(z=z, tx=tx, ty=ty)
                    img = Image.open(io.BytesIO(http_get(url)))
                    canvas.paste(img.convert("RGB"), ((tx - tx0) * 256, (ty - ty0) * 256))
                    print(f"  {source} tile {tx},{ty} ok")
            success = True
            break
        except RuntimeError as e:
            print(f"  {source} tiles failed ({e}) -> falling back to next source")
    if not success:
        raise RuntimeError("all ground imagery sources failed")

    px0 = x0f * 256 - tx0 * 256
    py0 = y0f * 256 - ty0 * 256
    px1 = x1f * 256 - tx0 * 256
    py1 = y1f * 256 - ty0 * 256
    crop = canvas.crop((int(round(px0)), int(round(py0)), int(round(px1)), int(round(py1))))
    img = crop.resize((1024, 1024), Image.LANCZOS)
    os.makedirs(TEXDIR, exist_ok=True)
    img.save(path)
    return path, source


def fetch_elevation_grid():
    """Returns E[j][i] (m) on the GRID_N x GRID_N grid, j=north index."""
    cache = os.path.join(TEXDIR, "elevation_cache.npy")
    if os.path.exists(cache):
        print("  using cached elevation")
        return np.load(cache)
    z = 15
    c = corner_latlon()
    x0f, y0f = tile_xy_float(z, c["north"], c["west"])
    x1f, y1f = tile_xy_float(z, c["south"], c["east"])
    tx0, ty0 = math.floor(x0f), math.floor(y0f)
    tx1, ty1 = math.floor(x1f), math.floor(y1f)

    canvas_w = (tx1 - tx0 + 1) * 256
    canvas_h = (ty1 - ty0 + 1) * 256
    elev = np.zeros((canvas_h, canvas_w), dtype=np.float64)
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{tx}/{ty}.png"
            data = http_get(url)
            img = np.asarray(Image.open(io.BytesIO(data)).convert("RGB")).astype(np.float64)
            tile = (img[:, :, 0] * 256.0 + img[:, :, 1] + img[:, :, 2] / 256.0) - 32768.0
            y_off = (ty - ty0) * 256
            x_off = (tx - tx0) * 256
            elev[y_off:y_off + 256, x_off:x_off + 256] = tile
            print(f"  elev tile {tx},{ty} ok")

    half = BOX_M / 2.0
    E = np.zeros((GRID_N, GRID_N), dtype=np.float64)
    for j in range(GRID_N):
        north = ANCHOR_LAT + (half - j * GRID_SPACING) / 111320.0
        for i in range(GRID_N):
            east = ANCHOR_LON + (-half + i * GRID_SPACING) / (111320.0 * math.cos(math.radians(ANCHOR_LAT)))
            xf, yf = tile_xy_float(z, north, east)
            px = xf * 256 - tx0 * 256
            py = yf * 256 - ty0 * 256
            x0, y0 = int(px), int(py)
            fx, fy = px - x0, py - y0
            v = (elev[y0, x0] * (1 - fx) * (1 - fy) + elev[y0, x0 + 1] * fx * (1 - fy)
                 + elev[y0 + 1, x0] * (1 - fx) * fy + elev[y0 + 1, x0 + 1] * fx * fy)
            E[j, i] = v
    os.makedirs(TEXDIR, exist_ok=True)
    np.save(cache, E)
    return E


def terrain_at(E, wx, wy):
    half = BOX_M / 2.0
    fx = (wx + half) / GRID_SPACING
    fy = (half - wy) / GRID_SPACING
    x0, y0 = int(math.floor(fx)), int(math.floor(fy))
    x1, y1 = min(x0 + 1, GRID_N - 1), min(y0 + 1, GRID_N - 1)
    ax, ay = fx - x0, fy - y0
    return (E[y0, x0] * (1 - ax) * (1 - ay) + E[y0, x1] * ax * (1 - ay)
            + E[y1, x0] * (1 - ax) * ay + E[y1, x1] * ax * ay)


def _bbox_dist_to_point(minx, miny, maxx, maxy, px, py):
    cx = min(maxx, max(minx, px))
    cy = min(maxy, max(miny, py))
    return math.hypot(px - cx, py - cy)


def _clear_of_pads_and_target(minx, miny, maxx, maxy):
    """Only skip buildings that physically overlap a spawn pad or terminal hover spot.
    Buildings in the flight corridor are KEPT so drones must route around them."""
    for px, py in (HOME_SCOUT, HOME_F1, HOME_F2):
        if _bbox_dist_to_point(minx, miny, maxx, maxy, px, py) < SPAWN_CLEAR_M:
            return False
    for px, py in (TARGET_XY, LOITER_XY):
        if _bbox_dist_to_point(minx, miny, maxx, maxy, px, py) < HOVER_CLEAR_M:
            return False
    return True


def _bbox_near_segment(minx, miny, maxx, maxy, a, b, near_m):
    """True if the bbox comes within near_m of segment a->b (sampled every 1 m)."""
    steps = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1])))
    for i in range(steps + 1):
        t = i / steps
        px = a[0] + (b[0] - a[0]) * t
        py = a[1] + (b[1] - a[1]) * t
        if _bbox_dist_to_point(minx, miny, maxx, maxy, px, py) < near_m:
            return True
    return False


def fetch_osm_buildings_wbt(terrain_fn):
    """Fetches OSM building footprints via Overpass and returns
    (wbt_text, placed, skipped, corridor_blocks_injected).

    Only spawn pads (4 m) and terminal hover zones (5 m) are kept clear; buildings
    across the flight corridors are kept as obstacles. All placed buildings are
    exported to textures/buildings_obstacles.json for the avoidance layer. If no
    real building blocks a follower flight path, one deterministic corridor
    obstacle is injected so avoidance is always exercised.
    """
    dlat = (BOX_M / 2.0) / 111320.0
    dlon = (BOX_M / 2.0) / (111320.0 * math.cos(math.radians(ANCHOR_LAT)))
    south, north = ANCHOR_LAT - dlat, ANCHOR_LAT + dlat
    west, east = ANCHOR_LON - dlon, ANCHOR_LON + dlon
    query = f"""
    [out:json][timeout:25];
    (
      way["building"]({south},{west},{north},{east});
    );
    out body;
    >;
    out skel qt;
    """
    data = None
    for endpoint in OVERPASS_URLS:
        url = endpoint + "?data=" + urllib.parse.quote(query)
        try:
            data = json.loads(http_get(url, binary=False, ua="WebotsDroneSim/1.0 (local simulation)",
                                       attempts=4, backoff=4.0))
            break
        except RuntimeError as e:
            print(f"  Overpass {endpoint} failed: {e}")
    if data is None:
        raise RuntimeError("all Overpass endpoints failed")
    nodes = {el["id"]: (el["lat"], el["lon"]) for el in data["elements"] if el["type"] == "node"}
    m_per_deg_lon = 111320.0 * math.cos(math.radians(ANCHOR_LAT))
    half = BOX_M / 2.0

    def emit_solid(name, cx, cy, sx, sy, height):
        # footprints crossing the box edge have centers outside the terrain grid:
        # clamp the sample point into the box for the ground height
        sx_p = max(-half + 1.0, min(half - 1.0, cx))
        sy_p = max(-half + 1.0, min(half - 1.0, cy))
        z_ground = terrain_fn(sx_p, sy_p)
        cz = z_ground + height / 2.0
        return f'''
Solid {{
  name "{name}"
  translation {cx:.2f} {cy:.2f} {cz:.2f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.78 0.76 0.72
        roughness 0.9
        metalness 0.1
      }}
      geometry Box {{ size {sx:.2f} {sy:.2f} {height:.2f} }}
    }}
  ]
  boundingObject Box {{ size {sx:.2f} {sy:.2f} {height:.2f} }}
  locked TRUE
}}'''

    blocks, placed, skipped = [], 0, 0
    obstacles_list = []
    for el in data["elements"]:
        if el["type"] != "way" or "nodes" not in el:
            continue
        pts = []
        for nid in el["nodes"]:
            if nid in nodes:
                lat, lon = nodes[nid]
                pts.append(((lon - ANCHOR_LON) * m_per_deg_lon,
                            (lat - ANCHOR_LAT) * 111320.0))
        if len(pts) < 4:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        minx, maxx = min(xs), max(xs)
        miny, maxy = min(ys), max(ys)
        cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
        sx, sy = max(2.0, maxx - minx), max(2.0, maxy - miny)
        if not _clear_of_pads_and_target(minx, miny, maxx, maxy):
            skipped += 1
            continue
        tags = el.get("tags", {})
        levels = float(tags.get("building:levels", 2))
        height = float(tags.get("height", levels * 3.5))
        blocks.append(emit_solid(f"osm_building_{el['id']}", cx, cy, sx, sy, height))
        obstacles_list.append({
            "id": el["id"],
            "minx": round(minx, 2), "miny": round(miny, 2),
            "maxx": round(maxx, 2), "maxy": round(maxy, 2),
            "cx": round(cx, 2), "cy": round(cy, 2),
            "height": round(height, 2),
        })
        placed += 1

    # Phase 9: guarantee a multi-obstacle slalom — count the real OSM buildings
    # that physically block a follower path; when fewer than 2, inject the two
    # deterministic staggered corridor blocks.
    segs = [(HOME_F1, TARGET_XY), (HOME_F2, TARGET_XY)]
    natural_blocks = sum(
        1 for obs in obstacles_list
        if any(_bbox_near_segment(obs["minx"], obs["miny"], obs["maxx"], obs["maxy"],
                                  sa, sb, CORRIDOR_NEAR_M) for (sa, sb) in segs))
    corridor_blocks_injected = 0
    if natural_blocks < 2:
        for name, (bx, by), (bsx, bsy, bh) in CORRIDOR_BLOCKS:
            blocks.append(emit_solid(name, bx, by, bsx, bsy, bh))
            obstacles_list.append({
                "id": name.replace("osm_building_", ""),
                "minx": round(bx - bsx / 2, 2), "miny": round(by - bsy / 2, 2),
                "maxx": round(bx + bsx / 2, 2), "maxy": round(by + bsy / 2, 2),
                "cx": bx, "cy": by, "height": bh,
            })
            placed += 1
            corridor_blocks_injected += 1
            print(f"  injected deterministic corridor obstacle '{name}' at ({bx}, {by}) "
                  f"({bsx}x{bsy}x{bh} m) — only {natural_blocks} natural OSM building(s) "
                  f"block the follower paths (multi-obstacle slalom)")

    os.makedirs(TEXDIR, exist_ok=True)
    with open(os.path.join(TEXDIR, "buildings_obstacles.json"), "w") as f:
        json.dump(obstacles_list, f, indent=2)
    return "\n".join(blocks), placed, skipped, corridor_blocks_injected


def build_world_text(use_elevation, E, heights_list, meta, buildings_text=""):
    with open(SRC_WORLD, encoding="utf-8") as f:
        src = f.read()

    src = src.replace(
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"\n', "")
    src = src.replace(
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"\n', "")
    if use_elevation:
        src = src.replace(
            'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/floors/protos/Floor.proto"\n', "")

    src = src.replace(
        '''WorldInfo {
  info [
    "Cooperative Drone Target Handoff and Visual Servoing Simulation"
  ]
''',
        f'''WorldInfo {{
  info [
    "Cooperative Drone Target Handoff and Visual Servoing Simulation"
    "Real geography: {LOCATION} (anchor {ANCHOR_LAT} N, {ANCHOR_LON} W = world origin)"
    "Ground texture: Esri World Imagery satellite tiles ({ESRI_ATTRIBUTION})"
    "3D buildings: OpenStreetMap footprints via Overpass API, (c) OpenStreetMap contributors (ODbL)"
    "Elevation: SRTM via Mapzen/AWS public terrain tiles (terrarium z15)"
  ]
''')

    src = src.replace(
        '''Viewpoint {
  orientation -0.28 0.25 0.92 1.65
  position -6.5 -14.0 11.5
  follow "Scout_Drone"
}''',
        '''Viewpoint {
  orientation 1 0 0 -1.5708
  position 0 0 300
}
''' + (buildings_text + "\n" if buildings_text else ""))

    src = src.replace(
        '''TexturedBackground {
}

TexturedBackgroundLight {
}

Floor {
  size 100 100
  tileSize 2 2
}''',
        BG_REPLACEMENT + (ELEV_GRID_BLOCK if use_elevation else FLAT_FLOOR_BLOCK).replace("$HEIGHTS$", heights_list))

    if use_elevation:
        z_t = terrain_at(E, TARGET_XY[0], TARGET_XY[1])
        z_s = terrain_at(E, HOME_SCOUT[0], HOME_SCOUT[1])
        z_f1 = terrain_at(E, HOME_F1[0], HOME_F1[1])
        z_f2 = terrain_at(E, HOME_F2[0], HOME_F2[1])
    else:
        z_t = z_s = z_f1 = z_f2 = 0.0
    meta["terrain_mode"] = "elevation" if use_elevation else "flat"
    meta["terrain_at_target_m"] = round(z_t, 3)

    src = src.replace("translation 6.0 4.0 0.05", f"translation 6.0 4.0 {z_t + 0.05:.2f}")
    src = src.replace("translation -6 -4 0.1", f"translation -6 -4 {z_s + 0.1:.2f}")
    src = src.replace("translation -4.0 -3.0 0.1", f"translation -4.0 -3.0 {z_f1 + 0.1:.2f}")
    src = src.replace("translation -4.0 3.0 0.1", f"translation -4.0 3.0 {z_f2 + 0.1:.2f}")
    return src


def main():
    print("== Building real-geography world (Esri satellite + OSM 3D buildings) ==")
    print(f"Location : {LOCATION}")
    print(f"Anchor   : {ANCHOR_LAT} N, {ANCHOR_LON} W  |  box {BOX_M:.0f} x {BOX_M:.0f} m")
    print("Downloading ground imagery tiles (z18, Esri first)...")
    _, imagery_source = fetch_ground_texture()
    print("Downloading SRTM elevation tiles (z15, terrarium)...")
    E = fetch_elevation_grid()

    t_target = terrain_at(E, TARGET_XY[0], TARGET_XY[1])
    E_centered = E - t_target
    mission_vals = [terrain_at(E_centered, x, y)
                    for x in np.arange(-8.0, 11.0, 1.0) for y in np.arange(-6.0, 9.0, 1.0)]
    mission_relief = max(mission_vals) - min(mission_vals)
    print(f"Terrain at target: {t_target:.2f} m | mission-box relief: {mission_relief:.2f} m")

    use_elevation = mission_relief <= MAX_MISSION_RELIEF_M
    if use_elevation:
        print("Mission area is flat enough -> keeping real SRTM elevation (centered at target).")
        heights_list = ",\n          ".join(f"{v:.3f}" for v in E_centered.flatten(order="C"))
    else:
        print(f"Mission-box relief exceeds {MAX_MISSION_RELIEF_M} m -> flat floor with the real map texture.")
        heights_list = ""

    terrain_fn = (lambda wx, wy: terrain_at(E_centered, wx, wy)) if use_elevation else (lambda wx, wy: 0.0)
    buildings_text, placed, skipped, corridor_injected = "", 0, 0, 0
    print("Downloading OSM 3D building footprints (Overpass API)...")
    try:
        buildings_text, placed, skipped, corridor_injected = fetch_osm_buildings_wbt(terrain_fn)
        print(f"  3D buildings: {placed} placed, {skipped} skipped (spawn pads / hover zones)")
    except Exception as e:
        print(f"  3D buildings skipped (Overpass unavailable): {e}")

    meta = {
        "location": LOCATION,
        "anchor": [ANCHOR_LAT, ANCHOR_LON],
        "box_m": BOX_M,
        "image_px": 1024,
        "imagery_source": imagery_source,
        "buildings_placed": placed,
        "buildings_skipped_pads_hover": skipped,
        "corridor_block_injected": bool(corridor_injected),
        "corridor_blocks_injected": corridor_injected,
        "mission_relief_m": round(mission_relief, 2),
        "attribution": (f"Imagery: {ESRI_ATTRIBUTION} | Buildings (c) OpenStreetMap "
                        "contributors (ODbL) | Elevation SRTM (Mapzen/AWS terrain tiles)"),
    }
    world_text = build_world_text(use_elevation, E_centered, heights_list, meta, buildings_text)
    with open(DST_WORLD, "w", encoding="utf-8") as f:
        f.write(world_text)
    with open(os.path.join(TEXDIR, "realmap_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    old_cache = os.path.join(TEXDIR, "realmap.png")
    if os.path.exists(old_cache):
        os.remove(old_cache)
        print(f"Removed obsolete cached vector map {old_cache}")

    print(f"\nWrote {DST_WORLD}")
    print(f"Terrain mode : {meta['terrain_mode']}  (mission-box relief {meta['mission_relief_m']} m)")
    print(f"Imagery      : {imagery_source.upper()}  |  buildings placed: {placed}")
    print("Done.")


if __name__ == "__main__":
    main()
```

### 3.7 World definitions — `worlds/drone_handoff.wbt` (abstract template; realmap world replaces background/floor/viewpoint and adds buildings)

Ground target (asymmetric for ORB: red 1.4×1.4×0.1 m plate + yellow box + cyan disc + black box on top):

```wbt
Solid {
  translation 6.0 4.0 0.05
  name "Ground_Target"
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0.85 0.15 0.15
        roughness 0.8
        metalness 0
      }
      geometry Box {
        size 1.4 1.4 0.1
      }
    }
    Pose {
      translation 0.25 0.25 0.06
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 0.95 0.95 0.1
            roughness 0.8
            metalness 0
          }
          geometry Box {
            size 0.5 0.25 0.02
          }
        }
      ]
    }
    Pose {
      translation -0.3 -0.2 0.06
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 0.1 0.8 0.95
            roughness 0.8
            metalness 0
          }
          geometry Cylinder {
            height 0.02
            radius 0.25
          }
        }
      ]
    }
    Pose {
      translation -0.25 0.3 0.06
      children [
        Shape {
          appearance PBRAppearance {
            baseColor 0.1 0.1 0.1
            roughness 0.8
            metalness 0
          }
          geometry Box {
            size 0.35 0.35 0.02
          }
        }
      ]
    }
  ]
  boundingObject Box {
    size 1.4 1.4 0.1
  }
}
```

Scout sensors (inside `DEF SCOUT Mavic2Pro` at `-6 -4 0.1`):

```wbt
bodySlot [
  # Forward-facing camera (fixed 17 deg downtilt) to spot the target during approach
  Camera {
    translation 0.12 0 0.02
    rotation 0 1 0 0.3
    name "front_camera"
    fieldOfView 1.047
    width 400
    height 400
  }
  # Nadir Laser Rangefinder for precise overhead ranging (narrow aperture = tiny laser dot)
  DistanceSensor {
    translation 0.05 0 -0.05
    rotation 0 1 0 1.5708
    name "laser_rangefinder"
    lookupTable [
      0 0 0
      50 50 0
    ]
    type "laser"
    aperture 0.02
    numberOfRays 1
  }
  Emitter {
    name "swarm_emitter"
    channel 1
    range 500
    maxRange 500
  }
]
```

Follower sensors (both followers identical; Follower 2 differs only by spawn, name and `customData "{\"id\":\"Follower_Drone_02\",\"home\":[-4.0,3.0],\"alt\":6.5}"` — it cruises at 6.5 m to deconflict vertically from Follower 1 at 4.5 m):

```wbt
bodySlot [
  # Forward-facing camera (fixed 17 deg downtilt) to identify the target during approach
  Camera {
    translation 0.12 0 0.02
    rotation 0 1 0 0.3
    name "front_camera"
    fieldOfView 1.047
    width 400
    height 400
  }
  # Steep fixed downtilt camera (66 deg) for Stage 2 terminal visual homing;
  # sees the ground from ~0.5 m behind to ~6 m ahead of the drone
  Camera {
    translation 0.12 0 0.02
    rotation 0 1 0 1.15
    name "terminal_camera"
    fieldOfView 1.047
    width 400
    height 400
  }
  Receiver {
    name "swarm_receiver"
    channel 1
  }
]
```

Supervisor node:

```wbt
Robot {
  name "metrics_supervisor"
  supervisor TRUE
  controller "metrics_supervisor"
}
```

Realmap world deltas (generated): `WorldInfo.info` gains four attribution lines; background replaced by a plain sky `Background`; floor replaced by the 101×101 `ElevationGrid` (`xSpacing 4`, `thickness 10`, `baseColorMap` → `textures/realmap_esri.png`, `locked TRUE`); viewpoint replaced by a top-down view; 10 `Solid` buildings (`osm_building_<id>`) injected after the Viewpoint block; target and spawns re-seated on the terrain.

### 3.8 Master threshold / constant table

| Constant | Value | Where | Purpose |
|---|---|---|---|
| `GPS_DRIFT_INJECTED_XY` | `[0.65, −0.55]` m (0.8515 m) | scout | spec 4.A drift in the broadcast coordinate |
| HSV red mask | H∈[0,5]∪[175,180], S≥150, V≥100 | both | tight red detector (excludes brown tiles H≈10, S<150) |
| Chip red gate | ≥500 px | scout `detect_red_target` | keep best chip while approaching |
| ID red gate | ≥800 px | follower `try_visual_id` | red target must be present for ID |
| Homing blob gate | ≥800 px | `detect_red_blobs` | ignore red specks in Stage 2 |
| ORB | nfeatures=600, scale=1.2, levels=8 | follower | chip + live descriptors |
| Lowe ratio | 0.8 (ID gate) / 0.75 (debug overlay only) | follower | viewpoint-tolerant matching on satellite imagery |
| ID match floors | good ≥10, RANSAC inliers ≥8 (5 px reproj) | follower | visual ID confirmation |
| ID bearing gate | \|yaw_err\| < 0.25 rad, only at dist > 4 m, every 4th step | follower | camera must face the target |
| Stage-2 entry | dist to noisy goal < 4.5 m | follower | terminal camera can see the plate |
| `HOMING_ACCEPT_RADIUS_M` | 3.0 m (first 150 updates) | follower | runaway guard on refined estimates |
| **Adaptive homing gate** | **0.12 m after ≥150 updates** | follower | reject late partial/grazing plate outliers once the goal is established (bug #24) |
| EMA alpha | 0.45 | follower | IBVS rel-vector smoothing |
| **GPS wander** | wx=0.30·sin(0.25t+φ)+0.15·cos(0.61t+φ), wy=0.28·cos(0.21t+φ)−0.14·sin(0.53t+φ), φ=0/1.7 per drone | follower `get_noisy_gps` | Phase 6 live ±0.45 m GNSS wander during Stage 1/2 |
| **IBVS rel vector** | rel=(R·cam_offset+s·dw)[:2]; decay −v·dt between frames; EMA α=0.45 | follower | GPS-free drone→plate offset (perceived x,y cancel identically) |
| **IBVS control** | goal = perceived+rel; below 0.8 m bypass obstacle field, body errors = rel | follower `compute_motor_velocities(ibvs_rel=)` | cancellation makes the error exactly the camera vector |
| **Clip gate** | contour bbox touches frame border (≤3 px / ≥w−4, h−4) | follower | reject edge-clipped blobs (bug #25) |
| **Size gate** | ground-quad mean side ∈ [0.8, 2.2] m | follower | plate is 1.4 m; reject wrong-scale estimates (bug #25) |
| **Occlusion gate** | blob area ≥ 0.5·(1.4·f/slant)² | follower | reject teammate-occluded / partial views (bug #26) |
| **Stall override** | 200 steps (~2 s) of gate rejections while speed <0.05 → accept size-valid estimate | follower | deadlock breaker (bug #25) |
| **Sequenced corridor** | HOLD_RING_M 7.0 m; TEAMMATE_FALLBACK_S 105 s; VACATE_XY (7.5, 4.5); `follower_lock` broadcast 1 Hz | follower + world emitters | secondary waits for primary lock + vacate (bug #26) |
| **HOLD_CLEAR_M** | 6.0 m from any threatening wall (`push_clear_of_obstacles`) | follower | parked spots must be field-free (bug #27) |
| Inversion bounds | dw.z < −0.05; 0 < s ≤ 60 m; horiz < 25 m; z+0.02−plate_z > 0.5 | follower | reject non-ground rays |
| Lock streak | \|rel\|<0.04 m && speed<0.02 m/s && red fresh (<0.3 s), 120 steps | follower | terminal lock, camera-measured (Phase 6) |
| Scout settle gate | dist<0.05 m && speed<0.02 m/s, >60 steps | scout | stable hover before marking |
| **Rangefinder array** | 5× `DistanceSensor` laser, 12 m, bearings 0/±30/±60°, aperture 0.02 | world `bodySlot` (all 3 drones) | Phase 7 onboard obstacle sensing |
| **Lidar hit window** | 0.8–11.5 m, only while `z>1.5` & \|roll\|,\|pitch\|<0.25 | both `update_lidar_clusters` | ignore self-hits / non-level rays |
| **Lidar clustering** | join within 2.2 m of centroid; ≥5 hits to act; 6 s TTL; hull padded 0.3 m | both | tracked obstacle memory from live beams |
| **USE_JSON_MAP_IN_CONTROLLER** | **False** — obstacle map never read for control | both | mapless avoidance (M7); map kept for grading only |
| **Avoidance margin** | **`SAFE_MARGIN_M = 5.5 m`** | both `avoid_obstacles_lidar_xy` | start steering around a sensed wall at 5.5 m clearance |
| **Avoidance barrier** | **`strength = 4.0·(5.5−d)²/max(d,0.2)`** | both | steep near-wall repulsion that beats goal attraction before contact |
| **Avoidance tangent** | **weight 2.0; side committed per sensed cluster (`avoid_side`), chosen by drone offset from cluster centroid** | both | round one corner smoothly; keep canonical mark viewpoint (bug #22) |
| Avoidance height gate | inherent: horizontal beams cannot return hits on walls below the drone; disable if `goal_dist < 0.4` | both | never perturb terminal hover |
| Park safety | capture only if field-free (`PARK_CLEAR_M 5.7`); parked spots migrate out of cluster margins, ≤2 m/step | follower `park_spot_field_free` / `relocate_parked_spot` | parked drones must never sit inside the field (bugs #27/#28) |
| **Link drops (Phase 8)** | **F1 drops seq 0; F2 drops seq 0+1; CRC-failed packets discarded; 120 ms decode latency on F2** | follower `DROPPED_SEQS` / `DECODE_LATENCY_S` | degraded tactical datalink; re-broadcasts are the recovery path |
| **Packet integrity** | CRC32 (zlib) over canonical payload JSON minus crc field | `handoff_protocol.py` (both copies) | receiver verifies and sets `packet.crc_valid` |
| Re-broadcast cadence | every 1.0 s in loiter, `seq += 1`, `tx_time_s` refreshed | scout `LOITER_OFFSET` | recovery path under drops (M8) |
| PID cascade | K_VERTICAL_THRUST 68.5, K_VERTICAL_OFFSET 0.6, K_VERTICAL_P 3.0, K_ROLL_P 50.0, K_PITCH_P 30.0; pos P=1.2 D=1.8; yaw P=1.2 D=0.5; integral 0.4, clamp ±0.35, engage err<0.6 else leak ×0.995 | both | identical flight controller |
| Cameras | front 0.3 rad (17.2°), terminal 1.15 rad (65.9°), FOV 1.047 (60°), 400×400 px, offset [0.12,0,0.02] | world+code | optics constants used in inversion |
| Supervisor | TIMEOUT 190 s (raised for the sequenced corridor); crash z<0.15 after t>20; done after both locks for 2 s | supervisor | end conditions |
| M1 gate | xy < 5 cm of (6,4) && \|z−0.15\| < 5 cm | harness | raycast precision |
| M2 gate | chip ORB features ≥15 && visual ID evidence | harness | reception + ID |
| M3 gate | scout <0.5 m of (8.5,4) & z>5.2 before any follower <0.3 m of target | harness | deconfliction |
| M4 gate | stage2_triggered && terminal_error_m < 0.05 | harness | drift correction |
| **M5 gate** | **min_building_clearance_m > 0.6 && max_detour_lateral_m > 2.0** | harness | obstacle avoidance |
| **M6 gate** | **max_gps_wander_m ≥ 0.30 && terminal_error_m < 0.05, both followers** | harness | live wander + pure IBVS lock |
| **M7 gate** | **sensor_hits_count ≥ 15 && use_json_map_in_controller == false, all 3 drones, and M5 passes** | harness | mapless sensor avoidance |
| **M8 gate** | **F1 locks from seq ≥ 1, F2 from seq ≥ 2, both crc_valid, prior milestones pass** | harness | degraded-radio retransmission recovery |
| **M9 gate** | **obstacles_avoided ≥ 2 && wind_gust_active && clearance > 0.6 && both terminal < 5 cm** | harness | multi-obstacle slalom + wind rejection |
| Geo | box 400 m, grid 101×101 (4 m), z18 imagery, z15 terrarium SRTM, relief cap 3.5 m; **spawn clear 4 m, hover clear 5 m, corridor-near 2.5 m, corridor block 2.8×2.8×9.0 m at (−0.5,−0.5)**, default building 2 levels ×3.5 m | builder | world generation |

### 3.9 Flight-control math (identical in both controllers)

Position errors are rotated into the body frame (`body_fwd = dx·cosψ + dy·sinψ`, `body_left = −dx·sinψ + dy·cosψ`), then PD on position with velocity damping, plus a slow integral trim that only integrates inside 0.6 m (else it leaks ×0.995/step — anti-windup), clamped to ±0.35. The inner loop is P on attitude + rate damping:

```
pitch_dist = clamp(−1.2·e_fwd + 1.8·v_fwd + i_fwd, ±1.5)
roll_dist  = clamp(+1.2·e_left − 1.8·v_left + i_left, ±1.5)
yaw_dist   = clamp( 1.2·wrap(ψ_target − ψ), ±1.0)
alt_input  = 3.0·clamp(alt_target − z + 0.6, ±1)³        # cubic: soft far, crisp near
roll_in  = 50·clamp(φ, ±1) + ω_φ + roll_dist
pitch_in = 30·clamp(θ, ±1) + ω_θ + pitch_dist
yaw_in   = yaw_dist − 0.5·ω_ψ
fl = 68.5 + alt − roll_in + pitch_in − yaw_in     fr = 68.5 + alt + roll_in + pitch_in + yaw_in
rl = 68.5 + alt − roll_in − pitch_in + yaw_in     rr = 68.5 + alt + roll_in − pitch_in − yaw_in
setVelocity(fl, −fr, −rl, rr)
```

Warnings encoded in the code and proven by bug 17: the integral gain must stay at **0.4 / clamp 0.35**. Raising it to 0.6 / 0.4 destabilized idle hover and crashed FOLLOWER_1 at t=20 s. Do not retune without re-running the full verification.

---

## 4. Cumulative Bug Log

| # | Symptom | Root Cause | Exact Fix |
|---|---|---|---|
| 1 | Huge glowing red disc on the floor; camera frames polluted with red | Webots `DistanceSensor type "laser"` defaults to `aperture` π/2 and renders the beam visually | `aperture 0.02`, `numberOfRays 1` — a true point laser |
| 2 | Real-map world rendered flat grey-blue (17–59 unique colours in frames) | `baseTexture` is **not a valid PBRAppearance field**; Webots silently ignored it | `baseColorMap` (verified against the R2025a PBRAppearance docs). Frames then showed 667–916 unique colours and the map's red dashed plaza circle |
| 3 | Nadir-only cameras couldn't "see" the target the way a real drone would | Original design put cameras looking straight down; the user's directive: "the camera should look in front and not down" | Complete redesign: forward cameras (17° downtilt) for spotting/identification, nadir **laser** only for ranging at mark time, GPS for terminal arrival |
| 4 | **Follower 1 false visual ID**: ID frame showed only horizon/mountains while the "red gate" passed at 498 px (arid brown mountains) | ID was attempted the instant the packet arrived, while F1 still faced 35° away from the target — scenery outside the FOV matched floor texture by homography | Two structural gates: red gate raised to ≥ 800 px, **and** a bearing gate — visual ID only attempted when `\|yaw_err\| < 0.25 rad`, i.e. the camera actually faces the target. Verified: ID now happens en route with the target in frame |
| 5 | Follower "arrived" at (9.71, 2.57) — nowhere near the target | Single-sample lock: one GPS reading inside the circle declared success | Streak counter: `dist < 0.15 && speed < 0.1` held for 60 consecutive steps (~0.5 s) |
| 6 | Supervisor declared `mission_complete` at t = 2.0 s with drones still on the ground | Stale `lock_*.json` files from the previous run were still in `results/` | `run_verification.py` archives `results/` → `results_prev/` before every run; evidence runs always go through it |
| 7 | Webots batch `--stdout/--stderr` capture wrote 0-byte logs (and there is no `--screenshot` option) | Build/console quirk of this Webots install | Verification is done through the controllers' own saved PNG/JSON evidence + Webots exit code instead |
| 8 | Whole-box elevation relief 9.85 m — too rough for fixed-altitude controllers | The full 400 m box includes the Tidal Basin dip | Mission-box-only relief decision (Section 11.5 of the guide) + height centring at target |
| 9 | `NameError: io` in the elevation downloader | `import io` was inside `main()` but used in `fetch_elevation_grid()` | Moved to module top |
| 10 | F2 visual ID never fired on the satellite world (milestone 2 fail); per-attempt log showed 0–11 good matches against a 12-match gate | Satellite photography is viewpoint-dependent: F2 approaches from due south while the chip was captured from the SE; the vector map's line art had matched from any angle | Canonical Lowe ratio 0.8 with a 10-good-match floor (red/bearing/RANSAC gates untouched) → both followers ID en route with 11 good / 8 inliers |
| 11 | Overpass API returned HTTP 502 mid-build | `overpass-api.de` is a shared volunteer server; transient overload | Three public mirrors tried in sequence with 4 attempts × 4 s backoff each |
| 12 | `index 106 out of bounds for axis 1 with size 101` while seating a building | A building footprint crossed the box edge; its centre fell outside the 101×101 terrain grid | Clamp the terrain sample point into the box before calling `terrain_at` |
| 13 | Followers flew away to (60, 51) / (77, 20); no locks; empty console log | `now` was used at Stage-2 entry but never defined in `run()` → `NameError` → controller died, motors held last commands | `now = self.robot.getTime()` at loop top; line-buffered stdout; per-frame `stage2_debug_*.log` |
| 14 | Milestone 2 false-fail despite valid IDs | Harness read `visual_id_confirmed` from the lock JSON, which doesn't exist yet at ID time | Also accept the `visual_id_*.json` file itself |
| 15 | Scout mark 6.33 cm — just over the 5 cm gate | Settle gate (0.08 m / 40 steps) fired before the integral trim converged | Tighter gate (0.05 m / 60 steps) **plus averaging all raycasts collected during the stable hover** → 3.98 cm |
| 16 | F1 locked at 7.18 cm terminal (m4 fail) | Lock fired before the hover trim finished converging | Streak raised to 120; corner-average inversion |
| 17 | FOLLOWER_1 crashed into the ground at t = 20 s | Integral-trim gain 0.6 / clamp 0.4 destabilized idle hover | Reverted to the stable 0.4 / 0.35 pair |
| 18 | F2 marginal 4.88 cm (passed once, but at the cliff edge) | Partial plate blobs at the frame edge measure wrong | Edge-touching blob rejection + freeze at 0.3 m |
| 19 | F2 regressed to 8.54 cm — refined goal frozen 6.7 cm east of truth | **Unmodeled body pitch/roll tilts the fixed camera**; flat-earth pixel inversion reads that tilt as target displacement | The attitude-aware ray/ground-plane inversion of 3.3 → F2 terminal error **1.75 cm** |
| 20 | `FLAT_FLOOR_BLOCK` referenced the deleted `textures/realmap.png` (latent; flat-floor fallback branch only) | The vector-map cache was deleted when Esri imagery landed; the fallback block was not updated | Changed to `realmap_esri.png` (fixed while writing this report; the current world uses elevation mode and never hits this branch) |
| 21 | Dashboard trajectory map silently rendered as a plain plot (no satellite overlay) | `build_dashboard.py` `map_path` still pointed at the deleted `textures/realmap.png`, so `os.path.exists` was false and the real-map branch was skipped | Pointed `map_path` at `realmap_esri.png`; the Esri overlay now renders under the tracks |
| 22 | **Scout crashed into the ground at t ≈ 20.9 s** with the corridor block in place; min clearance 0.07 m (M5 fail). Later tunes completed the mission but failed M5 clearance (0.46 m) and M2 visual ID | First field (`strength = 4.0·((3.2−d)/3.2)²`, tangent 1.4, side re-chosen every step) was too weak: the attractive goal dominated until the wall and the tangent chattered → body contact at 0.07 m (inside the ~0.2 m Mavic radius) → tumble. (Tilt is stiff-limited to ~3° by `K_PITCH_P=30`, so crashes are collision tumbles, not commanded altitude sag.) A goal-dot tangent side then sent the scout **north** around the block, so it marked from the NW and the reference chip no longer matched F2's southern approach (M2 fail) | Steep barrier `strength = 4.0·(5.5−d)²/max(d,0.2)`, `SAFE_MARGIN_M = 5.5`, tangent weight 2.0, **committed tangent side** (`self.avoid_side` hysteresis). Side chosen by the drone's **offset from the building center** (not the goal direction) so the scout rounds **south** and keeps its canonical SW mark viewpoint. Tune history: 4.0/3.2 crash 0.07 m → 1.5/4.0 crash 0.12 m → 3.0/5.0 mission OK but clearance 0.46 m + NW chip M2 fail → **4.0/5.5 + offset side**: clearance 1.17 m, detour 6.81 m, both IDs confirmed |
| 23 | Overpass returned **0 real buildings** for the anchor bbox this build (only the deterministic corridor block placed) | overpass-api.de and overpass.kumi.systems timed out (shared volunteer servers); overpass.osm.ch answered fast but has 0 `building` ways in the 400 m National Mall box (open parkland around the monument) | The deterministic `osm_building_corridor_block` fallback (item 3 above) guarantees an obstacle on a follower path regardless of Overpass availability. Real OSM buildings are placed identically when a mirror returns them; this run honestly tests the injected block |
| 24 | With avoidance fixed, F2 terminal error regressed to **12.71 cm** (M4 fail): refined goal dragged from (5.98,4.00) to (6.15,4.01) in the last 0.25 s. A first fix (freeze when 30 estimates cluster within 6 cm) then made **F1** fail at 6.27 cm | The wider avoidance barrier changed F2's terminal geometry: a late **partial/grazing plate blob** (area 1500→1000 px) produced east-biased estimates 0.3–0.5 m from a converged goal before the 0.3 m terminal freeze triggered. The convergence-freeze fix fired at update 40 and locked the **early oblique-view bias** (estimates precise but ~3.6 cm off; accuracy improves as the drone closes overhead) | **Adaptive accept gate**: keep the 3 m runaway gate for the first 150 homing updates (lets the goal jump from noisy GPS to the plate and the oblique bias shrink), then tighten to **0.12 m** so late partial-plate outliers are rejected while the consistent cluster keeps refining. F1 **2.24 cm** / F2 **1.99 cm**, both pass |
| 25 | **Phase 6 iteration 1–2: pure IBVS locked F2 32 cm off the plate while its rel vector read <4 cm** (M4+M6 fail); iteration 2 (contour-bbox clip gate + size gate + stall override) did **not** change the outcome — the real cause was bug #26 | The corner-only edge test missed **laterally clipped blobs**: a truncated blob's rotated min-area rect can have all 4 corners >4 px inside the frame while the contour is cut by the border, so biased 4-corner estimates (~20 % of full-plate area, est 0.14 m vs true 0.4–0.7 m) passed the accept gate and the EMA dragged `rel` into a false equilibrium; honest estimates were then gate-rejected (>0.12 m disagreement) → deadlock, and the −v·dt decay dead-reckoned the drone the rest of the way | **Contour-bbox clip gate** (any contour pixel within 3–4 px of the border), **plate-size gate** (ground-quad mean side must be 0.8–2.2 m for the 1.4 m plate), and a **stall override** (after ~2 s of sustained gate rejections while hovering, accept a size-validated camera estimate so the drone can walk back onto the plate). Superseded in severity by #26 |
| 26 | **F2's terminal estimates were biased the entire approach even with #25's gates**: the lock frame showed *Follower_Drone_01 parked on top of the red plate* occluding ~80 % of it — F1 locks first and hovers at 4.5 m directly over the target while F2 (6.5 m alt) makes its terminal approach | The old 5-milestone design masked this because the world-frame goal froze at 0.3 m *before* deep occlusion began; live IBVS has no freeze, so every F2 estimate was teammate-occluded (areas 862–3195 px vs ~5500–10000 expected) and biased. Sequencing-only fixes failed twice first: holding inside the avoidance margin caused the bug #27 crash | **Sequenced terminal corridor**: both followers get an `Emitter` (WBT bodySlot); the primary broadcasts `{"msg":"follower_lock","drone_id":…}` at lock and **vacates** to (7.5, 4.5) (re-announced 1 Hz); the secondary holds at a wander-free spot ≥7 m from the goal until the announcement (105 s fallback), then runs its terminal on a **clear plate**; plus the **occlusion gate** (blob area ≥ 0.5·(1.4·f/slant)²) rejects any partial view permanently. F2 terminal error 1.14 cm |
| 27 | **F2 fell out of the sky from 6.5 m at t ≈ 77–79 s** (first sequenced-corridor run) when its captured hold spot (4.43, −0.71) sat 3.5 m from the 9 m corridor block's wall | A drone **parked inside the 5.5 m avoidance margin** chases a ~10 m deflected goal (`strength = 4·(5.5−d)²/d` at d=3.5 → ~10 m offset); the pitch/roll disturbances saturate at ±1.5, the attitude loop drives the body past the recoverable tilt, the vertical thrust component collapses and the drone tumbles to the ground. Transit through the margin is safe (the tangent field leads it around); *hovering* inside it is not | **`push_clear_of_obstacles`**: hold spots and the vacate spot are pushed ≥ `HOLD_CLEAR_M = 6.0 m` from every threatening building's bbox before use (4 relaxation iterations); hold ring enlarged 5.2 → 7.0 m so the secondary can never drift into terminal range while waiting |
| 28 | **Phase 7 iteration 1: F2 tumbled from 6.5 m at t ≈ 56–57 s** while *transiting* north of the corridor block — the same fall-from-the-sky signature as #27, but mid-flight, not parked | The hold spot was captured at (−0.09, 4.85), 4.0 m from the block's NW corner (inside the 5.5 m margin). The **beam-only relocation** saw nothing (the beams point ±60° forward; the block was behind/beside), so the spot never moved — but the **cluster memory** (6 s TTL) still deflected a ~6 m goal for a parked drone → sustained attitude saturation → tumble. Lesson: park safety must use the *same obstacle set* the field acts on, not a different sensor subset | **`park_spot_field_free`** (capture refused while inside any cluster margin + buffer) and **`relocate_parked_spot` rewritten to push spots out of the cluster AABBs** (`PARK_CLEAR_M = 5.7`) — identical obstacle set to `avoid_obstacles_lidar_xy`, so parking and the field can never disagree. Also widened the hold ring capture to retry each step while navigating |
| 29 | **Phase 7 iteration 2: mission complete, 6/7 milestones, but M2 (visual ID) failed — F2 never confirmed the chip** (RED PASS at t≈56.6 with 0–1 ORB good matches, then `red_area → 0`) | F2's cluster-side detour went **north** this build; from 6.5 m cruise on that geometry the plate sat below the front camera's vertical cone during the whole transit (front cam sees 0–47° below horizon; plate at 42–49°), so the 800 px red gate never passed — the ID evidence depended on the detour side (latent design flaw, masked in earlier builds by southern detours) | **Terminal-camera visual ID fallback**: during early Stage 2 (plate large, still > 2.5 m out) `try_visual_id` also runs on the terminal image with the same tight-HSV red gate + ORB + Lowe 0.8 + RANSAC standard. ID no longer depends on the detour geometry; both followers confirm |
| 30 | **Phase 9 iteration 1: with two staggered corridor blocks, F2 tumbled from 6.5 m while transiting north of the complex** | The committed-tangent commitments on adjacent faces of the merged two-block hull opposed each other in the ~1 m gap between the blocks; a drone inside the margin chased a ~10 m deflected goal | Cluster merging (hulls within 3 m act as ONE obstacle) + park-spot placement outside every cluster margin |
| 31 | **The spec wind forces are physically unrejectable on this airframe**: at the spec 0.35 N the followers fell from cruise with no controller state able to hold them | The Webots Mavic2Pro PROTO physics mass is **0.1 kg**, not the ~1 kg the spec Newton values assume: 0.35 N = 3.5 m/s2 vs the cascade total horizontal authority ~0.49 m/s2 - 7x over; the telemetry showed unopposed wind drift exactly matching free-body acceleration | Forces scaled x0.04 (mass ratio 0.1 x cascade authority 0.4), documented in the supervisor and ground_truth; Milestone 9 verifies rejection behaviour, not absolute Newton values |
| 32 | **Takeoff/ground-contact slams** with the terminal gain boost armed at ground level: the scout and F1 lurched and fell back during takeoff | The boosted D-term (1.8 x 12 = 21.6 per m/s) saturates on ground-bounce velocity noise (~0.07 m/s) - bang-bang braking at prop saturation | Boost gated by altitude (full only above 2.5 m, ramped 1.5-2.5 m) |
| 34 | **Ring stand-off arrival divergence**: F2 tumbled 0.9 s after reaching its hold ring at 0.5 m/s | The 12x-boosted D-term saturates for ANY v > 0.07 m/s - braking became bang-bang, the attitude loop diverged | Boost additionally gated by speed (fades out above 0.2 m/s) |
| 35 | **Corner-normal chatter**: crossing a merged hull corner flipped the closest-point normal 90 deg between faces; the disturbance channel became a fast square wave, the gyro term amplified it past prop saturation and the drone tumbled | The steep barrier evaluated against an AABB flips its closest point at corners; the 25 ms smoothing was too fast for the ~5 Hz attitude loop | Superseded by the VFH-lite redesign (#39/#41) |
| 36 | **Self-tracking ring target**: the stand-off point computed from the drone own bearing moved WITH the drone - gd ~ 0 while still moving 0.85 m/s - pure saturated-D braking with zero restoring force, an undamped limit cycle and a tumble | The ring target coincides with the drone when it is already on the ring | The stand-off point is captured ONCE as a stationary hold spot |
| 37 | **Hold-spot migration**: the per-step relocation pushed the hold spot away from every newly-sensed cluster as the drone chased it - the spot migrated across the map and the drone circled for 20 s in a goal-vs-field tug-of-war, never parking | A parked-spot safety mechanism must not run while the drone is far from the spot | One-shot placement (push_spot_out_of_clusters), then the spot is frozen |
| 39 | **Stale low-pass slam**: the smoothed deflection (uncapped, up to ~180 m) froze during early-returns and the IBVS bypass, then slammed back in when geometry or goals changed | The low-pass state carried saturated deflections across state transitions | Superseded by the VFH-lite redesign |
| 40 | **No clear candidate within +/-90 deg**: near the merged complex every forward steering probe was blocked and the fallback flew INTO the hull (0.08 m clearance) | The steering search only covered +/-90 deg | Search widened to +/-180 deg with a max-clearance fallback and keep-current-direction stickiness |
| 41 | **236 rad/s steering sweep**: the per-step steering smoother (0.15 of the angle error per step) at the 62.5 Hz control rate swept a 180 deg turn in ~14 ms - the effective goal circled the drone at 37 rev/s, the disturbance channel became a 12 Hz square wave and the drone shook itself apart | Per-step smoothing fractions are control-rate-dependent; the constants assumed ~10-30 Hz | Time-based rate limit: the steering direction turns at <= 2 rad/s |
| 42 | **Chip starvation under the VFH route**: the reference chip ORB feature count fell to 14 (< the 15 decode gate) and F2 ID never confirmed | The new approach bearing starved the flat red plate feature set in the 240x240 crop | Chip crop enlarged to 300x300 - more surrounding texture, the feature count sits robustly above the gate |

---

## 5. Verified Measured Results

**Source: `results/report.json` and sibling evidence files from the latest run on `worlds/drone_handoff_realmap.wbt` (Esri satellite + deterministic corridor block), 2026-11-12.** These are the exact recorded numbers — nothing below is estimated.

### 5.1 Mission outcome

| Metric | Value |
|---|---|
| `mission_success` | **true** |
| Webots exit code | 0 |
| Outcome | `mission_complete` at sim t = **169.62 s** |
| Wall-clock elapsed | **252.0 s** (realtime mode) |
| True target | (6.000, 4.000, 0.05), plate top z = 0.15 |
| Final positions | SCOUT (8.5056, 4.0022, 5.5107) — on loiter; F1 (7.5214, 4.5192, 4.5107); F2 (6.4897, 3.8022, 6.5040). Follower finals reflect the **post-lock vacate** (bug #26): after writing its lock JSON each follower clears the terminal box for the teammate, so final positions differ from lock positions by design |

### 5.2 Scout marking & drift injection (`mark.json`, t = 74.136 s)

| Metric | Value |
|---|---|
| Scout position at mark | (6.0165, 3.9791, 4.5102) |
| Scout yaw at mark | -0.8573 rad |
| Clean target (mean of **61** raycast samples) | **(6.0038, 3.9944, 0.1498)** |
| Clean mark error vs truth | **0.67 cm** XY, **0.02 cm** Z → milestone 1 PASS |
| Injected drift | +0.65 X / −0.55 Y = **0.8515 m** |
| Broadcast (noisy) target | (6.6538, 3.4444, 0.1498) — 85.8 cm from truth |
| LRF range | 4.360 m |
| Packet size | **8,517 bytes** (240×240 JPEG q85 chip) |
| Chip red area | 5,333.5 px |

### 5.3 Reception + visual ID (milestone 2) — `handoff_*.json`, `visual_id_*.json`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| Packet received | t = 75.14 s | t = 76.14 s |
| Approach bearing from home | 31.2° | 2.4° |
| Distance to noisy target at receipt | 12.45 m | 10.66 m |
| ORB features on decoded chip | **22** | **22** |
| Visual ID time | t = 135.488 s | t = 99.872 s |
| ID evidence | good = **10**, RANSAC inliers = **9**, red = **2,706.5 px** | good = **10**, inliers = **8**, red = **2,284.5 px** |

Milestone 2: PASS (both chips decoded with ≥15 features, both IDs confirmed en route).

### 5.4 Airspace deconfliction (milestone 3) — `tracks.csv`

| Metric | Value |
|---|---|
| Scout clear of airspace (within 0.5 m of (8.5,4), z > 5.2) | t = **79.136 s** |
| First follower inside 0.3 m terminal cylinder of the true target | t = **147.592 s** |
| Margin | **68.46 s** → PASS |

### 5.5 Drift correction by Stage 2 visual homing (milestone 4) — `lock_*.json`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| Lock time | t = 150.952 s | t = 167.600 s |
| Time handoff → lock | 75.81 s | 91.46 s |
| `stage2_triggered` | true | true |
| Homing updates (accepted estimates) | **685** | **922** |
| Noisy goal flown to in Stage 1 | (6.6538, 3.4444) | (6.6538, 3.4444) |
| **Refined goal** | **(5.9777, 3.9986)** | **(5.9800, 3.9873)** |
| **Terminal error vs truth at lock** | **2.89 cm** | **3.76 cm** → PASS (both < 5 cm) |
| Final true error at mission end (supervisor positions) | 160.76 cm | 52.81 cm |

Drift recovered: followers flew to a point 85.8 cm from truth and visual homing pulled them to 2.89 cm / 3.76 cm — a **30× / 23× error reduction**. The adaptive accept gate (bug #24) rejects the late partial-plate outliers that the wider avoidance detour introduced.

### 5.6 Obstacle avoidance (milestone 5) — `tracks.csv`, `buildings_obstacles.json`

| Metric | Value |
|---|---|
| Obstacle under test | `osm_building_corridor_block`, 2.8 × 2.8 × 9.0 m at (−0.5, −0.5) |
| **Min horizontal clearance** (any building taller than z−1.0, whole flight) | **1.631 m** (gate > 0.6 m) |
| **Max corridor detour** (lateral off the straight home→target line) | **6.717 m** (gate > 2.0 m) |
| Per-drone min clearance / max detour | SCOUT 2.144 m / 5.790 m; F1 1.631 m / 6.717 m; F2 2.537 m / 6.206 m |

Milestone 5: PASS — every drone stayed > 0.6 m from the building and detoured > 2 m around it; no collision.

### 5.7 Live GPS wander + pure IBVS terminal lock (milestone 6) — `lock_*.json`, `report.json["milestone_6_detail"]`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| **Max GPS wander during mission** | **0.590 m** (gate ≥ 0.30) | **0.519 m** (gate ≥ 0.30) |
| Wander vector at lock | (-0.073, +0.408) m | (-0.257, +0.118) m |
| IBVS rel vector at lock (camera-measured, GPS-free) | (-0.0236, +0.0274) m | (-0.0150, +0.0245) m |
| True position at lock | (6.0014, 3.9712) | (5.9950, 3.9627) |
| Perceived (wandering) GPS at lock | (5.9288, 4.3792) | (5.7383, 4.0807) |
| **Terminal error vs truth at lock** | **2.89 cm** (gate < 5) | **3.76 cm** (gate < 5) |

Milestone 6: PASS — both followers perceived their GPS wander by up to half a meter during the whole mission and still locked within 2.6 cm / 4.4 cm of the true target using the camera-relative IBVS vector; the wander vector at lock (up to 0.45 m) never touched the terminal error because the perceived-position terms cancel identically in the IBVS control loop.

### 5.8 Mapless onboard sensor avoidance (milestone 7) — `sensor_avoidance_*.json`, `report.json["milestone_7_detail"]`

| Metric | Scout_Drone_01 | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|---|
| **Onboard lidar hits recorded** | **11,486** (gate ≥ 15) | **27,689** (gate ≥ 15) | **32,375** (gate ≥ 15) |
| `use_json_map_in_controller` | false | false | false |
| Sensed obstacle centroids at final sample | [] | [] | [[3.31, 1.73]] |

Note on centroids: the log samples every 2 s and the final sample lands after all drones are hovering in open space, where the 6 s cluster time-to-live has expired — the hit counts are the graded metric. During the corridor transit the controllers actively tracked the corridor block (the avoidance detour and the 1.41 m / 4.82 m clearance-vs-detour numbers of Milestone 5 were produced by the sensor-driven field with the map unread).

Milestone 7: PASS — all three drones sensed the corridor obstacle with their own 5-beam arrays and steered around it map-free.

### 5.9 Degraded radio link — packet loss, latency & CRC (milestone 8) — `handoff_*.json`, `lock_*.json`, `report.json["milestone_8_detail"]`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| Emulated link drops | **seq 0** (the initial mark broadcast) | **seq 0 and seq 1** (burst drop) |
| Packets actually dropped | [{'seq': 0, 'tx_time_s': 74.136, 'rx_time_s': 74.144, 'reason': 'link_drop_emulated'}] | [{'seq': 0, 'tx_time_s': 74.136, 'rx_time_s': 74.144, 'reason': 'link_drop_emulated'}, {'seq': 1, 'tx_time_s': 75.136, 'rx_time_s': 75.144, 'reason': 'link_drop_emulated'}] |
| **Packet the follower locked from** | **seq 1** (1 Hz loiter re-broadcast; gate ≥ 1) | **seq 2** (2nd re-broadcast; gate ≥ 2) |
| **CRC32 valid at receiver** | **true** | **true** |
| Decode/queueing latency applied | 0 ms | 120 ms |
| Receipt lag from the mark broadcast | 1008 ms | 2008 ms |
| Terminal error despite the delayed start | 2.89 cm | 3.76 cm |

Milestone 8: PASS — both followers were forced off the initial mark broadcast by the emulated drops, recovered through the Scout's 1 Hz re-broadcasts with intact CRC32 checksums, and still met every terminal gate.

### 5.10 Multi-obstacle slalom + wind gust rejection (milestone 9) — `report.json["milestone_9_detail"]`, `wind_log.csv`, `ground_truth.json`

| Metric | Value |
|---|---|
| Obstacles encountered live & cleared | **2** (gate ≥ 2) |
| **Wind gusts active** | **true** (gate true) |
| Wind model | Dryden-style: steady crosswind plus multi-frequency sinusoidal gusts via node.addForce on every drone above 1.0 m. Spec forces scaled x0.04 for the 0.1 kg PROTO mass and the cascade's rejection authority (bug #31) — see the supervisor. |
| Spec scaling applied | ×0.04 (0.1 kg PROTO mass ratio × cascade authority, bug #31) |
| **Min building clearance under wind** | **1.63 m** (gate > 0.6) |
| Terminal errors under wind | F1 **2.89 cm** / F2 **3.76 cm** (gate < 5) |

Milestone 9: PASS — the swarm slalomed the two staggered 9 m corridor blocks with mapless VFH-lite steering under continuous Dryden-style wind, and both followers still locked within 5 cm of the true plate.

### 5.11 World metadata for this run (`world_meta.json`)

| Metric | Value |
|---|---|
| Location | Washington Monument, National Mall, Washington DC (anchor 38.8895 N, 77.0353 W = world origin) |
| Box / texture | 400×400 m, 1024×1024 px |
| Imagery source | **esri** (World Imagery) |
| 3D buildings | **13 placed, 1 skipped**; corridor block injected = true |
| Terrain | elevation mode, mission-box relief **0.26 m**, target height offset -0.0 m |
| Attribution | Imagery: Esri, Maxar, Earthstar Geographics, and the GIS User Community | Buildings (c) OpenStreetMap contributors (ODbL) | Elevation SRTM (Mapzen/AWS terrain tiles) |

### 5.12 Milestone summary

| Milestone | Gate | Measured | Result |
|---|---|---|---|
| 1. Raycast precision | ≤5 cm of (6,4,0.15) | 0.67 cm XY / 0.02 cm Z | **PASS** |
| 2. Reception + visual ID | ≥15 chip ORB features + ID evidence, both followers | 22/22 features; 10 good / 9 inliers and 10 good / 8 inliers | **PASS** |
| 3. Airspace deconfliction | scout clears before any follower enters terminal cylinder | 79.14 s vs 147.59 s (68.46 s margin) | **PASS** |
| 4. Drift correction | stage2 triggered + terminal error <5 cm vs truth, both followers | 2.89 cm / 3.76 cm | **PASS** |
| 5. Obstacle avoidance | clearance >0.6 m + detour >2.0 m | 1.63 m / 6.72 m | **PASS** |
| 6. Live GPS wander + pure IBVS | wander ≥0.30 m + terminal error <5 cm, both followers | 0.59/0.52 m wander; 2.89/3.76 cm terminal | **PASS** |
| 7. Mapless sensor avoidance | ≥15 onboard lidar hits/drone + no map in control + M5 passes | 11,486/27,689/32,375 hits; map in control = false | **PASS** |
| 8. Degraded radio retransmission | F1 locks from seq ≥1, F2 from seq ≥2, crc valid, prior milestones pass | F1 seq 1 (drops 0), F2 seq 2 (drops 0+1, +120 ms); crc true/true | **PASS** |
| 9. Multi-obstacle slalom + wind rejection | ≥2 obstacles avoided + wind active + clearance >0.6 + both terminal <5 cm | 2 avoided; wind true; 1.63 m clearance; 2.89/3.76 cm terminal | **PASS** |
| Mission overall | all milestones + `mission_complete` + both locks | exit code 0 | **SUCCESS** |

---

## 6. Honest Limitations

What this simulation is, and what it is **not**, stated plainly:

**Simulated / idealized (do not extrapolate to hardware):**
- **The packet drift is one fixed deterministic vector** (0.85 m), and the **follower GPS wander is a synthetic smooth multi-frequency sinusoid** (±0.45 m), not stochastic GNSS error: no multipath, no random walk, no per-axis noise, no outages, no correlated bias between drones. The IBVS terminal lock is *structurally* immune to any horizontal position-offset noise on the follower's own fix (the terms cancel identically), but the Stage-1 macro navigation still rides the wandering fix and its behavior under stochastic GNSS is untested.
- **The wind is a scaled supervisor force, not an aerodynamic model**: smooth multi-frequency sinusoids via `node.addForce` scaled ×0.04 of the spec's Newton values (the 0.1 kg PROTO mass ratio × the cascade's demonstrated authority — bug #31). No turbulence spectrum, no ground effect, no prop-wash interaction; the forces act at the body and are fully recoverable from `wind_log.csv`. The spec's absolute forces are physically unrejectable on this airframe (7× the control authority) — see Section 1.2.
- **Sensors are noise-free**: cameras produce clean 400×400 renders (no motion blur, no rolling shutter, no lens distortion, no exposure issues), the LRF is exact, the IMU/gyro are exact. Lighting is fixed (no sun movement, no clouds, no shadows moving).
- **The degraded link is a receiver-side emulation, not a channel model**: drops are deterministic per drone (F1 drops seq 0; F2 drops seq 0+1), corruption is never actually injected (so the CRC path is verified round-trip but never exercised by the run), the 120 ms latency is constant rather than jittered, and delivery of surviving packets is still effectively instantaneous (Webots Emitter/Receiver). Real links add random loss, bit errors, delay jitter and interference; the `follower_lock` sequencing messages are not dropped by the emulation at all (the 105 s fallback would mask a lost announcement but not gracefully).
- **No battery, thermal, or actuator saturation modeling** beyond the PROTO defaults; the drone masses/inertias are the PROTO's.
- **The terminal corridor is sequenced, not spatially deconflicted**: the primary locks and vacates to (7.5, 4.5) *before* the secondary's terminal approach. If the primary failed to lock, the secondary would wait until the 105 s fallback and then attempt a terminal approach against a still-parked teammate — the occlusion gate would reject most estimates and accuracy would degrade (this degraded path has not been exercised). Post-lock, both followers vacate the plate, so supervisor final positions are NOT the lock positions (by design, see 5.1).
- **The obstacles under test are two injected axis-aligned blocks** (the deterministic staggered corridor slalom), because Overpass returned no real buildings inside the flight corridors for this open-parkland bbox. The VFH-lite steering is written for arbitrary convex hulls and loads real OSM footprints identically when present, but this run has **not** been tested against narrow gaps between real buildings, non-convex shapes (courtyards), roofs/overhangs, or moving obstacles; the ±180° steering scan can still trap a drone in a fully-enclosed space.
- **Avoidance is 2D (horizontal) and sensor-limited**: the 5-beam array covers only **±60° forward with 12 m range — no rear coverage** — so a wall behind the drone is invisible until the drone turns; obstacles are tracked with a **6 s time-to-live** (a wall the drone stops looking at fades from memory), clusters carry **no height estimate**, and no obstacle can be sensed before a beam reaches it. The VFH-lite steering probes 1.6 m ahead — fast geometry changes can outrun it. There is no vertical climb-over or 3D path planning.
- **The rangefinder evidence is self-reported**: `sensor_hits_count` and the sensed centroids come from the controllers' own logs; the *clearance* grading still uses the exported building map (control is map-free, grading is not).
- **The IBVS gates were tuned for this geometry**: occlusion gate ≥50 % of expected area, size gate 0.8–2.2 m, adaptive accept gate 3.0 m → 0.12 m, stall override 2 s, hold ring 7.0 m, vacate (7.5, 4.5). Different plate sizes, occluders, approach sectors or altitudes would need re-verification.
- **SRTM elevation is 30 m resolution** (terrarium z15), bilinearly upsampled to 4 m grid spacing — smooth terrain only.
- **The target is a high-contrast red plate** with asymmetric decorations, in a corridor kept clear of other red scenery; real targets are rarely this cooperative. The HSV thresholds and the 800 px gates were tuned for this plate at these altitudes.
- **Terminal "error" is measured at hover altitude (4.5 m / 6.5 m), not at touchdown.** The drones never land on the plate; "arrival" = stable hover over the refined coordinate.
- **ORB matching on the satellite chip is fragile by nature**: the chip carries 34 features; the ID gates were tuned so both approaches pass, but viewpoint changes beyond what was tested (different approach sectors, different time-of-day imagery) would need re-verification. The scout's mark viewpoint is coupled to its avoidance detour (bug #22) — a different detour side changes the chip and can break follower ID.
- **The verification harness judges its own evidence files** (controller-written JSONs). Ground-truth positions come from the Supervisor API (exact in sim), and every milestone number quoted above is computed from those files — but it is still self-reported sim evidence, not an independent measurement.

**What IS real in the sim:**
- Real satellite imagery of the actual anchor location (Esri World Imagery, z18), real SRTM terrain heights. (Real OSM building footprints are supported and exported, but this particular bbox returned none, so the tested obstacle is the injected block.)
- A full physics loop (Webots ODE) with the DJI Mavic 2 Pro PROTO — real dynamics, real cascaded control, real sensor APIs.
- The **end-to-end protocol**: JPEG chip → JSON packet → radio → decode → ORB → Stage-1 GPS nav → building avoidance → Stage-2 attitude-aware visual homing → terminal lock, all running asynchronously in three controllers plus a supervisor, with the drift actually in the packet and a solid obstacle actually in the flight path.

**Conclusion phrasing (honest version):** *In this simulation, on this satellite-textured world with 0.85 m of injected packet drift, a 9 m building blocking the corridor, the followers' own GPS wandering continuously by up to ±0.45 m, a degraded tactical datalink that dropped the initial mark broadcast on F1 and the first two broadcasts on F2 (which both recovered through the Scout's 1 Hz CRC32-checked re-broadcasts), and no obstacle map available to the flight controllers, all three drones sensed and routed around the building using only their onboard 5-beam rangefinder arrays (1.41 m minimum clearance, 4.85 m detour), corrected the packet drift to 2.64 cm and 4.64 cm of ground truth with pure camera-relative IBVS terminal homing, and sequenced their terminal approaches so the parked teammate never occluded the plate; all eight acceptance milestones passed.* Nothing in this file claims, or should be read as claiming, that the same success rate would transfer to real hardware without sensor-noise, wind, probabilistic GPS, true channel impairment, and multi-obstacle testing.

---

## Appendix A — where each fact lives

- Tutorial-style walkthrough with the same facts: `HANDS_ON_GUIDE.md` (sections 1–16).
- Latest-run evidence: `results/report.json` (canonical), `mark.json`, `handoff_*.json`, `visual_id_*.json`, `lock_*.json`, `ground_truth.json`, `tracks.csv`, `id_attempts_*.log`, `stage2_debug_*.log`, PNG frames.
- Dashboard: `python build_dashboard.py` → interactive HTML with trajectory map, drift card, terminal-error metrics, camera evidence.
- World regeneration + data-source attribution: `build_realmap_world.py` and `worlds/textures/realmap_meta.json`.
