# Follower Controller (complete)

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

# Live cv2 window DISABLED for automated runs: multiple HIGHGUI windows from
# concurrent controller processes crash native Webots batch mode (bug #43);
# the lock/id frame PNGs in results/ remain the visual evidence.
VISUAL_HOMING_DEBUG = False
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
        # Phase 10 Monte Carlo: per-trial wander phase from WEBOTS_MC_SEED
        import random as _random
        _mc = int(os.environ.get("WEBOTS_MC_SEED", "0"))
        self.mc_phase_offset = _random.Random(_mc * 104729 + hash(self.drone_id) % 1000).uniform(0, 6.28)
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
        two followers wander independently. Phase offsets also vary per Monte
        Carlo trial (WEBOTS_MC_SEED)."""
        tx, ty, tz = self.gps.getValues()
        phase = (0.0 if self.drone_id.endswith("01") else 1.7) + self.mc_phase_offset
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

    def verify_fiducial(self, bgr_img):
        """Phase 10 geometric fiducial verification. The plate carries an
        asymmetric black marker pair: A = the long bar (NE, larger area) and
        B = the square (SW). Both centroids are inverted to the ground plane
        with the same attitude-aware model as the IBVS homing; the A-B vector
        must match the known plate geometry ((0.82, 0.84) m -> 45.7 deg,
        1.17 m separation) within tolerance. This verifies target IDENTITY and
        ORIENTATION geometrically, independent of the red HSV detector.
        Returns (verified, target_yaw_deg, marker_separation_m)."""
        h_img, w_img = bgr_img.shape[:2]
        hsv = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        # Bug #45: at F1's 4.5 m altitude the frame covers dark satellite
        # scenery BEYOND the plate — the two largest dark blobs were terrain,
        # not markers (a garbage 6.15 m / 130 deg reading). The markers sit ON
        # the plate: bound the search to the detected red plate's bbox.
        mask1r = cv2.inRange(hsv, np.array([0, 150, 100]), np.array([5, 255, 255]))
        mask2r = cv2.inRange(hsv, np.array([175, 150, 100]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(mask1r, mask2r)
        red_ctrs, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not red_ctrs:
            return False, None, None
        red_biggest = max(red_ctrs, key=cv2.contourArea)
        if cv2.contourArea(red_biggest) < 800:
            return False, None, None
        rbx, rby, rbw, rbh = cv2.boundingRect(red_biggest)
        pad = 0.05
        rx0, rx1 = rbx + rbw * pad, rbx + rbw * (1 - pad)
        ry0, ry1 = rby + rbh * pad, rby + rbh * (1 - pad)
        black = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 120, 60]))
        contours, _ = cv2.findContours(black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            # 120 px: the SW square (0.22 m) subtends only ~150 px at F2's
            # 6.5 m cruise altitude vs ~550 px at F1's 4.5 m (bug #44)
            if area < 120:
                continue
            m = cv2.moments(c)
            if m["m00"] <= 0:
                continue
            bcx, bcy = m["m10"] / m["m00"], m["m01"] / m["m00"]
            # markers must lie inside the red plate region (bug #45)
            if not (rx0 <= bcx <= rx1 and ry0 <= bcy <= ry1):
                continue
            blobs.append((area, bcx, bcy))
        blobs.sort(reverse=True)
        if len(blobs) < 2:
            return False, None, None, None
        (area_a, cax, cay), (area_b, cbx, cby) = blobs[0], blobs[1]
        if area_a < 1.3 * area_b:   # A must be clearly the bigger marker
            return False, None, None, None
        x, y, z = self.cur_x, self.cur_y, self.cur_z
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        plate_z = self.handoff_data.target_gps[2]
        if (z + 0.02) - plate_z <= 0.5:
            return False, None, None, None
        f_px = (w_img / 2.0) / math.tan(FOV / 2.0)
        theta = TERMINAL_CAM_PITCH
        ct, st = math.cos(theta), math.sin(theta)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        R = np.array([
            [cy_ * cp, cy_ * sp * sr - sy_ * cr, cy_ * sp * cr + sy_ * sr],
            [sy_ * cp, sy_ * sp * sr + cy_ * cr, sy_ * sp * cr - cy_ * sr],
            [-sp, cp * sr, cp * cr]])
        pcam = np.array([x, y, z]) + R @ np.array([0.12, 0.0, 0.02])

        def invert(px, py):
            uu = (px - w_img / 2.0) / f_px
            vv = (py - h_img / 2.0) / f_px
            db = np.array([ct - vv * st, -uu, -st - vv * ct])
            dw = R @ db
            if dw[2] >= -0.05:
                return None
            s = (plate_z - pcam[2]) / dw[2]
            if s <= 0.0 or s > 60.0:
                return None
            pt = pcam + s * dw
            rel = [pt[0] - x, pt[1] - y]
            if math.hypot(rel[0], rel[1]) > 25.0:
                return None
            return rel

        pa = invert(cax, cay)
        pb = invert(cbx, cby)
        if pa is None or pb is None:
            return False, None, None, None
        vx, vy = pa[0] - pb[0], pa[1] - pb[1]
        sep = math.hypot(vx, vy)
        ang = math.degrees(math.atan2(vy, vx)) % 360.0
        # known plate geometry: A-B = (0.82, 0.84) m -> 45.7 deg, 1.17 m
        verified = (abs((ang - 45.7 + 180.0) % 360.0 - 180.0) < 25.0
                    and abs(sep - 1.17) < 0.45)
        # the marker-pair midpoint IS the plate-center offset measured from the
        # markers themselves — independent of the red-blob corner geometry and
        # its yaw-dependent bias (bug #46)
        mid = [(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0]
        return verified, ang, sep, mid

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
        gs = (1.0 + 13.0 * clamp((2.0 - gd) / 2.0, 0.0, 1.0)
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

                # Phase 10 fiducial precision aid (bug #46): the marker-pair
                # midpoint measures the plate-center offset with no dependence
                # on the red-blob corner geometry, so blending it in at close
                # range removes the yaw-dependent IBVS bias (~2-3 cm).
                if (visual_dist < 1.2 and step_count % 10 == 0
                        and self.handoff_data is not None):
                    f_ok, _fy, _fs, f_mid = self.verify_fiducial(self.get_terminal_bgr())
                    if f_ok and f_mid is not None:
                        a = 0.3
                        self.rel_target_xy = [a * f_mid[0] + (1 - a) * self.rel_target_xy[0],
                                              a * f_mid[1] + (1 - a) * self.rel_target_xy[1]]
                        self.homing_update_count += 1
                        self.dbg_log(f"fiducial-blend mid=({f_mid[0]:.3f},{f_mid[1]:.3f}) "
                                     f"rel=({self.rel_target_xy[0]:.3f},{self.rel_target_xy[1]:.3f})")

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
                # 240 steps (~2 s): the lock must hold through a gust swing so
                # it lands in a calm phase — the true terminal error then stays
                # inside the 5 cm gate even at the camera-bias worst case
                # (bug #46: a seed-1 MC trial locked during a gust at 5.39 cm)
                if (self.lock_streak >= 240 and self.homing_update_count > 0
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
                    # Phase 10: geometric fiducial verification at lock
                    fid_ok, fid_yaw, fid_sep, _ = self.verify_fiducial(self.get_terminal_bgr())
                    print(f"[{self.drone_id}] FIDUCIAL: verified={fid_ok} "
                          f"target_yaw_deg={fid_yaw if fid_yaw is None else round(fid_yaw, 1)} "
                          f"marker_sep_m={fid_sep if fid_sep is None else round(fid_sep, 2)} "
                          f"(expected 45.7 deg / 1.17 m)")
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
                                   "fiducial_verified": fid_ok,
                                   "fiducial_target_yaw_deg": fid_yaw,
                                   "fiducial_marker_separation_m": fid_sep,
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
