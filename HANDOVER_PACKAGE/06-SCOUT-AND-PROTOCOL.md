# Scout Controller + Handoff Protocol

## scout_controller.py
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

## handoff_protocol.py (both copies identical)
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
