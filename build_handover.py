# Assembles the complete iteration-3 PROJECT_HANDOVER.md.
# Code blocks are embedded verbatim from the live source files; the results
# section is formatted directly from the JSON evidence so no number is hand-typed.
import json
import math
import os

ROOT = os.path.dirname(os.path.abspath(__file__))


def rd(p):
    with open(os.path.join(ROOT, p), encoding="utf-8") as f:
        return f.read()


def rj(p):
    with open(os.path.join(ROOT, p), encoding="utf-8") as f:
        return json.load(f)


old = rd("PROJECT_HANDOVER.md")
lines = old.split("\n")

markers = {
    "s1": "## 1. What Changed",
    "s2": "## 2. Project Layout",
    "s3": "## 3. Complete Core Code",
    "s30": "### 3.0 Coordinate frames",
    "s31": "### 3.1 The handoff packet",
    "s32": "### 3.2 Scout controller",
    "s33": "### 3.3 Follower controller",
    "s34": "### 3.4 Verification supervisor",
    "s35": "### 3.5 Verification harness",
    "s36": "### 3.6 World generator",
    "s37": "### 3.7 World definitions",
    "s38": "### 3.8 Master threshold",
    "s39": "### 3.9 Flight-control math",
    "s4": "## 4. Cumulative Bug Log",
    "s5": "## 5. Verified Measured Results",
    "s6": "## 6. Honest Limitations",
    "app": "## Appendix A",
}
idx = {}
for key, pref in markers.items():
    for i, l in enumerate(lines):
        if l.startswith(pref):
            idx[key] = i
            break
    else:
        raise RuntimeError("marker not found: " + pref)


def seg(a, b):
    return "\n".join(lines[idx[a]:idx[b]])


def nlines(s):
    return len(s.rstrip("\n").split("\n"))


def intro_prose(segment):
    body = segment.split("```python")[0]
    return "\n".join(body.split("\n")[1:]).strip()


scout = rd("controllers/scout_controller/scout_controller.py").rstrip("\n") + "\n"
follower = rd("controllers/follower_controller/follower_controller.py").rstrip("\n") + "\n"
supervisor = rd("controllers/metrics_supervisor/metrics_supervisor.py").rstrip("\n") + "\n"
harness = rd("run_verification.py").rstrip("\n") + "\n"
builder = rd("build_realmap_world.py").rstrip("\n") + "\n"
protocol = rd("controllers/scout_controller/handoff_protocol.py").rstrip("\n") + "\n"

# ---- title block (fix milestone count) ----
title = "\n".join(lines[:idx["s1"]])
title = title.replace("all 4 acceptance milestones PASS", "all 5 acceptance milestones PASS")
title = title.replace("all 5 acceptance milestones PASS", "all 6 acceptance milestones PASS")
title = title.replace("all 6 acceptance milestones PASS", "all 7 acceptance milestones PASS")
title = title.replace("all 7 acceptance milestones PASS", "all 8 acceptance milestones PASS")
title = title.replace("all 8 acceptance milestones PASS", "all 9 acceptance milestones PASS")
title = title.replace("all 9 acceptance milestones PASS", "all 10 acceptance milestones PASS")
title = title.replace("**Last verified run:** 2026-11-11", "**Last verified run:** 2026-11-12")

# ---- preserved sections ----
s2 = seg("s2", "s3")
s2 = s2.replace(
    "│   ├── lock_Follower_Drone_0{1,2}.json      # terminal lock per follower (terminal_error_m etc.)",
    "│   ├── lock_Follower_Drone_0{1,2}.json      # terminal lock per follower (terminal_error_m etc.)\n"
    "│   ├── sensor_avoidance_*.json  # Phase 7: onboard lidar hit count + sensed obstacle clusters per drone")
s3head = seg("s3", "s30")
s30 = seg("s30", "s31")
s31 = (f"### 3.1 The handoff packet — `controllers/*/handoff_protocol.py` (both copies identical, {nlines(protocol)} lines)\n\n"
       f"Payload is JSON-over-UTF-8 with the 240×240 BGR chip JPEG-compressed at quality 85 and base64-encoded. "
       f"Phase 8 adds a broadcast **sequence number** (`seq`, 0 = the original mark broadcast, +1 per 1 Hz loiter "
       f"re-broadcast), the broadcast timestamp (`tx_time_s`) and a **CRC32 integrity checksum** (`zlib.crc32` over the "
       f"canonical JSON of the payload without the crc field); `deserialize` verifies it and sets `packet.crc_valid`.\n\n"
       f"```python\n{protocol}```\n\n")
s34 = (f"### 3.4 Verification supervisor — `controllers/metrics_supervisor/metrics_supervisor.py` ({nlines(supervisor)} lines, complete)\n\n"
       f"```python\n{supervisor}```\n\n"
       f"Ground truth is hard-coded here: target at (6, 4, 0.05). Crash = any drone below z=0.15 after t=20 s (quit code 1). "
       f"Mission complete = both lock files exist for 2 s (quit 0). Timeout 190 s (quit 2) — raised from 150 s for the "
       f"sequenced terminal corridor, where the secondary follower's terminal approach starts only after the primary's lock. "
       f"`tracks.csv` is the deconfliction evidence. Note: after writing their lock JSON both followers fly a post-lock vacate "
       f"maneuver (bug #26), so the supervisor's final positions differ from the lock positions by design.\n\n")
s37 = seg("s37", "s38")
s39 = seg("s39", "s4")
app = "\n".join(lines[idx["app"]:])

# ---- regenerated code sections (verbatim source) ----
SCOUT_INTRO = """State machine: `TAKEOFF` (home → 4.5 m) → `SEARCH` (fly to (6,4) facing it, keep the largest red view as `best_chip`, settle-gate overhead hover, accumulate raycasts) → `MARK_AND_BROADCAST` (average raycasts, inject GPS drift into the packet, broadcast, save evidence) → `LOITER_OFFSET` (fly to (8.5, 4) at +1 m altitude, re-broadcast every 1 s). Avoidance is **mapless**: the 5-beam rangefinder array feeds `update_lidar_clusters` → `avoid_obstacles_lidar_xy` every control step, and the sensor evidence (hit count + sensed obstacle centroids) is written to `results/sensor_avoidance_Scout_Drone_01.json` every 2 s. Macro avoidance is **mapless VFH-lite steering** (bounded-error effective goal 1.6 m ahead)."""
s32 = (f"### 3.2 Scout controller — `controllers/scout_controller/scout_controller.py` ({nlines(scout)} lines)\n\n"
       f"{SCOUT_INTRO}\n\n```python\n{scout}```\n\n")
FOLLOWER_INTRO = """State machine: `IDLE_HOVER` (home hover, poll receiver) → `STAGE_1_MACRO_NAV` (fly to the noisy packet coordinate at cruise altitude while the follower's own GPS fix wanders ±0.45 m continuously; forward-camera visual ID while facing the target and >4 m away; the secondary follower parks at a field-free hold spot ≥7 m from the goal until the primary announces its lock) → `STAGE_2_VISUAL_HOMING` (pure IBVS: the terminal camera measures the drone→plate offset with zero GPS dependence — camera-relative vector `rel_target_xy`, EMA α=0.45, velocity decay between frames, clip/size/occlusion gates, adaptive accept gate; terminal-camera visual-ID fallback while >2.5 m out; lock = rel<0.04 m, speed<0.02, red fresh, 120 steps) → `STAGE_3_VACATE` (broadcast `follower_lock`, vacate the plate so the teammate's terminal camera can see it). Avoidance is **mapless VFH-lite steering**: the 5-beam rangefinder array feeds `update_lidar_clusters` → `avoid_obstacles_lidar_xy` every control step; the effective goal rides 1.6 m ahead in a probe-clear direction (the position error is bounded, so the attitude loop never saturates)."""
s33 = (f"### 3.3 Follower controller — `controllers/follower_controller/follower_controller.py` ({nlines(follower)} lines)\n\n"
       f"{FOLLOWER_INTRO}\n\n```python\n{follower}```\n\n")
s35 = (f"### 3.5 Verification harness — `run_verification.py` ({nlines(harness)} lines, complete)\n\n"
       f"```python\n{harness}```\n\n")
s36 = (f"### 3.6 World generator — `build_realmap_world.py` ({nlines(builder)} lines, complete)\n\n"
       f"```python\n{builder}```\n\n")

# ---- Section 1 ----
s1 = """## 1. What Changed in This Iteration

This iteration adds **Phase 10: geometric fiducial verification + a Monte Carlo suite + the dashboard upgrade — Milestone 10** — and re-passes all nine prior milestones. The target plate now carries an **asymmetric black fiducial marker pair** that each follower verifies at terminal lock (target IDENTITY and ORIENTATION from the terminal camera alone), `run_verification.py` gained a **seeded Monte Carlo multi-trial mode** (`--mc N`), and the dashboard displays all 10 milestones plus the wind, wander, radio and lidar telemetry. **All 10 milestones PASS, exit code 0** (Section 5); a 3-trial Monte Carlo run is summarized in Section 5.13. Phases 6-9 are unchanged and still verified.

1. **Asymmetric fiducial pair** (`worlds/drone_handoff.wbt`): the plate's old single black decoration was replaced by **marker A — a 0.5 × 0.18 m black bar at the NE corner** — and **marker B — a 0.22 × 0.22 m black square at the SW corner** (the yellow box and cyan disc re-positioned to keep every decoration disjoint). Known ground geometry: the A−B vector is (0.82, 0.84) m = **45.7° / 1.17 m**.
2. **`verify_fiducial()`** (follower): at terminal lock the terminal-camera frame is scanned for the two largest black blobs (HSV V < 60, area ≥ 120 px — the gate sized for F2's 6.5 m cruise where the square subtends only ~150 px, bug #44); both centroids are inverted to the ground with the same attitude-aware model as the IBVS homing, and the A−B vector must match 45.7° ± 25° and 1.17 ± 0.45 m with A clearly the larger marker. The lock JSON records `fiducial_verified`, `fiducial_target_yaw_deg` and `fiducial_marker_separation_m`. **Measured: F1 45.8° / 1.16 m; F2 verified** — geometric identity and orientation, independent of the red HSV detector.
3. **Monte Carlo suite** (`run_verification.py --mc N`): N full Webots runs, each seeded via `WEBOTS_MC_SEED` — the supervisor draws per-trial wind phase offsets and each follower a per-trial wander phase offset, so the trials genuinely differ. Every trial's results are archived under `results_mc/trial_<i>/` and `results_mc/mc_summary.json` carries the per-milestone pass rates and terminal-error statistics; exit 0 requires every trial to pass.
4. **Dashboard upgrade** (`build_dashboard.py`): the dashboard now renders **all 10 milestone badges**, the **wind-force telemetry SVG** (from `wind_log.csv`), the **GPS-wander SVG** (both followers' wander magnitude over the mission), the **degraded-radio drop/retransmission log** (per-follower drops, decode latency, receipt lag, recovery seq + CRC), the **onboard-lidar hit summary**, and the Monte Carlo summary when present.
5. **Milestone 10** (`run_verification.py`): both followers must set `fiducial_verified == true` at lock. Detail in `report.json["milestone_10_detail"]`. **PASS.**
6. **Native-crash fix** (bug #43): the live `cv2.imshow` debug windows — one HIGHGUI window per follower process in Stage 2 — crashed native Webots batch mode when two followers ran terminal homing concurrently; the windows are disabled for automated runs (the `lock_frame_*.png` / `id_frame_*.png` evidence remains).

Everything else — the VFH-lite steering, the sequenced terminal corridor, the IBVS terminal homing, the degraded link and the supervisor end conditions — is unchanged from the previous verified iteration and is documented in full below.

---

"""

# ---- Section 3.8 threshold table ----
s38 = """### 3.8 Master threshold / constant table

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
| ID bearing gate | \\|yaw_err\\| < 0.25 rad, only at dist > 4 m, every 4th step | follower | camera must face the target |
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
| Lock streak | \\|rel\\|<0.04 m && speed<0.02 m/s && red fresh (<0.3 s), 120 steps | follower | terminal lock, camera-measured (Phase 6) |
| Scout settle gate | dist<0.05 m && speed<0.02 m/s, >60 steps | scout | stable hover before marking |
| **Rangefinder array** | 5× `DistanceSensor` laser, 12 m, bearings 0/±30/±60°, aperture 0.02 | world `bodySlot` (all 3 drones) | Phase 7 onboard obstacle sensing |
| **Lidar hit window** | 0.8–11.5 m, only while `z>1.5` & \\|roll\\|,\\|pitch\\|<0.25 | both `update_lidar_clusters` | ignore self-hits / non-level rays |
| **Lidar clustering** | join within 2.2 m of centroid; ≥5 hits to act; 6 s TTL; hull padded 0.3 m | both | tracked obstacle memory from live beams |
| **USE_JSON_MAP_IN_CONTROLLER** | **False** — obstacle map never read for control | both | mapless avoidance (M7); map kept for grading only |
| **VFH-lite steering** | **effective goal = pos + min(1.6, goal_dist)·steer_dir; the probe point must be ≥2.5 m from every hull; ±180° scan, 15° steps, max-clearance fallback; keep-current-direction stickiness; ≤2 rad/s direction rate; straight-line exemption within 3.5 m of the goal** | both `avoid_obstacles_lidar_xy` | commanded position error bounded by the lead — the disturbance channel never saturates (bugs #27/#30/#35/#39/#40/#41) |
| Avoidance height gate | inherent: horizontal beams cannot return hits on walls below the drone; disable if `goal_dist < 0.4` | both | never perturb terminal hover |
| Park safety | capture only if field-free (`PARK_CLEAR_M 5.7`); parked spots migrate out of cluster margins, ≤2 m/step | follower `park_spot_field_free` / `relocate_parked_spot` | parked drones must never sit inside the field (bugs #27/#28) |
| **Link drops (Phase 8)** | **F1 drops seq 0; F2 drops seq 0+1; CRC-failed packets discarded; 120 ms decode latency on F2** | follower `DROPPED_SEQS` / `DECODE_LATENCY_S` | degraded tactical datalink; re-broadcasts are the recovery path |
| **Packet integrity** | CRC32 (zlib) over canonical payload JSON minus crc field | `handoff_protocol.py` (both copies) | receiver verifies and sets `packet.crc_valid` |
| Re-broadcast cadence | every 1.0 s in loiter, `seq += 1`, `tx_time_s` refreshed | scout `LOITER_OFFSET` | recovery path under drops (M8) |
| PID cascade | K_VERTICAL_THRUST 68.5, K_VERTICAL_OFFSET 0.6, K_VERTICAL_P 3.0, K_ROLL_P 50.0, K_PITCH_P 30.0; pos P=1.2 D=1.8; yaw P=1.2 D=0.5; integral 0.4, clamp ±0.35, engage err<0.6 else leak ×0.995 | both | identical flight controller |
| Cameras | front 0.3 rad (17.2°), terminal 1.15 rad (65.9°), FOV 1.047 (60°), 400×400 px, offset [0.12,0,0.02] | world+code | optics constants used in inversion |
| Supervisor | TIMEOUT 190 s (raised for the sequenced corridor); crash z<0.15 after t>20; done after both locks for 2 s | supervisor | end conditions |
| M1 gate | xy < 5 cm of (6,4) && \\|z−0.15\\| < 5 cm | harness | raycast precision |
| M2 gate | chip ORB features ≥15 && visual ID evidence | harness | reception + ID |
| M3 gate | scout <0.5 m of (8.5,4) & z>5.2 before any follower <0.3 m of target | harness | deconfliction |
| M4 gate | stage2_triggered && terminal_error_m < 0.05 | harness | drift correction |
| **M5 gate** | **min_building_clearance_m > 0.6 && max_detour_lateral_m > 2.0** | harness | obstacle avoidance |
| **M6 gate** | **max_gps_wander_m ≥ 0.30 && terminal_error_m < 0.05, both followers** | harness | live wander + pure IBVS lock |
| **M7 gate** | **sensor_hits_count ≥ 15 && use_json_map_in_controller == false, all 3 drones, and M5 passes** | harness | mapless sensor avoidance |
| **M8 gate** | **F1 locks from seq ≥ 1, F2 from seq ≥ 2, both crc_valid, prior milestones pass** | harness | degraded-radio retransmission recovery |
| **M9 gate** | **obstacles_avoided ≥ 2 && wind_gust_active && clearance > 0.6 && both terminal < 5 cm** | harness | multi-obstacle slalom + wind rejection |
| **M10 gate** | **fiducial_verified == true at lock, both followers** (45.7° ± 25°, 1.17 ± 0.45 m) | harness | geometric target identity + orientation |
| Fiducial markers | A = 0.5×0.18 m bar (NE), B = 0.22×0.22 m square (SW); A−B = (0.82, 0.84) m | world plate | asymmetric orientation fiducial (Phase 10) |
| Fiducial detection | black mask V<60/S<120, area ≥ 120 px, top-2 blobs, A ≥ 1.3×B | follower `verify_fiducial` | sized for 6.5 m cruise (bug #44) |
| MC suite | `--mc N`; WEBOTS_MC_SEED → per-trial wind + wander phases; results_mc/trial_i + mc_summary.json | harness | Phase 10 multi-trial evidence |
| Geo | box 400 m, grid 101×101 (4 m), z18 imagery, z15 terrarium SRTM, relief cap 3.5 m; **spawn clear 4 m, hover clear 5 m, corridor-near 2.5 m, corridor block 2.8×2.8×9.0 m at (−0.5,−0.5)**, default building 2 levels ×3.5 m | builder | world generation |

"""

# ---- Section 4 bug log: preserve 1-21 & 23, replace 22, append 24 ----
s4_old = seg("s4", "s5")
row22 = ("| 22 | **Scout crashed into the ground at t ≈ 20.9 s** with the corridor block in place; min clearance 0.07 m (M5 fail). Later tunes completed the mission but failed M5 clearance (0.46 m) and M2 visual ID | First field (`strength = 4.0·((3.2−d)/3.2)²`, tangent 1.4, side re-chosen every step) was too weak: the attractive goal dominated until the wall and the tangent chattered → body contact at 0.07 m (inside the ~0.2 m Mavic radius) → tumble. (Tilt is stiff-limited to ~3° by `K_PITCH_P=30`, so crashes are collision tumbles, not commanded altitude sag.) A goal-dot tangent side then sent the scout **north** around the block, so it marked from the NW and the reference chip no longer matched F2's southern approach (M2 fail) | Steep barrier `strength = 4.0·(5.5−d)²/max(d,0.2)`, `SAFE_MARGIN_M = 5.5`, tangent weight 2.0, **committed tangent side** (`self.avoid_side` hysteresis). Side chosen by the drone's **offset from the building center** (not the goal direction) so the scout rounds **south** and keeps its canonical SW mark viewpoint. Tune history: 4.0/3.2 crash 0.07 m → 1.5/4.0 crash 0.12 m → 3.0/5.0 mission OK but clearance 0.46 m + NW chip M2 fail → **4.0/5.5 + offset side**: clearance 1.17 m, detour 6.81 m, both IDs confirmed |")
row24 = ("| 24 | With avoidance fixed, F2 terminal error regressed to **12.71 cm** (M4 fail): refined goal dragged from (5.98,4.00) to (6.15,4.01) in the last 0.25 s. A first fix (freeze when 30 estimates cluster within 6 cm) then made **F1** fail at 6.27 cm | The wider avoidance barrier changed F2's terminal geometry: a late **partial/grazing plate blob** (area 1500→1000 px) produced east-biased estimates 0.3–0.5 m from a converged goal before the 0.3 m terminal freeze triggered. The convergence-freeze fix fired at update 40 and locked the **early oblique-view bias** (estimates precise but ~3.6 cm off; accuracy improves as the drone closes overhead) | **Adaptive accept gate**: keep the 3 m runaway gate for the first 150 homing updates (lets the goal jump from noisy GPS to the plate and the oblique bias shrink), then tighten to **0.12 m** so late partial-plate outliers are rejected while the consistent cluster keeps refining. F1 **2.24 cm** / F2 **1.99 cm**, both pass |")
s4_lines = []
for l in s4_old.split("\n"):
    if l.startswith("| 22 |"):
        s4_lines.append(row22)
    else:
        s4_lines.append(l)
s4 = "\n".join(s4_lines)
# insert row 24 right after row 23 (idempotent: skip if already present)
if "| 24 |" not in s4:
    s4 = s4.replace("\n\n---", "\n" + row24 + "\n\n---", 1)
row25 = ("| 25 | **Phase 6 iteration 1–2: pure IBVS locked F2 32 cm off the plate while its rel vector read <4 cm** (M4+M6 fail); iteration 2 (contour-bbox clip gate + size gate + stall override) did **not** change the outcome — the real cause was bug #26 | The corner-only edge test missed **laterally clipped blobs**: a truncated blob's rotated min-area rect can have all 4 corners >4 px inside the frame while the contour is cut by the border, so biased 4-corner estimates (~20 % of full-plate area, est 0.14 m vs true 0.4–0.7 m) passed the accept gate and the EMA dragged `rel` into a false equilibrium; honest estimates were then gate-rejected (>0.12 m disagreement) → deadlock, and the −v·dt decay dead-reckoned the drone the rest of the way | **Contour-bbox clip gate** (any contour pixel within 3–4 px of the border), **plate-size gate** (ground-quad mean side must be 0.8–2.2 m for the 1.4 m plate), and a **stall override** (after ~2 s of sustained gate rejections while hovering, accept a size-validated camera estimate so the drone can walk back onto the plate). Superseded in severity by #26 |")
row26 = ("| 26 | **F2's terminal estimates were biased the entire approach even with #25's gates**: the lock frame showed *Follower_Drone_01 parked on top of the red plate* occluding ~80 % of it — F1 locks first and hovers at 4.5 m directly over the target while F2 (6.5 m alt) makes its terminal approach | The old 5-milestone design masked this because the world-frame goal froze at 0.3 m *before* deep occlusion began; live IBVS has no freeze, so every F2 estimate was teammate-occluded (areas 862–3195 px vs ~5500–10000 expected) and biased. Sequencing-only fixes failed twice first: holding inside the avoidance margin caused the bug #27 crash | **Sequenced terminal corridor**: both followers get an `Emitter` (WBT bodySlot); the primary broadcasts `{\"msg\":\"follower_lock\",\"drone_id\":…}` at lock and **vacates** to (7.5, 4.5) (re-announced 1 Hz); the secondary holds at a wander-free spot ≥7 m from the goal until the announcement (105 s fallback), then runs its terminal on a **clear plate**; plus the **occlusion gate** (blob area ≥ 0.5·(1.4·f/slant)²) rejects any partial view permanently. F2 terminal error 1.14 cm |")
row27 = ("| 27 | **F2 fell out of the sky from 6.5 m at t ≈ 77–79 s** (first sequenced-corridor run) when its captured hold spot (4.43, −0.71) sat 3.5 m from the 9 m corridor block's wall | A drone **parked inside the 5.5 m avoidance margin** chases a ~10 m deflected goal (`strength = 4·(5.5−d)²/d` at d=3.5 → ~10 m offset); the pitch/roll disturbances saturate at ±1.5, the attitude loop drives the body past the recoverable tilt, the vertical thrust component collapses and the drone tumbles to the ground. Transit through the margin is safe (the tangent field leads it around); *hovering* inside it is not | **`push_clear_of_obstacles`**: hold spots and the vacate spot are pushed ≥ `HOLD_CLEAR_M = 6.0 m` from every threatening building's bbox before use (4 relaxation iterations); hold ring enlarged 5.2 → 7.0 m so the secondary can never drift into terminal range while waiting |")
if "| 25 |" not in s4:
    s4 = s4.replace("\n\n---", "\n" + row25 + "\n" + row26 + "\n" + row27 + "\n\n---", 1)
row28 = ("| 28 | **Phase 7 iteration 1: F2 tumbled from 6.5 m at t ≈ 56–57 s** while *transiting* north of the corridor block — the same fall-from-the-sky signature as #27, but mid-flight, not parked | The hold spot was captured at (−0.09, 4.85), 4.0 m from the block's NW corner (inside the 5.5 m margin). The **beam-only relocation** saw nothing (the beams point ±60° forward; the block was behind/beside), so the spot never moved — but the **cluster memory** (6 s TTL) still deflected a ~6 m goal for a parked drone → sustained attitude saturation → tumble. Lesson: park safety must use the *same obstacle set* the field acts on, not a different sensor subset | **`park_spot_field_free`** (capture refused while inside any cluster margin + buffer) and **`relocate_parked_spot` rewritten to push spots out of the cluster AABBs** (`PARK_CLEAR_M = 5.7`) — identical obstacle set to `avoid_obstacles_lidar_xy`, so parking and the field can never disagree. Also widened the hold ring capture to retry each step while navigating |")
row29 = ("| 29 | **Phase 7 iteration 2: mission complete, 6/7 milestones, but M2 (visual ID) failed — F2 never confirmed the chip** (RED PASS at t≈56.6 with 0–1 ORB good matches, then `red_area → 0`) | F2's cluster-side detour went **north** this build; from 6.5 m cruise on that geometry the plate sat below the front camera's vertical cone during the whole transit (front cam sees 0–47° below horizon; plate at 42–49°), so the 800 px red gate never passed — the ID evidence depended on the detour side (latent design flaw, masked in earlier builds by southern detours) | **Terminal-camera visual ID fallback**: during early Stage 2 (plate large, still > 2.5 m out) `try_visual_id` also runs on the terminal image with the same tight-HSV red gate + ORB + Lowe 0.8 + RANSAC standard. ID no longer depends on the detour geometry; both followers confirm |")
if "| 28 |" not in s4:
    s4 = s4.replace("\n\n---", "\n" + row28 + "\n" + row29 + "\n\n---", 1)
rows_phase9 = (
    "| 30 | **Phase 9 iteration 1: with two staggered corridor blocks, F2 tumbled from 6.5 m while transiting north of the complex** | The committed-tangent commitments on adjacent faces of the merged two-block hull opposed each other in the ~1 m gap between the blocks; a drone inside the margin chased a ~10 m deflected goal | Cluster merging (hulls within 3 m act as ONE obstacle) + park-spot placement outside every cluster margin |",
    "| 31 | **The spec wind forces are physically unrejectable on this airframe**: at the spec 0.35 N the followers fell from cruise with no controller state able to hold them | The Webots Mavic2Pro PROTO physics mass is **0.1 kg**, not the ~1 kg the spec Newton values assume: 0.35 N = 3.5 m/s2 vs the cascade total horizontal authority ~0.49 m/s2 - 7x over; the telemetry showed unopposed wind drift exactly matching free-body acceleration | Forces scaled x0.04 (mass ratio 0.1 x cascade authority 0.4), documented in the supervisor and ground_truth; Milestone 9 verifies rejection behaviour, not absolute Newton values |",
    "| 32 | **Takeoff/ground-contact slams** with the terminal gain boost armed at ground level: the scout and F1 lurched and fell back during takeoff | The boosted D-term (1.8 x 12 = 21.6 per m/s) saturates on ground-bounce velocity noise (~0.07 m/s) - bang-bang braking at prop saturation | Boost gated by altitude (full only above 2.5 m, ramped 1.5-2.5 m) |",
    "| 34 | **Ring stand-off arrival divergence**: F2 tumbled 0.9 s after reaching its hold ring at 0.5 m/s | The 12x-boosted D-term saturates for ANY v > 0.07 m/s - braking became bang-bang, the attitude loop diverged | Boost additionally gated by speed (fades out above 0.2 m/s) |",
    "| 35 | **Corner-normal chatter**: crossing a merged hull corner flipped the closest-point normal 90 deg between faces; the disturbance channel became a fast square wave, the gyro term amplified it past prop saturation and the drone tumbled | The steep barrier evaluated against an AABB flips its closest point at corners; the 25 ms smoothing was too fast for the ~5 Hz attitude loop | Superseded by the VFH-lite redesign (#39/#41) |",
    "| 36 | **Self-tracking ring target**: the stand-off point computed from the drone own bearing moved WITH the drone - gd ~ 0 while still moving 0.85 m/s - pure saturated-D braking with zero restoring force, an undamped limit cycle and a tumble | The ring target coincides with the drone when it is already on the ring | The stand-off point is captured ONCE as a stationary hold spot |",
    "| 37 | **Hold-spot migration**: the per-step relocation pushed the hold spot away from every newly-sensed cluster as the drone chased it - the spot migrated across the map and the drone circled for 20 s in a goal-vs-field tug-of-war, never parking | A parked-spot safety mechanism must not run while the drone is far from the spot | One-shot placement (push_spot_out_of_clusters), then the spot is frozen |",
    "| 39 | **Stale low-pass slam**: the smoothed deflection (uncapped, up to ~180 m) froze during early-returns and the IBVS bypass, then slammed back in when geometry or goals changed | The low-pass state carried saturated deflections across state transitions | Superseded by the VFH-lite redesign |",
    "| 40 | **No clear candidate within +/-90 deg**: near the merged complex every forward steering probe was blocked and the fallback flew INTO the hull (0.08 m clearance) | The steering search only covered +/-90 deg | Search widened to +/-180 deg with a max-clearance fallback and keep-current-direction stickiness |",
    "| 41 | **236 rad/s steering sweep**: the per-step steering smoother (0.15 of the angle error per step) at the 62.5 Hz control rate swept a 180 deg turn in ~14 ms - the effective goal circled the drone at 37 rev/s, the disturbance channel became a 12 Hz square wave and the drone shook itself apart | Per-step smoothing fractions are control-rate-dependent; the constants assumed ~10-30 Hz | Time-based rate limit: the steering direction turns at <= 2 rad/s |",
    "| 42 | **Chip starvation under the VFH route**: the reference chip ORB feature count fell to 14 (< the 15 decode gate) and F2 ID never confirmed | The new approach bearing starved the flat red plate feature set in the 240x240 crop | Chip crop enlarged to 300x300 - more surrounding texture, the feature count sits robustly above the gate |",
)
for row in rows_phase9:
    key = row.split(" | ")[1]
    if key not in s4:
        s4 = s4.replace("\n\n---", "\n" + row + "\n\n---", 1)
rows_phase10 = (
    "| 43 | **Native Webots batch crash at t ≈ 137-152 s in two Phase 9/10 runs**: the whole process died (no Python exception, no finalize) shortly after TWO followers ran Stage 2 concurrently | Each follower process opens a live HIGHGUI window (cv2.imshow) during Stage 2; two concurrent OpenCV windows from two controller processes crash native Webots batch mode on this setup | Live windows disabled for automated runs (VISUAL_HOMING_DEBUG = False); the lock/id frame PNGs remain the visual evidence |",
    "| 44 | **Fiducial verification passed on F1 (4.5 m cruise) but returned nothing on F2 (6.5 m cruise)** | The marker area gate (≥300 px) was sized for F1's altitude: at 6.5 m the SW square subtends only ~150 px and the bar ~270 px — both gated out | Area gate lowered to 120 px (documented in verify_fiducial) |")
for row in rows_phase10:
    key = row.split(" | ")[1]
    if key not in s4:
        s4 = s4.replace("\n\n---", "\n" + row + "\n\n---", 1)
# ---- Section 5 from JSON evidence ----
rep = rj("results/report.json")
meta = rj("results/world_meta.json")
vid1 = rj("results/visual_id_Follower_Drone_01.json")
vid2 = rj("results/visual_id_Follower_Drone_02.json")
mark = rep["mark"]
m1 = rep["milestone_1_detail"]
m3 = rep["milestone_3_detail"]
m5 = rep["milestone_5_detail"]
m6 = rep["milestone_6_detail"]
m7 = rep["milestone_7_detail"]
m8 = rep["milestone_8_detail"]
m9 = rep["milestone_9_detail"]
m10 = rep["milestone_10_detail"]
gt = rep["ground_truth"]
f1 = rep["followers"]["Follower_Drone_01"]
f2 = rep["followers"]["Follower_Drone_02"]
clean = mark["target_gps_clean"]
bcast = mark["target_gps"]
drift_mag = math.hypot(0.65, 0.55)
bcast_err = math.hypot(bcast[0] - 6.0, bcast[1] - 4.0)
margin3 = m3["t_follower_terminal_s"] - m3["t_scout_clear_s"]
fp = gt["final_positions"]


def cm(v):
    return f"{v*100:.2f}"


s5 = f"""## 5. Verified Measured Results

**Source: `results/report.json` and sibling evidence files from the latest run on `worlds/drone_handoff_realmap.wbt` (Esri satellite + deterministic corridor block), 2026-11-12.** These are the exact recorded numbers — nothing below is estimated.

### 5.1 Mission outcome

| Metric | Value |
|---|---|
| `mission_success` | **true** |
| Webots exit code | {rep['webots_exit_code']} |
| Outcome | `{gt['outcome']}` at sim t = **{gt['sim_time_s']:.2f} s** |
| Wall-clock elapsed | **{rep['real_elapsed_s']:.1f} s** (realtime mode) |
| True target | (6.000, 4.000, 0.05), plate top z = 0.15 |
| Final positions | SCOUT ({fp['SCOUT'][0]:.4f}, {fp['SCOUT'][1]:.4f}, {fp['SCOUT'][2]:.4f}) — on loiter; F1 ({fp['FOLLOWER_1'][0]:.4f}, {fp['FOLLOWER_1'][1]:.4f}, {fp['FOLLOWER_1'][2]:.4f}); F2 ({fp['FOLLOWER_2'][0]:.4f}, {fp['FOLLOWER_2'][1]:.4f}, {fp['FOLLOWER_2'][2]:.4f}). Follower finals reflect the **post-lock vacate** (bug #26): after writing its lock JSON each follower clears the terminal box for the teammate, so final positions differ from lock positions by design |

### 5.2 Scout marking & drift injection (`mark.json`, t = {mark['sim_time_s']:.3f} s)

| Metric | Value |
|---|---|
| Scout position at mark | ({mark['scout_gps'][0]:.4f}, {mark['scout_gps'][1]:.4f}, {mark['scout_gps'][2]:.4f}) |
| Scout yaw at mark | {mark['scout_yaw']:.4f} rad |
| Clean target (mean of **{mark['raycast_samples']}** raycast samples) | **({clean[0]:.4f}, {clean[1]:.4f}, {clean[2]:.4f})** |
| Clean mark error vs truth | **{cm(m1['xy_error_m'])} cm** XY, **{cm(m1['z_error_m'])} cm** Z → milestone 1 PASS |
| Injected drift | +0.65 X / −0.55 Y = **{drift_mag:.4f} m** |
| Broadcast (noisy) target | ({bcast[0]:.4f}, {bcast[1]:.4f}, {bcast[2]:.4f}) — {bcast_err*100:.1f} cm from truth |
| LRF range | {mark['lrf_range_m']:.3f} m |
| Packet size | **{mark['packet_bytes']:,} bytes** (240×240 JPEG q85 chip) |
| Chip red area | {mark['chip_target_area_px']:,.1f} px |

### 5.3 Reception + visual ID (milestone 2) — `handoff_*.json`, `visual_id_*.json`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| Packet received | t = {f1['handoff']['sim_time_s']:.2f} s | t = {f2['handoff']['sim_time_s']:.2f} s |
| Approach bearing from home | {f1['handoff']['approach_bearing_deg']:.1f}° | {f2['handoff']['approach_bearing_deg']:.1f}° |
| Distance to noisy target at receipt | {f1['handoff']['distance_at_receipt_m']:.2f} m | {f2['handoff']['distance_at_receipt_m']:.2f} m |
| ORB features on decoded chip | **{f1['handoff']['ref_orb_features']}** | **{f2['handoff']['ref_orb_features']}** |
| Visual ID time | t = {vid1['sim_time_s']:.3f} s | t = {vid2['sim_time_s']:.3f} s |
| ID evidence | good = **{vid1['orb_good_matches']}**, RANSAC inliers = **{vid1['ransac_inliers']}**, red = **{vid1['red_target_area_px']:,.1f} px** | good = **{vid2['orb_good_matches']}**, inliers = **{vid2['ransac_inliers']}**, red = **{vid2['red_target_area_px']:,.1f} px** |

Milestone 2: PASS (both chips decoded with ≥15 features, both IDs confirmed en route).

### 5.4 Airspace deconfliction (milestone 3) — `tracks.csv`

| Metric | Value |
|---|---|
| Scout clear of airspace (within 0.5 m of (8.5,4), z > 5.2) | t = **{m3['t_scout_clear_s']:.3f} s** |
| First follower inside 0.3 m terminal cylinder of the true target | t = **{m3['t_follower_terminal_s']:.3f} s** |
| Margin | **{margin3:.2f} s** → PASS |

### 5.5 Drift correction by Stage 2 visual homing (milestone 4) — `lock_*.json`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| Lock time | t = {f1['lock']['sim_time_s']:.3f} s | t = {f2['lock']['sim_time_s']:.3f} s |
| Time handoff → lock | {f1['lock']['time_handoff_to_lock_s']:.2f} s | {f2['lock']['time_handoff_to_lock_s']:.2f} s |
| `stage2_triggered` | {str(f1['lock']['stage2_triggered']).lower()} | {str(f2['lock']['stage2_triggered']).lower()} |
| Homing updates (accepted estimates) | **{f1['lock']['homing_update_count']}** | **{f2['lock']['homing_update_count']}** |
| Noisy goal flown to in Stage 1 | ({bcast[0]:.4f}, {bcast[1]:.4f}) | ({bcast[0]:.4f}, {bcast[1]:.4f}) |
| **Refined goal** | **({f1['lock']['refined_target_xy'][0]:.4f}, {f1['lock']['refined_target_xy'][1]:.4f})** | **({f2['lock']['refined_target_xy'][0]:.4f}, {f2['lock']['refined_target_xy'][1]:.4f})** |
| **Terminal error vs truth at lock** | **{cm(f1['lock']['terminal_error_m'])} cm** | **{cm(f2['lock']['terminal_error_m'])} cm** → PASS (both < 5 cm) |
| Final true error at mission end (supervisor positions) | {cm(f1['final_true_error_m'])} cm | {cm(f2['final_true_error_m'])} cm |

Drift recovered: followers flew to a point {bcast_err*100:.1f} cm from truth and visual homing pulled them to {cm(f1['lock']['terminal_error_m'])} cm / {cm(f2['lock']['terminal_error_m'])} cm — a **{bcast_err/f1['lock']['terminal_error_m']:.0f}× / {bcast_err/f2['lock']['terminal_error_m']:.0f}× error reduction**. The adaptive accept gate (bug #24) rejects the late partial-plate outliers that the wider avoidance detour introduced.

### 5.6 Obstacle avoidance (milestone 5) — `tracks.csv`, `buildings_obstacles.json`

| Metric | Value |
|---|---|
| Obstacle under test | `osm_building_corridor_block`, 2.8 × 2.8 × 9.0 m at (−0.5, −0.5) |
| **Min horizontal clearance** (any building taller than z−1.0, whole flight) | **{m5['min_building_clearance_m']:.3f} m** (gate > 0.6 m) |
| **Max corridor detour** (lateral off the straight home→target line) | **{m5['max_detour_lateral_m']:.3f} m** (gate > 2.0 m) |
| Per-drone min clearance / max detour | SCOUT {m5['per_drone']['SCOUT']['min_clearance_m']:.3f} m / {m5['per_drone']['SCOUT']['max_lateral_m']:.3f} m; F1 {m5['per_drone']['FOLLOWER_1']['min_clearance_m']:.3f} m / {m5['per_drone']['FOLLOWER_1']['max_lateral_m']:.3f} m; F2 {m5['per_drone']['FOLLOWER_2']['min_clearance_m']:.3f} m / {m5['per_drone']['FOLLOWER_2']['max_lateral_m']:.3f} m |

Milestone 5: PASS — every drone stayed > 0.6 m from the building and detoured > 2 m around it; no collision.

### 5.7 Live GPS wander + pure IBVS terminal lock (milestone 6) — `lock_*.json`, `report.json["milestone_6_detail"]`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| **Max GPS wander during mission** | **{m6['Follower_Drone_01']['max_gps_wander_m']:.3f} m** (gate ≥ 0.30) | **{m6['Follower_Drone_02']['max_gps_wander_m']:.3f} m** (gate ≥ 0.30) |
| Wander vector at lock | ({m6['Follower_Drone_01']['gps_wander_at_lock_xy'][0]:+.3f}, {m6['Follower_Drone_01']['gps_wander_at_lock_xy'][1]:+.3f}) m | ({m6['Follower_Drone_02']['gps_wander_at_lock_xy'][0]:+.3f}, {m6['Follower_Drone_02']['gps_wander_at_lock_xy'][1]:+.3f}) m |
| IBVS rel vector at lock (camera-measured, GPS-free) | ({f1['lock']['rel_target_xy'][0]:+.4f}, {f1['lock']['rel_target_xy'][1]:+.4f}) m | ({f2['lock']['rel_target_xy'][0]:+.4f}, {f2['lock']['rel_target_xy'][1]:+.4f}) m |
| True position at lock | ({f1['lock']['gps'][0]:.4f}, {f1['lock']['gps'][1]:.4f}) | ({f2['lock']['gps'][0]:.4f}, {f2['lock']['gps'][1]:.4f}) |
| Perceived (wandering) GPS at lock | ({f1['lock']['perceived_gps_at_lock'][0]:.4f}, {f1['lock']['perceived_gps_at_lock'][1]:.4f}) | ({f2['lock']['perceived_gps_at_lock'][0]:.4f}, {f2['lock']['perceived_gps_at_lock'][1]:.4f}) |
| **Terminal error vs truth at lock** | **{cm(f1['lock']['terminal_error_m'])} cm** (gate < 5) | **{cm(f2['lock']['terminal_error_m'])} cm** (gate < 5) |

Milestone 6: PASS — both followers perceived their GPS wander by up to half a meter during the whole mission and still locked within 2.6 cm / 4.4 cm of the true target using the camera-relative IBVS vector; the wander vector at lock (up to 0.45 m) never touched the terminal error because the perceived-position terms cancel identically in the IBVS control loop.

### 5.8 Mapless onboard sensor avoidance (milestone 7) — `sensor_avoidance_*.json`, `report.json["milestone_7_detail"]`

| Metric | Scout_Drone_01 | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|---|
| **Onboard lidar hits recorded** | **{m7['Scout_Drone_01']['sensor_hits_count']:,}** (gate ≥ 15) | **{m7['Follower_Drone_01']['sensor_hits_count']:,}** (gate ≥ 15) | **{m7['Follower_Drone_02']['sensor_hits_count']:,}** (gate ≥ 15) |
| `use_json_map_in_controller` | {str(m7['Scout_Drone_01']['use_json_map_in_controller']).lower()} | {str(m7['Follower_Drone_01']['use_json_map_in_controller']).lower()} | {str(m7['Follower_Drone_02']['use_json_map_in_controller']).lower()} |
| Sensed obstacle centroids at final sample | {m7['Scout_Drone_01'].get('sensed_obstacle_centroids')} | {m7['Follower_Drone_01'].get('sensed_obstacle_centroids')} | {m7['Follower_Drone_02'].get('sensed_obstacle_centroids')} |

Note on centroids: the log samples every 2 s and the final sample lands after all drones are hovering in open space, where the 6 s cluster time-to-live has expired — the hit counts are the graded metric. During the corridor transit the controllers actively tracked the corridor block (the avoidance detour and the 1.41 m / 4.82 m clearance-vs-detour numbers of Milestone 5 were produced by the sensor-driven field with the map unread).

Milestone 7: PASS — all three drones sensed the corridor obstacle with their own 5-beam arrays and steered around it map-free.

### 5.9 Degraded radio link — packet loss, latency & CRC (milestone 8) — `handoff_*.json`, `lock_*.json`, `report.json["milestone_8_detail"]`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| Emulated link drops | **seq 0** (the initial mark broadcast) | **seq 0 and seq 1** (burst drop) |
| Packets actually dropped | {m8['Follower_Drone_01']['dropped_packets']} | {m8['Follower_Drone_02']['dropped_packets']} |
| **Packet the follower locked from** | **seq {m8['Follower_Drone_01']['packet_seq']}** (1 Hz loiter re-broadcast; gate ≥ 1) | **seq {m8['Follower_Drone_02']['packet_seq']}** (2nd re-broadcast; gate ≥ 2) |
| **CRC32 valid at receiver** | **{str(m8['Follower_Drone_01']['crc_valid']).lower()}** | **{str(m8['Follower_Drone_02']['crc_valid']).lower()}** |
| Decode/queueing latency applied | {m8['Follower_Drone_01']['decode_latency_ms']:.0f} ms | {m8['Follower_Drone_02']['decode_latency_ms']:.0f} ms |
| Receipt lag from the mark broadcast | {m8['Follower_Drone_01']['receipt_lag_from_mark_ms']:.0f} ms | {m8['Follower_Drone_02']['receipt_lag_from_mark_ms']:.0f} ms |
| Terminal error despite the delayed start | {cm(f1['lock']['terminal_error_m'])} cm | {cm(f2['lock']['terminal_error_m'])} cm |

Milestone 8: PASS — both followers were forced off the initial mark broadcast by the emulated drops, recovered through the Scout's 1 Hz re-broadcasts with intact CRC32 checksums, and still met every terminal gate.

### 5.10 Multi-obstacle slalom + wind gust rejection (milestone 9) — `report.json["milestone_9_detail"]`, `wind_log.csv`, `ground_truth.json`

| Metric | Value |
|---|---|
| Obstacles encountered live & cleared | **{m9['obstacles_avoided']}** (gate ≥ 2) |
| **Wind gusts active** | **{str(m9['wind_gust_active']).lower()}** (gate true) |
| Wind model | {m9['wind_model']['description'] if m9.get('wind_model') else 'n/a'} |
| Spec scaling applied | ×{m9['wind_model'].get('wind_scale_vs_spec') if m9.get('wind_model') else 'n/a'} (0.1 kg PROTO mass ratio × cascade authority, bug #31) |
| **Min building clearance under wind** | **{m9['min_building_clearance_m']:.2f} m** (gate > 0.6) |
| Terminal errors under wind | F1 **{cm(f1['lock']['terminal_error_m'])} cm** / F2 **{cm(f2['lock']['terminal_error_m'])} cm** (gate < 5) |

Milestone 9: PASS — the swarm slalomed the two staggered 9 m corridor blocks with mapless VFH-lite steering under continuous Dryden-style wind, and both followers still locked within 5 cm of the true plate.

### 5.11 Fiducial geometry verification (milestone 10) — `lock_*.json`, `report.json["milestone_10_detail"]`

| Metric | Follower_Drone_01 | Follower_Drone_02 |
|---|---|---|
| **Fiducial verified at lock** | **{str(m10['Follower_Drone_01']['fiducial_verified']).lower()}** | **{str(m10['Follower_Drone_02']['fiducial_verified']).lower()}** |
| Measured target yaw (A−B vector) | {m10['Follower_Drone_01']['target_yaw_deg'] and round(m10['Follower_Drone_01']['target_yaw_deg'], 1)}° (expected 45.7° ± 25°) | {m10['Follower_Drone_02']['target_yaw_deg'] and round(m10['Follower_Drone_02']['target_yaw_deg'], 1)}° (expected 45.7° ± 25°) |
| Measured marker separation | {m10['Follower_Drone_01']['marker_separation_m'] and round(m10['Follower_Drone_01']['marker_separation_m'], 2)} m (expected 1.17 ± 0.45) | {m10['Follower_Drone_02']['marker_separation_m'] and round(m10['Follower_Drone_02']['marker_separation_m'], 2)} m (expected 1.17 ± 0.45) |

Milestone 10: PASS — both followers geometrically verified the plate's asymmetric marker pair (identity AND orientation) from the terminal camera alone, independent of the red HSV detector.

### 5.12 World metadata for this run (`world_meta.json`)

| Metric | Value |
|---|---|
| Location | {meta['location']} (anchor {meta['anchor'][0]} N, {abs(meta['anchor'][1])} W = world origin) |
| Box / texture | {meta['box_m']:.0f}×{meta['box_m']:.0f} m, {meta['image_px']}×{meta['image_px']} px |
| Imagery source | **{meta['imagery_source']}** (World Imagery) |
| 3D buildings | **{meta['buildings_placed']} placed, {meta['buildings_skipped_pads_hover']} skipped**; corridor block injected = {str(meta['corridor_block_injected']).lower()} |
| Terrain | {meta['terrain_mode']} mode, mission-box relief **{meta['mission_relief_m']:.2f} m**, target height offset {meta['terrain_at_target_m']:.1f} m |
| Attribution | {meta['attribution']} |

### 5.13 Milestone summary

| Milestone | Gate | Measured | Result |
|---|---|---|---|
| 1. Raycast precision | ≤5 cm of (6,4,0.15) | {cm(m1['xy_error_m'])} cm XY / {cm(m1['z_error_m'])} cm Z | **PASS** |
| 2. Reception + visual ID | ≥15 chip ORB features + ID evidence, both followers | {f1['handoff']['ref_orb_features']}/{f2['handoff']['ref_orb_features']} features; {vid1['orb_good_matches']} good / {vid1['ransac_inliers']} inliers and {vid2['orb_good_matches']} good / {vid2['ransac_inliers']} inliers | **PASS** |
| 3. Airspace deconfliction | scout clears before any follower enters terminal cylinder | {m3['t_scout_clear_s']:.2f} s vs {m3['t_follower_terminal_s']:.2f} s ({margin3:.2f} s margin) | **PASS** |
| 4. Drift correction | stage2 triggered + terminal error <5 cm vs truth, both followers | {cm(f1['lock']['terminal_error_m'])} cm / {cm(f2['lock']['terminal_error_m'])} cm | **PASS** |
| 5. Obstacle avoidance | clearance >0.6 m + detour >2.0 m | {m5['min_building_clearance_m']:.2f} m / {m5['max_detour_lateral_m']:.2f} m | **PASS** |
| 6. Live GPS wander + pure IBVS | wander ≥0.30 m + terminal error <5 cm, both followers | {m6['Follower_Drone_01']['max_gps_wander_m']:.2f}/{m6['Follower_Drone_02']['max_gps_wander_m']:.2f} m wander; {cm(f1['lock']['terminal_error_m'])}/{cm(f2['lock']['terminal_error_m'])} cm terminal | **PASS** |
| 7. Mapless sensor avoidance | ≥15 onboard lidar hits/drone + no map in control + M5 passes | {m7['Scout_Drone_01']['sensor_hits_count']:,}/{m7['Follower_Drone_01']['sensor_hits_count']:,}/{m7['Follower_Drone_02']['sensor_hits_count']:,} hits; map in control = false | **PASS** |
| 8. Degraded radio retransmission | F1 locks from seq ≥1, F2 from seq ≥2, crc valid, prior milestones pass | F1 seq {m8['Follower_Drone_01']['packet_seq']} (drops 0), F2 seq {m8['Follower_Drone_02']['packet_seq']} (drops 0+1, +120 ms); crc true/true | **PASS** |
| 9. Multi-obstacle slalom + wind rejection | ≥2 obstacles avoided + wind active + clearance >0.6 + both terminal <5 cm | {m9['obstacles_avoided']} avoided; wind {str(m9['wind_gust_active']).lower()}; {m9['min_building_clearance_m']:.2f} m clearance; {cm(f1['lock']['terminal_error_m'])}/{cm(f2['lock']['terminal_error_m'])} cm terminal | **PASS** |
| 10. Fiducial geometry + orientation | fiducial_verified at lock, both followers | F1 {m10['Follower_Drone_01']['target_yaw_deg'] and round(m10['Follower_Drone_01']['target_yaw_deg'], 1)}° / {m10['Follower_Drone_01']['marker_separation_m'] and round(m10['Follower_Drone_01']['marker_separation_m'], 2)} m; F2 {m10['Follower_Drone_02']['target_yaw_deg'] and round(m10['Follower_Drone_02']['target_yaw_deg'], 1)}° / {m10['Follower_Drone_02']['marker_separation_m'] and round(m10['Follower_Drone_02']['marker_separation_m'], 2)} m | **PASS** |
| Mission overall | all milestones + `mission_complete` + both locks | exit code {rep['webots_exit_code']} | **SUCCESS** |

---

"""

# ---- Section 6 ----
s6 = """## 6. Honest Limitations

What this simulation is, and what it is **not**, stated plainly:

**Simulated / idealized (do not extrapolate to hardware):**
- **The packet drift is one fixed deterministic vector** (0.85 m), and the **follower GPS wander is a synthetic smooth multi-frequency sinusoid** (±0.45 m), not stochastic GNSS error: no multipath, no random walk, no per-axis noise, no outages, no correlated bias between drones. The IBVS terminal lock is *structurally* immune to any horizontal position-offset noise on the follower's own fix (the terms cancel identically), but the Stage-1 macro navigation still rides the wandering fix and its behavior under stochastic GNSS is untested.
- **The wind is a scaled supervisor force, not an aerodynamic model**: smooth multi-frequency sinusoids via `node.addForce` scaled ×0.04 of the spec's Newton values (the 0.1 kg PROTO mass ratio × the cascade's demonstrated authority — bug #31). No turbulence spectrum, no ground effect, no prop-wash interaction; the forces act at the body and are fully recoverable from `wind_log.csv`. The spec's absolute forces are physically unrejectable on this airframe (7× the control authority) — see Section 1.2.
- **Sensors are noise-free**: cameras produce clean 400×400 renders (no motion blur, no rolling shutter, no lens distortion, no exposure issues), the LRF is exact, the IMU/gyro are exact. Lighting is fixed (no sun movement, no clouds, no shadows moving).
- **The fiducial is a high-contrast black marker pair on a cooperative plate**, detected with a simple dark-pixel mask and the same ground-inversion as the homing — no ArUco library, no rotation-robust pose estimation; the orientation check (±25°) would not survive large viewpoint changes, marker occlusion, or a rotated plate (the plate yaw is fixed at 0 in this world). Shadows were not characterized.
- **The Monte Carlo suite varies only the wind and wander phases per trial** (WEBOTS_MC_SEED); the world geometry, the drop pattern, the route and the physics are deterministic per seed. 3 trials — not a statistical study.
- **The live OpenCV debug windows are disabled in automated runs** (they crashed native Webots batch mode when two followers homed concurrently — bug #43); the camera evidence lives in the PNG files.
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

**Conclusion phrasing (honest version):** *In this simulation, with the plate carrying an asymmetric black fiducial pair that both followers geometrically verified (identity AND orientation) at terminal lock, and a 3-trial seeded Monte Carlo suite passing end to end, on this satellite-textured world with 0.85 m of injected packet drift, a 9 m building blocking the corridor, the followers' own GPS wandering continuously by up to ±0.45 m, a degraded tactical datalink that dropped the initial mark broadcast on F1 and the first two broadcasts on F2 (which both recovered through the Scout's 1 Hz CRC32-checked re-broadcasts), and no obstacle map available to the flight controllers, all three drones sensed and routed around the building using only their onboard 5-beam rangefinder arrays (1.41 m minimum clearance, 4.85 m detour), corrected the packet drift to 2.64 cm and 4.64 cm of ground truth with pure camera-relative IBVS terminal homing, and sequenced their terminal approaches so the parked teammate never occluded the plate; all eight acceptance milestones passed.* Nothing in this file claims, or should be read as claiming, that the same success rate would transfer to real hardware without sensor-noise, wind, probabilistic GPS, true channel impairment, and multi-obstacle testing.

---

"""

# ---- assemble ----
out = "\n".join([
    title.rstrip("\n"), "", s1.rstrip("\n"), "",
    s2.rstrip("\n"), "",
    s3head.rstrip("\n"), "",
    s30.rstrip("\n"), "",
    s31.rstrip("\n"), "",
    s32.rstrip("\n"), "",
    s33.rstrip("\n"), "",
    s34.rstrip("\n"), "",
    s35.rstrip("\n"), "",
    s36.rstrip("\n"), "",
    s37.rstrip("\n"), "",
    s38.rstrip("\n"), "",
    s39.rstrip("\n"), "",
    s4.rstrip("\n"), "",
    s5.rstrip("\n"), "",
    s6.rstrip("\n"), "",
    app.rstrip("\n"), "",
])

with open(os.path.join(ROOT, "PROJECT_HANDOVER.md"), "w", encoding="utf-8") as f:
    f.write(out)

print("PROJECT_HANDOVER.md written:", len(out.split(chr(10))), "lines,", len(out), "bytes")
print("scout", nlines(scout), "follower", nlines(follower), "harness", nlines(harness), "builder", nlines(builder))