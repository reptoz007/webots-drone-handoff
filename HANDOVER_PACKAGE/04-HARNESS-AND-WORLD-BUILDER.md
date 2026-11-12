# Verification Harness + World Builder

## run_verification.py
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
def _parse_args(argv):
    """[world.wbt] [--mc N] — N > 0 runs N seeded Monte Carlo trials."""
    world = os.path.join(ROOT, "worlds", "drone_handoff.wbt")
    mc = 0
    i = 0
    while i < len(argv):
        if argv[i] == "--mc":
            mc = int(argv[i + 1])
            i += 2
        else:
            world = argv[i]
            i += 1
    return world, mc


RESULTS = os.path.join(ROOT, "results")
WORLD = None   # set by main() from the parsed arguments


def read_json(name):
    p = os.path.join(RESULTS, name)
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return None


def run_trial(trial_seed=0):
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
    env["WEBOTS_MC_SEED"] = str(trial_seed)
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

    # Milestone 10: geometric fiducial verification — at terminal lock each
    # follower must have confirmed the plate's asymmetric black marker pair
    # (identity AND orientation) from the terminal camera alone.
    fid = [(followers[f].get("lock") or {}).get("fiducial_verified", False)
           for f in followers]
    milestones["10_fiducial_geometry_verification"] = bool(all(fid) and len(fid) == 2)

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
    report["milestone_10_detail"] = {
        fid: {"fiducial_verified": (followers[fid].get("lock") or {}).get("fiducial_verified"),
              "target_yaw_deg": (followers[fid].get("lock") or {}).get("fiducial_target_yaw_deg"),
              "marker_separation_m": (followers[fid].get("lock") or {}).get("fiducial_marker_separation_m")}
        for fid in followers}
    report["mc_seed"] = trial_seed
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
             "9_multi_obstacle_and_wind_rejection": "9. Multi-obstacle slalom (>=2 avoided) + wind gust rejection (terminal <5 cm)",
             "10_fiducial_geometry_verification": "10. Fiducial geometry + orientation verified at lock (both followers)"}
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


def run_monte_carlo(n_trials):
    """Phase 10 Monte Carlo suite: n_trials full Webots runs, each with its own
    WEBOTS_MC_SEED (per-trial wind phase offsets + follower wander phases).
    Every trial's results are archived under results_mc/trial_<i>/ and a
    summary (per-milestone pass rates, terminal-error statistics) is written to
    results_mc/mc_summary.json. Exit code 0 requires EVERY trial to pass."""
    mc_dir = os.path.join(ROOT, "results_mc")
    os.makedirs(mc_dir, exist_ok=True)
    trials = []
    for i in range(n_trials):
        print(f"\n===== MONTE CARLO TRIAL {i + 1}/{n_trials} (seed {i}) =====", flush=True)
        code = run_trial(i)
        with open(os.path.join(RESULTS, "report.json")) as f:
            rep = json.load(f)
        tdir = os.path.join(mc_dir, f"trial_{i}")
        if os.path.isdir(tdir):
            shutil.rmtree(tdir)
        shutil.copytree(RESULTS, tdir)
        trials.append({"seed": i, "mission_success": rep["mission_success"],
                       "webots_exit_code": rep["webots_exit_code"],
                       "milestones": rep["milestones"],
                       "terminal_errors_m": {fid: (rep["followers"][fid].get("lock") or {}).get("terminal_error_m")
                                             for fid in rep["followers"]},
                       "sim_time_s": (rep.get("ground_truth") or {}).get("sim_time_s")})
    term = [t["terminal_errors_m"][f] for t in trials for f in t["terminal_errors_m"]
            if t["terminal_errors_m"][f] is not None]
    summary = {
        "n_trials": n_trials,
        "all_passed": all(t["mission_success"] for t in trials),
        "pass_count": sum(1 for t in trials if t["mission_success"]),
        "milestone_pass_rates": {k: sum(1 for t in trials if t["milestones"].get(k))
                                 for k in trials[0]["milestones"]},
        "terminal_error_m": {"min": min(term), "max": max(term),
                             "mean": sum(term) / len(term)} if term else None,
        "trials": trials,
    }
    with open(os.path.join(mc_dir, "mc_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print("\n===== MONTE CARLO SUMMARY =====")
    print(f"  trials passed: {summary['pass_count']}/{n_trials}")
    for k, v in summary["milestone_pass_rates"].items():
        print(f"  {k}: {v}/{n_trials}")
    if summary["terminal_error_m"]:
        te = summary["terminal_error_m"]
        print(f"  terminal error: min {te['min']*100:.2f} cm / mean {te['mean']*100:.2f} cm / max {te['max']*100:.2f} cm")
    print(f"  summary: {os.path.join(mc_dir, 'mc_summary.json')}")
    return 0 if summary["all_passed"] else 1


def main():
    global WORLD
    WORLD, mc = _parse_args(sys.argv[1:])
    if mc > 0:
        return run_monte_carlo(mc)
    return run_trial(0)


if __name__ == "__main__":
    sys.exit(main())
```

## build_realmap_world.py
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
