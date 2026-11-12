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