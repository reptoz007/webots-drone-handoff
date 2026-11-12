"""Automated verification harness:
1. Compiles native Rust controllers (`cargo build --release`) and deploys binaries + DLLs.
2. Runs Webots in batch realtime mode (single run or `--mc N` seeded Monte Carlo trials).
3. Judges all 13 acceptance milestones and writes `results/report.json` / `results_mc/mc_summary.json`.
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
RESULTS = os.path.join(ROOT, "results")
WORLD = None


def _parse_args(argv):
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


def build_rust_controllers():
    """Compiles native Rust controllers in release mode and copies binaries into controllers/<name>/."""
    env = dict(os.environ)
    env["WEBOTS_HOME"] = WEBOTS_HOME
    lib_ctrl = os.path.join(WEBOTS_HOME, "lib", "controller")
    mingw_bin = os.path.join(WEBOTS_HOME, "msys64", "mingw64", "bin")
    cargo_bin = os.path.join(os.path.expanduser("~"), ".cargo", "bin")
    env["PATH"] = f"{cargo_bin};{lib_ctrl};{mingw_bin};" + env.get("PATH", "")

    print("Building native Rust controllers (`cargo build --release`)...", flush=True)
    res = subprocess.run(["cargo", "build", "--release"], cwd=ROOT, env=env)
    if res.returncode != 0:
        raise RuntimeError("Cargo release build failed!")

    release_dir = os.path.join(ROOT, "target", "release")
    dll_src = os.path.join(lib_ctrl, "Controller.dll")
    for cname in ("scout_controller", "follower_controller", "metrics_supervisor"):
        cdir = os.path.join(ROOT, "controllers", cname)
        exe_src = os.path.join(release_dir, f"{cname}.exe")
        if os.path.exists(exe_src):
            shutil.copy2(exe_src, os.path.join(cdir, f"{cname}.exe"))
        if os.path.exists(dll_src):
            shutil.copy2(dll_src, os.path.join(cdir, "Controller.dll"))
    print("Deployed native Rust controller executables.", flush=True)


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
    lib_ctrl = os.path.join(WEBOTS_HOME, "lib", "controller")
    mingw_bin = os.path.join(WEBOTS_HOME, "msys64", "mingw64", "bin")
    cargo_bin = os.path.join(os.path.expanduser("~"), ".cargo", "bin")
    env["PATH"] = f"{cargo_bin};{lib_ctrl};{mingw_bin};" + env.get("PATH", "")

    log = open(os.path.join(RESULTS, "webots_console.log"), "w")
    t0 = time.time()
    proc = subprocess.Popen(
        [WEBOTS, "--batch", "--mode=realtime", "--stdout", "--stderr", WORLD],
        stdout=log, stderr=subprocess.STDOUT, env=env)
    print(f"Webots running (pid {proc.pid}, seed={trial_seed}).", flush=True)
    try:
        code = proc.wait(timeout=600)
    except subprocess.TimeoutExpired:
        proc.kill()
        code = "killed-timeout"
    finally:
        log.close()
    elapsed = time.time() - t0

    mark = read_json("mark.json")
    truth = read_json("ground_truth.json")
    wmeta = read_json("world_meta.json") or {}
    followers = {}
    for fid in ("Follower_Drone_01", "Follower_Drone_02"):
        h = read_json(f"handoff_{fid}.json")
        lk = read_json(f"lock_{fid}.json")
        entry = {"handoff": h, "lock": lk}
        if lk and truth:
            ltgt = (truth.get("lock_true_targets") or {}).get(fid, truth.get("true_target_xyz", [6.0, 4.0, 0.05]))
            entry["true_target_at_lock"] = ltgt
        followers[fid] = entry

    def deconfliction_pass():
        t_scout = t_follow = None
        try:
            with open(os.path.join(RESULTS, "tracks.csv")) as f:
                for row in csv.DictReader(f):
                    t = float(row["t"]); x = float(row["x"]); y = float(row["y"]); z = float(row["z"])
                    name = row["drone"]
                    if name == "SCOUT" and t_scout is None and math.hypot(x - 15.0, y - 10.0) < 1.0 and z > 5.2:
                        t_scout = t
                    if name in ("FOLLOWER_1", "FOLLOWER_2") and t_follow is None and t > (mark or {}).get("sim_time_s", 0):
                        # First follower entering terminal hover (< 0.4 m of lock spot)
                        lk = followers["Follower_Drone_01" if name == "FOLLOWER_1" else "Follower_Drone_02"].get("lock")
                        if lk and math.hypot(x - lk["gps"][0], y - lk["gps"][1]) < 0.4:
                            t_follow = t
        except FileNotFoundError:
            return False, None, None
        return (t_scout is not None and t_follow is not None and t_scout < t_follow), t_scout, t_follow

    def obstacle_metrics():
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
                            continue
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
                        if 0.0 <= t <= 1.0:
                            lat = abs(vx * (y - hy) - vy * (x - hx)) / seg_len
                            max_lateral = max(max_lateral, lat)
                            d["max_lateral_m"] = max(d["max_lateral_m"], lat)
        except FileNotFoundError:
            return None, 0.0, {}
        return min_clear, max_lateral, per_drone

    def obstacles_avoided_count(min_encounter_m=6.0):
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
        m1_z = abs(c[2] - 0.15)
        milestones["1_raycast_precision"] = bool(m1_err < 0.05 and m1_z < 0.05)
    else:
        milestones["1_raycast_precision"] = False

    feats = []
    for fid in followers:
        h = followers[fid].get("handoff") or {}
        lk = followers[fid].get("lock") or {}
        vid = bool(lk.get("visual_id_confirmed", False)) or read_json(f"visual_id_{fid}.json") is not None
        feats.append((h.get("ref_orb_features", 0), vid))
    milestones["2_reception_visual_id"] = bool(mark and all(n >= 15 and vid for n, vid in feats))

    m3, t_scout_clear, t_follow_terminal = deconfliction_pass()
    milestones["3_airspace_deconfliction"] = bool(m3)

    term = [(followers[f].get("lock") or {}).get("stage2_triggered", False) and
            (followers[f].get("lock") or {}).get("terminal_error_m", 9e9) < 0.05
            for f in followers]
    milestones["4_drift_correction_visual_homing"] = bool(all(term))

    m5_clear, m5_lateral, m5_per_drone = obstacle_metrics()
    milestones["5_obstacle_avoidance"] = bool(m5_clear is not None and m5_clear > 0.6 and m5_lateral > 2.0)

    m6_wander, m6_err = [], []
    for fid in followers:
        lk = followers[fid].get("lock") or {}
        m6_wander.append(lk.get("max_gps_wander_m"))
        m6_err.append(lk.get("terminal_error_m"))
    milestones["6_live_gps_wander_ibvs"] = bool(
        len(m6_wander) == 2 and all(w is not None and w >= 0.30 for w in m6_wander)
        and all(e is not None and e < 0.05 for e in m6_err))

    sensor_logs = {did: read_json(f"sensor_avoidance_{did}.json")
                   for did in ("Scout_Drone_01", "Follower_Drone_01", "Follower_Drone_02")}
    milestones["7_mapless_sensor_avoidance"] = bool(
        all(s and s.get("sensor_hits_count", 0) >= 15 and s.get("use_json_map_in_controller") is False
            for s in sensor_logs.values()) and milestones["5_obstacle_avoidance"])

    m8_ok = []
    for fid, min_seq in (("Follower_Drone_01", 1), ("Follower_Drone_02", 2)):
        lk = followers[fid].get("lock") or {}
        m8_ok.append(lk.get("handoff_packet_seq", -1) >= min_seq and lk.get("handoff_crc_valid", False) is True)
    milestones["8_degraded_radio_retransmission"] = bool(all(m8_ok) and len(m8_ok) == 2)

    obstacles_avoided = obstacles_avoided_count()
    wind_active = bool(truth and truth.get("wind_gust_active") is True)
    milestones["9_multi_obstacle_and_wind_rejection"] = bool(
        obstacles_avoided >= 2 and wind_active and m5_clear is not None and m5_clear > 0.6 and all(term))

    fid_ok = [(followers[f].get("lock") or {}).get("fiducial_verified", False) for f in followers]
    milestones["10_fiducial_geometry_verification"] = bool(all(fid_ok) and len(fid_ok) == 2)

    # Milestone 11: Real urban OSM buildings present & cleared
    real_osm_count = wmeta.get("real_osm_buildings_placed", 0)
    milestones["11_real_osm_urban_buildings"] = bool(
        real_osm_count >= 5 and m5_clear is not None and m5_clear > 0.6)

    # Milestone 12: Stochastic Ornstein-Uhlenbeck GNSS random walk + sensor noise
    ou_ok = [(followers[f].get("lock") or {}).get("gnss_model") == "ornstein_uhlenbeck"
             and (followers[f].get("lock") or {}).get("terminal_error_m", 9e9) < 0.05
             for f in followers]
    milestones["12_stochastic_gnss_and_sensor_noise"] = bool(all(ou_ok) and len(ou_ok) == 2)

    # Milestone 13: Moving ground target tracking (>= 6.0 m displacement, < 5 cm lock)
    tgt_disp = (truth or {}).get("target_displacement_m", 0.0)
    milestones["13_moving_target_tracking"] = bool(tgt_disp >= 6.0 and all(term))

    perf_logs = {did: read_json(f"rust_perf_{did}.json")
                 for did in ("Scout_Drone_01", "Follower_Drone_01", "Follower_Drone_02")}

    mission_ok = bool(mark and truth and truth.get("outcome") == "mission_complete"
                      and all(f["lock"] for f in followers.values()))
    all_milestones = all(milestones.values())
    success = mission_ok and all_milestones

    report = {
        "mission_success": success,
        "webots_exit_code": code,
        "real_elapsed_s": round(elapsed, 1),
        "mc_seed": trial_seed,
        "milestones": milestones,
        "mark": mark,
        "followers": followers,
        "ground_truth": truth,
        "rust_performance": perf_logs,
        "milestone_5_detail": {"min_building_clearance_m": m5_clear, "max_detour_lateral_m": m5_lateral, "per_drone": m5_per_drone},
        "milestone_6_detail": {fid: {"max_gps_wander_m": (followers[fid].get("lock") or {}).get("max_gps_wander_m")} for fid in followers},
        "milestone_7_detail": sensor_logs,
        "milestone_11_detail": {"real_osm_buildings_placed": real_osm_count, "min_building_clearance_m": m5_clear},
        "milestone_13_detail": {"target_displacement_m": tgt_disp, "lock_true_targets": (truth or {}).get("lock_true_targets")},
    }
    with open(os.path.join(RESULTS, "report.json"), "w") as f:
        json.dump(report, f, indent=2)

    print(f"\n===== TRIAL {trial_seed} RESULT: SUCCESS={success} ({sum(milestones.values())}/{len(milestones)} milestones) =====", flush=True)
    for k, v in milestones.items():
        print(f"  [{'PASS' if v else 'FAIL'}] {k}")
    return 0 if success else 1


def run_monte_carlo(n_trials):
    mc_dir = os.path.join(ROOT, "results_mc")
    os.makedirs(mc_dir, exist_ok=True)
    trials = []
    for i in range(n_trials):
        print(f"\n===== MONTE CARLO TRIAL {i + 1}/{n_trials} (seed {i}) =====", flush=True)
        run_trial(i)
        with open(os.path.join(RESULTS, "report.json")) as f:
            rep = json.load(f)
        tdir = os.path.join(mc_dir, f"trial_{i}")
        if os.path.isdir(tdir):
            shutil.rmtree(tdir)
        shutil.copytree(RESULTS, tdir)
        trials.append({
            "seed": i,
            "mission_success": rep["mission_success"],
            "webots_exit_code": rep["webots_exit_code"],
            "milestones": rep["milestones"],
            "terminal_errors_m": {fid: (rep["followers"][fid].get("lock") or {}).get("terminal_error_m")
                                  for fid in rep["followers"]},
            "sim_time_s": (rep.get("ground_truth") or {}).get("sim_time_s"),
        })
    term = [t["terminal_errors_m"][f] for t in trials for f in t["terminal_errors_m"]
            if t["terminal_errors_m"][f] is not None]
    summary = {
        "n_trials": n_trials,
        "all_passed": all(t["mission_success"] for t in trials),
        "pass_count": sum(1 for t in trials if t["mission_success"]),
        "milestone_pass_rates": {k: sum(1 for t in trials if t["milestones"].get(k))
                                 for k in trials[0]["milestones"]},
        "terminal_error_m": {"min": min(term), "max": max(term), "mean": sum(term) / len(term)} if term else None,
        "trials": trials,
    }
    with open(os.path.join(mc_dir, "mc_summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\n===== MONTE CARLO SUMMARY: {summary['pass_count']}/{n_trials} PASSED =====")
    return 0 if summary["all_passed"] else 1


def main():
    global WORLD
    WORLD, mc = _parse_args(sys.argv[1:])
    build_rust_controllers()
    if mc > 0:
        return run_monte_carlo(mc)
    return run_trial(0)


if __name__ == "__main__":
    sys.exit(main())
