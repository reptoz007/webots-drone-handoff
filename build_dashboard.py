"""Builds self-contained HTML dashboard displaying all 13 Milestones, Rust latency stats,
moving target trajectories, and Monte Carlo results from results/ and results_mc/."""
import base64
import csv
import io
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Rectangle
import cv2

ROOT = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(ROOT, "results")
DEFAULT_OUT = os.path.join(ROOT, "results")


def read_json(name):
    p = os.path.join(RESULTS, name)
    if os.path.exists(p):
        with open(p) as f:
            return json.load(f)
    return None


def img_b64_jpeg(path, width=320, quality=80):
    p = os.path.join(RESULTS, path)
    if not os.path.exists(p):
        return None
    img = cv2.imread(p)
    if img is None:
        return None
    h, w = img.shape[:2]
    if w > width:
        img = cv2.resize(img, (width, int(h * width / w)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return base64.b64encode(buf.tobytes()).decode("ascii") if ok else None


def build_map_png(mark, truth, tracks, meta, map_path, obstacles):
    fig, ax = plt.subplots(figsize=(9.0, 6.8), dpi=110)
    if meta and map_path and os.path.exists(map_path):
        img = mpimg.imread(map_path)
        half = meta["box_m"] / 2.0
        ax.imshow(img, extent=[-half, half, -half, half], origin="upper", zorder=0)
        ax.set_xlim(-60, 60)
        ax.set_ylim(-60, 60)
        ax.set_title(f"{meta['location']} — Rust 13-Milestone Swarm Trajectory", fontsize=11)

    for b in obstacles:
        is_inj = str(b.get("id", "")).startswith("corridor_block")
        ax.add_patch(Rectangle((b["minx"], b["miny"]), b["maxx"] - b["minx"], b["maxy"] - b["miny"],
                               facecolor="#e11d48" if is_inj else "#475569", edgecolor="white", lw=0.8, alpha=0.55, zorder=1))

    colors = {"SCOUT": ("#94a3b8", "--"), "FOLLOWER_1": ("#2563eb", "-"), "FOLLOWER_2": ("#d97706", "-")}
    for name, (col, sty) in colors.items():
        rows = tracks.get(name, [])
        if rows:
            xs = [r[1] for r in rows]
            ys = [r[2] for r in rows]
            ax.plot(xs, ys, color="white", lw=3.8, zorder=2, alpha=0.85)
            ax.plot(xs, ys, sty, color=col, lw=2.0, label=name, zorder=3)

    if truth:
        it = truth.get("initial_target_xyz", [6.0, 4.0, 0.05])
        ft = truth.get("true_target_xyz", [6.0, 4.0, 0.05])
        ax.plot([it[0], ft[0]], [it[1], ft[1]], "r-.", lw=2.2, label="Moving Target Path", zorder=4)
        ax.plot(ft[0], ft[1], "*", ms=18, mfc="#dc2626", mec="white", mew=1.2, label="Final Target Pos", zorder=5)

    ax.legend(loc="upper left", fontsize=8, framealpha=0.92)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    fig.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format="png", facecolor="white")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def main():
    out_dir = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else DEFAULT_OUT
    os.makedirs(out_dir, exist_ok=True)
    rep = read_json("report.json") or {}
    mark = rep.get("mark") or {}
    truth = rep.get("ground_truth") or {}
    meta = read_json("world_meta.json")
    obstacles = read_json("buildings_obstacles.json") or []

    tracks = {}
    tpath = os.path.join(RESULTS, "tracks.csv")
    if os.path.exists(tpath):
        with open(tpath) as f:
            for row in csv.DictReader(f):
                tracks.setdefault(row["drone"], []).append((float(row["t"]), float(row["x"]), float(row["y"])))

    map_path = os.path.join(ROOT, "worlds", "textures", "realmap_esri.png")
    map_b64 = build_map_png(mark, truth, tracks, meta, map_path, obstacles)

    m_rows = "".join(
        f"<tr><td>{k}</td><td><b>{'PASS' if v else 'FAIL'}</b></td></tr>"
        for k, v in (rep.get("milestones") or {}).items()
    )

    perf = rep.get("rust_performance") or {}
    p_rows = "".join(
        f"<tr><td>{k}</td><td>{v.get('mean_step_us', 0):.1f} us</td><td>{v.get('p99_step_us', 0):.1f} us</td></tr>"
        for k, v in perf.items() if isinstance(v, dict)
    )

    mc_html = ""
    mc_path = os.path.join(ROOT, "results_mc", "mc_summary.json")
    if os.path.exists(mc_path):
        with open(mc_path) as f:
            mc = json.load(f)
        te = mc.get("terminal_error_m") or {}
        mc_html = (f"<h3>Monte Carlo Verification ({mc.get('pass_count')}/{mc.get('n_trials')} Passed)</h3>"
                   f"<p>Terminal Error: min {te.get('min', 0)*100:.2f} cm | mean {te.get('mean', 0)*100:.2f} cm | max {te.get('max', 0)*100:.2f} cm</p>")

    chip_b64 = img_b64_jpeg("scout_chip.png") or ""
    l1_b64 = img_b64_jpeg("lock_frame_Follower_Drone_01.png") or ""
    l2_b64 = img_b64_jpeg("lock_frame_Follower_Drone_02.png") or ""

    html = f"""<!doctype html><html><head><meta charset="utf-8"><title>Rust Swarm Handoff Dashboard</title>
<style>body{{font-family:system-ui,sans-serif;max-width:1050px;margin:24px auto;padding:0 16px;color:#1e293b}}
table{{border-collapse:collapse;width:100%;margin-bottom:20px}}th,td{{border:1px solid #cbd5e1;padding:6px 10px;text-align:left}}
.imgs{{display:flex;gap:12px;flex-wrap:wrap}}.imgs img{{width:300px;border:1px solid #94a3b8}}</style></head><body>
<h1>Rust Cooperative Drone Handoff — Verification Report</h1>
<p><b>Mission Success:</b> {rep.get('mission_success')} | <b>Sim Time:</b> {truth.get('sim_time_s', 0):.1f} s | <b>Target Displacement:</b> {truth.get('target_displacement_m', 0):.2f} m</p>
{mc_html}
<h3>13 Acceptance Milestones</h3><table><tr><th>Milestone</th><th>Status</th></tr>{m_rows}</table>
<h3>Native Rust Controller Execution Latency (8 ms control loop)</h3>
<table><tr><th>Controller</th><th>Mean Step</th><th>P99 Step</th></tr>{p_rows}</table>
<h3>Urban Trajectory & Moving Target Map</h3>
<img src="data:image/png;base64,{map_b64}" style="width:100%;max-width:850px">
<h3>Onboard Visual Evidence</h3>
<div class="imgs">
  <img src="data:image/jpeg;base64,{chip_b64}" alt="Scout Chip">
  <img src="data:image/jpeg;base64,{l1_b64}" alt="F1 Lock">
  <img src="data:image/jpeg;base64,{l2_b64}" alt="F2 Lock">
</div></body></html>"""

    out_file = os.path.join(out_dir, "drone-mission-dashboard.html")
    with open(out_file, "w", encoding="utf-8") as f:
        f.write(html)
    print(f"Dashboard written to {out_file}")


if __name__ == "__main__":
    main()
