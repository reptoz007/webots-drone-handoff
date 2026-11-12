# 6-8 HOUR AUTONOMOUS EXECUTION PROTOCOL (PHASES 11-15) — RUST BASELINE

The system is now fully Rust (native `.exe` Webots controllers built from this Cargo
workspace). Work sequentially through the phases below. **Do not skip any verification gate.**

1. **PHASE 11 — Rust Workspace Compilation & Clipping Fix (`--mc 5`):**
   * Verify all files compile: `cargo build --release` (run_verification.py does this automatically).
   * Run `python build_realmap_world.py`.
   * Run `python run_verification.py worlds/drone_handoff_realmap.wbt --mc 5`.
   * Verify `5/5` trials pass (`mission_success: true`, exit code `0`). If any trial fails,
     inspect `results_mc/trial_<i>/`, tune parameters, document the fix in
     `PROJECT_HANDOVER.md` Section 4 (starting at Bug #45), and re-run until `5/5` passes.
   * Checkpoint `PROJECT_HANDOVER.md` immediately upon `5/5` pass.

2. **PHASE 12 — Dense Urban OSM Geometry Verification (Milestone 11):**
   * Verify `world_meta.json` confirms `real_osm_buildings_placed >= 5` around Farragut
     Square, Downtown DC, and that all 3 drones route around the real multi-story
     buildings with `min_building_clearance_m > 0.6` across `--mc 5`.
   * Checkpoint `PROJECT_HANDOVER.md`.

3. **PHASE 13 — Stochastic Ornstein-Uhlenbeck GNSS & Sensor Noise Stress Test (Milestone 12):**
   * Verify that across `--mc 5`, the Ornstein-Uhlenbeck random walk (`sigma = 0.35 m`,
     `tau = 12.0 s`) and Scout LRF noise (`sigma = 0.012 m`) are active
     (`gnss_model == "ornstein_uhlenbeck"`) and every seed still achieves `< 5.0 cm`
     terminal error.
   * Checkpoint `PROJECT_HANDOVER.md`.

4. **PHASE 14 — Moving Ground Target & Velocity Handoff (Milestone 13):**
   * Verify `Ground_Target` moves `>= 6.0 m` after mark broadcast and both followers
     track and lock onto the moving plate within `< 5.0 cm` across `--mc 5`.
   * Checkpoint `PROJECT_HANDOVER.md`.

5. **PHASE 15 — `--mc 10` Statistical Benchmark & Final Handover:**
   * Run `python run_verification.py worlds/drone_handoff_realmap.wbt --mc 10` and
     confirm `10/10` trials pass across all **13 Milestones**.
   * Run `python run_verification.py worlds/drone_handoff_realmap.wbt` (canonical run)
     and `python build_dashboard.py`.
   * Update `PROJECT_HANDOVER.md` with the complete final Rust source code, all bug log
     entries, the 13-milestone summary table, the `--mc 10` statistical distribution, and
     the Rust microsecond latency benchmark (`mean_step_us` / `p99_step_us`).

## Key differences vs the retired Python controllers (Phases 6-10)
* `TERMINAL_CAM_PITCH = 1.35 rad` + border-clip guard + 3-frame fiducial consensus
  (`fiducial_streak >= 3`, blend `a = 0.15`) — fixes the MC seed-1/seed-2 failures of the
  Python era (bottom-edge plate clipping, un-gated fiducial step jumps).
* Terminal gain boost capped: `1.0 + 6.5 * ...` for followers (scout 11.0).
* Phase 13: GNSS wander is now an Ornstein-Uhlenbeck random walk (`GnssRandomWalk`,
  sigma 0.35 m, tau 12 s); Scout LRF has Gaussian noise (sigma 0.012 m).
* Phase 14: ground target MOVES at [0.09, 0.06, 0] m/s after the mark; handoff packets
  carry `target_vel`; followers predict target position and feed forward relative velocity.
* Pure-Rust CV: HSV, BFS blobs, convex-hull rotating-calipers `minAreaRect`,
  FAST-9 + rotated BRIEF-256 (ORB-equivalent), Lowe ratio 0.8, RANSAC homography,
  JPEG codec — zero OpenCV dependency.
* The old Python controllers are retired as `*_python_fallback.py` (kept for reference;
  Webots runs the native `.exe` when present).
* Legacy Python-era harness kept as `run_verification_python10.py` /
  `build_dashboard_python10.py` (10-milestone gates; results in git history).
