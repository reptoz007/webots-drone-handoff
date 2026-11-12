# Rust Conversion Bring-Up Log (Phase 11 baseline, pre-loop)

Date: 2026-11-13. Records the Python -> Rust conversion, the bugs fixed during
bring-up, and the first two integration runs. The Phase 11 tuning loop
(`--mc 5` iterate-until-pass, bug entries from #45) starts FROM this state.

## What was done

1. **Rust toolchain installed**: rustup stable, `x86_64-pc-windows-msvc`
   (MSVC Build Tools 2022 already present on the machine).
2. **Workspace materialized**: `Cargo.toml` (workspace, release LTO),
   `crates/webots_ffi` (safe FFI over the Webots R2025a C API, links
   `lib/controller/Controller.lib`), `crates/swarm_core` (protocol / vision /
   control), and the three controller crates. Pure-Rust CV: HSV, BFS blobs,
   convex-hull rotating-calipers minAreaRect, FAST-9 + rotated BRIEF-256,
   Lowe ratio 0.8, RANSAC homography, JPEG codec — zero OpenCV.
3. **World updated**: `worlds/drone_handoff.wbt` — target Solid is now
   `DEF TARGET` (the Rust supervisor moves it via the translation field) and
   both `terminal_camera` nodes rotated 1.15 -> **1.35 rad** (bottom-edge plate
   clipping fix). `build_realmap_world.py` — added `TARGET_END_XY=(16.5, 11.0)`
   and keeps the moving-target lane clear of buildings; `LOITER_XY=(15, 10)`.
4. **Harness swapped**: `run_verification.py` now auto-runs
   `cargo build --release`, deploys the exes + `Controller.dll` into
   `controllers/<name>/`, runs Webots, and grades **13 milestones**.
   `build_dashboard.py` reports the 13 milestones + Rust step-latency stats.
   The retired Python-era files are kept as
   `run_verification_python10.py` / `build_dashboard_python10.py`, and the
   Python controllers as `<name>_python_fallback.py`.
5. **Realmap world regenerated** (elevation mode, 0.26 m relief, 13 real OSM
   buildings).

## Bugs fixed during bring-up

* **FFI (build)**: Webots R2025a renamed the root-node accessor —
  `wb_supervisor_get_root` no longer exists in `Controller.lib`
  (LNK2019). Fixed to `wb_supervisor_node_get_root` (per
  `include/controller/c/webots/supervisor.h`).
* **Type inference (build)**: `max_gps_wander_m` needed an explicit `: f64`.
* **CRC32 round-trip (protocol, real bug)**: serde_json's DEFAULT float parser
  is not correctly rounded — parse -> re-serialize could flip an f64 by 1 ulp,
  changing the canonical JSON bytes and breaking the packet CRC. Reproduced
  with `crates/swarm_core/examples/crc_roundtrip.rs` (seq-0 packet passed,
  seq-2 packet failed). Fixed with `serde_json = { version = "1.0", features =
  ["float_roundtrip"] }` in `crates/swarm_core/Cargo.toml`. Verified in-sim:
  drop logs went from spurious `crc_invalid` entries to exactly the spec's
  emulation (F1: seq 0; F2: seq 0 + 1, all `link_drop_emulated`).
* **Harness (typo in the ported grader)**: `deconfliction_pass` looked up
  `followers["FOLLOWER_2"]` (KeyError) instead of `"Follower_Drone_02"`. Fixed.

## First integration results (both runs: seed 0, realmap world)

Run 1: outcome `crash:FOLLOWER_2` at t=153.4 s, target displacement 6.68 m.
Run 2 (after CRC fix): outcome `crash:FOLLOWER_2` at t=151.2 s, displacement
6.44 m. Grading (run 2): **3/13** — PASS on 1_raycast_precision,
5_obstacle_avoidance, 7_mapless_sensor_avoidance. Working already: scout mark
(t=91.6 s, raycast precision pass), packet reception both followers (90 ORB
features each), moving-target supervisor (6.4-6.7 m displacement), wind
modeling, lidar clustering (26k-34k hits/drone), 13-milestone grading.

## Open failure signatures for the Phase 11 loop (start of Bug #45)

* **Bug #45 — F2 hold-hover tumble**: F2 parks on its frozen hold spot
  (HOLD_RING_M logic, full terminal boost gs=7.5 engaged because goal_dist~0,
  z=6.5, speed~0) for ~55 s while waiting for F1; attitude then diverges over
  ~1.5 s (slight xy jerk at t~151.2, then tumbling descent from 6.49 m to
  ground by 152.5 s). All three controller processes were ALIVE at t=152
  (heartbeats current) — a control divergence, not a crash. Reproduced twice
  at nearly the same sim time. Suspects: integral + max-boost limit cycle
  during long holds (the Python-era bug #36/#41 family), possibly triggered by
  a gust peak + OU wander state. Candidate mitigations: decay the boost toward
  1.0 the longer a hold persists, relax the integral during holds, or reduce
  max_terminal_boost when the nav goal equals the current position.
* **Bug #46 — F1 macro-nav crawl**: F1 averaged ~0.13 m/s in Stage 1 (should
  cruise ~1-2 m/s): it took ~56 s to cover ~7 m while threading the 13-building
  field and avoiding the phantom cluster formed by lidar hits on the parked F2
  (F1 tracked 1 persistent cluster; F2's hold spot sits ~5 m off F1's path).
  The Rust `scan_best_angle` picks MAX clearance over the full +-180 deg scan
  each call, while the tuned Python version was greedy-first-clear + stickier;
  consider restoring the Python steering policy or excluding lidar hits that
  match the teammate's broadcast position. F1 never reached Stage 2, so no
  locks, and milestones 2/3/4/6/8/9/10/12/13 all chain-fail on it.
* Milestone 11 (`real_osm_buildings_placed >= 5`) failed grading because the
  realmap run placed 13 buildings but `realmap_meta.json` -> `world_meta.json`
  key copy happens in `run_trial`; verify after the first clean mission run.

## Regression check available

`cargo run --release -p swarm_core --example crc_roundtrip` must print
`crc_valid = true` for both packets.

## Phase 11 loop outcome (2026-11-13) — RESOLVED

The two open failure signatures were root-caused and fixed in the first tuning
iteration; full details in `PROJECT_HANDOVER.md` (bug log #45-#49):

* **#45 F2 hold-hover tumble** -> Stage-1 hold no longer releases when the moving
  target drifts past the 7 m ring, and the terminal boost is Stage-2-only.
* **#46 F1 macro-nav crawl** -> greedy first-clear steering restored (the
  max-clearance +-180 deg scan was the throttle).
* **#47** harness wall-clock timeout 320 s -> 600 s (runs need ~RTF 0.55 x 190 s sim).
* **#48** cross-scale visual ID: added a 4-level scale pyramid to
  `extract_orb_features` (offline proof: `match_diag` example, 0 -> 58 inliers).
* **#49** M11 meta key: builder now writes `real_osm_buildings_placed` too.

Result: `--mc 5` = **5/5 trials, 13/13 milestones each**, terminal errors
3.05-4.12 cm (mean 3.60 cm) on the moving target, missions complete t = 176-188 s.

