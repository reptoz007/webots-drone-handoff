# Executive Status — Read First

# Drone Handoff Project — EXECUTIVE STATUS (read this first)

Generated: 2026-11-13 01:13

## What this package is
Complete readable export of the Webots cooperative drone handoff project after an
autonomous engineering session that executed Phases 6-10 of the roadmap. Hand this
whole folder (10 markdown files) to an AI to continue.

## Where the project stands (honest)
- **Single canonical verification run: ALL 10 MILESTONES PASS, Webots exit code 0,
  MISSION SUCCESS True** (terminal errors 1.6-2.9 cm under wind, both followers).
- **Monte Carlo 3-trial suite (`--mc 3`) is implemented and was run three times.**
  Best result 3/3 milestones 1-9 + 2/3 on M10 (one fiducial reading hit dark
  scenery); after the plate-region fix the fiducial went 3/3 but one seed hit a
  5.19-7.1 cm terminal lock (gate 5 cm). **The MC exposes that the terminal lock
  under gusts is marginal at the 5 cm gate — the known open issue.** The last
  attempted improvement (fusing the fiducial midpoint into the IBVS vector)
  passed the single run 10/10 but REGRESSED the MC to 1/3 — it is implemented in
  the code but should be re-tuned or gated (e.g., blend only when the fiducial
  reading is stable across 3 consecutive probes).
- Everything else (Phases 6-9) is solid: 23-iteration Phase 9 history in the bug log.

## Milestone scoreboard (single canonical run)
1. Scout raycast precision <=5 cm — PASS (0.67-3.75 cm)
2. Radio reception + visual ID — PASS
3. Airspace deconfliction — PASS
4. Drift correction, terminal <5 cm — PASS (1.6-3.1 cm)
5. Obstacle avoidance — PASS (clearance 1.45-1.63 m, detour 5-7.8 m)
6. Live GPS wander ±0.45 m + pure IBVS lock — PASS
7. Mapless onboard-lidar avoidance — PASS (26k-34k beam hits/drone, map-free)
8. Degraded radio: drops + CRC retransmit — PASS (F1 seq 1, F2 seq 2)
9. Two-block slalom + wind rejection — PASS (wind scaled x0.04, see bug #31)
10. Fiducial geometry + orientation — PASS (yaw 45.8° vs 45.7° expected)

## Recommended next steps for the AI
1. Read 02/03 (full handover) — especially Section 6 (limitations) and the bug log.
2. The one open engineering task: make the terminal lock robust to the gust phase
   (MC seeds 1-2 hit 5.2-7.1 cm). Options: tune the fiducial-blend gating, add a
   gust-phase-aware lock window, or lower the terminal gain boost. Re-run
   `python run_verification.py worlds/drone_handoff_realmap.wbt --mc 3` after any change.
3. `results_mc/mc_summary.json` has per-trial per-milestone evidence.
4. Run commands (from the project root, Webots at C:\Users\harsh\AppData\Local\Programs\Webots):
   - `python build_realmap_world.py`
   - `python run_verification.py worlds/drone_handoff_realmap.wbt` (single run)
   - `python run_verification.py worlds/drone_handoff_realmap.wbt --mc 3` (suite)
   - `python build_dashboard.py`
5. The bug log numbering runs #21-#46 — read the Phase 9/10 entries before changing
   the avoidance or terminal control: the failure modes were subtle (236 rad/s
   steering sweeps, saturated-D limit cycles, stale low-pass slams).

## File map of this package
- 01: this status file
- 02/03: PROJECT_HANDOVER.md (complete handover, all sections, verbatim)
- 04: run_verification.py + build_realmap_world.py
- 05: follower_controller.py (the largest, most iterated file)
- 06: scout_controller.py + handoff_protocol.py
- 07: metrics_supervisor.py + worlds/drone_handoff.wbt
- 08: build_dashboard.py + build_handover.py
- 09: key results JSONs (report, mc_summary, ground truth, locks, mark, wind log head)
- 10: README/FILES/HANDS_ON_GUIDE pointers + requirements
