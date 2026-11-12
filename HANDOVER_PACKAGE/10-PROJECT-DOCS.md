# Project Docs (README/FILES/requirements + guide pointer)

## README.md
```markdown
# Webots Cooperative Drone Target Handoff Simulation

## Prerequisites
1. Install **Webots R2023b** (or newer; verified on R2025a) from [cyberbotics.com](https://cyberbotics.com).
2. Ensure Python 3.9+ is configured in Webots (`Tools -> Preferences -> Python command`).
3. Install required Python libraries in the Python environment used by Webots:
   ```bash
   pip install -r requirements.txt
   ```

## How to Run

1. Launch Webots.
2. Open the world file: `File -> Open World... -> webots_drone_handoff/worlds/drone_handoff.wbt`.
3. Press the **Play (Real-time)** button in the top toolbar.

### Automated verification (headless-friendly)

```bash
python run_verification.py
```

Runs the world, auto-quits when both followers arrive over the marked target, and writes
`results/report.json` plus captured images (`scout_chip.png`, `received_chip_*.png`,
`id_frame_*.png`, `lock_frame_*.png`) and ground-truth tracks (`tracks.csv`).

### Real-geography world (Washington Monument, Washington DC)

A second world, `worlds/drone_handoff_realmap.wbt`, runs the identical mission on a
400 x 400 m patch of real terrain. The world origin (0,0) is the real anchor point
**38.8895 N, -77.0353 W** (Washington Monument, National Mall, Washington DC), the ground
is textured with real **Esri World Imagery satellite photography** (OpenStreetMap tiles as
automatic fallback), the relief is real SRTM elevation, and **real OSM 3D buildings**
(footprints via the free Overpass API) stand around the perimeter, kept clear of the
mission flight corridors. The red target at world (6.0, 4.0) therefore sits about 7 m NE
of the Monument center, on the real plaza — the drone cameras show actual aerial photography.

```bash
python build_realmap_world.py                          # downloads free tiles once (cached)
python run_verification.py worlds/drone_handoff_realmap.wbt
```

Free data sources:
- Imagery: Esri World Imagery (Esri, Maxar, Earthstar Geographics, and the GIS User Community).
- Buildings: OSM footprints via the Overpass API (c) OpenStreetMap contributors, ODbL.
- Elevation: SRTM via the public Mapzen/AWS terrain tiles (terrarium, z15).

The builder keeps the real elevation only if the flight-area relief is small enough for the
fixed-altitude controllers; otherwise it falls back to a flat floor with the same map texture.

## Mission Flow

All drones carry **forward-facing cameras** (fixed 17 deg downtilt, so they look ahead at the
ground in front of them, like a real drone camera):

1. **Takeoff & Search:** `Scout_Drone` climbs to 4.5 m at `(-6, -4)` and flies toward the
   suspected target zone `(6.0, 4.0)`, yawing to face the target. Its forward camera detects the
   red target (tight HSV mask) and continuously keeps the closest, largest view as the reference
   image chip.
2. **Mark & Broadcast:** Once hovering over the zone, the Scout fires its downward
   `laser_rangefinder` (narrow 0.02 rad aperture), raycasts the exact 3D ground coordinates from
   GPS + IMU attitude, and broadcasts the compressed `HandoffPacket` — target coordinates, scout
   position/heading, and the JPEG reference chip — over radio Channel 1. It then moves to
   `(8.5, 4.0)` to clear the airspace.
3. **Stage 1 — GPS Navigation with visual ID:** Both followers receive the packet, log the Scout's
   location and bearing, and fly to the marked coordinates while facing the target. During the
   approach, each follower confirms the target ahead **visually** with ORB + Lowe's ratio test +
   RANSAC homography against the Scout's reference chip.
4. **Terminal arrival:** Each follower hovers over the marked GPS point; arrival is logged when it
   holds within 15 cm of the marked coordinates at low speed. The supervisor records ground truth
   and ends the mission when both followers have arrived.

## Notes

- `Follower_Drone_2` cruises at 6.5 m (2 m above Follower 1) for airspace deconfliction; it
  receives the same broadcast on Channel 1.
- The laser rangefinder's aperture is set to 0.02 rad — the default (1.5708 rad) renders a huge
  red cone in camera images, which pollutes color detection and feature matching.
- The `metrics_supervisor` robot only records ground truth and ends the run; it does not
  influence flight.
- Webots GPS is idealized (no noise); on real hardware the macro stage inherits GPS error
  (~1-3 m). The forward-camera target ID demonstrates how the image chip lets each receiver
  verify it has the right target; terminal precision in this simulation comes from the idealized
  GPS, which is why a real deployment would add a visual or RTK terminal stage.

```

## FILES.md
```markdown
# Project File Inventory

Project root: `C:\Users\harsh\Documents\Qoder\2026-11-10\c386c8d6\webots_drone_handoff`

## Source & config

| File | Location | Purpose |
|---|---|---|
| `README.md` | `webots_drone_handoff\README.md` | Setup, run instructions, mission flow, notes |
| `requirements.txt` | `webots_drone_handoff\requirements.txt` | Python dependencies (numpy, opencv-python) |
| `run_verification.py` | `webots_drone_handoff\run_verification.py` | Headless verification runner (optional world path arg); launches Webots, builds `results\report.json` |
| `build_dashboard.py` | `webots_drone_handoff\build_dashboard.py` | Builds the mission results dashboard HTML from `results\` |
| `build_realmap_world.py` | `webots_drone_handoff\build_realmap_world.py` | Downloads free Esri satellite tiles (+OSM fallback) + SRTM elevation + OSM 3D buildings (Overpass), generates the real-geography world |
| `drone_handoff.wbt` | `webots_drone_handoff\worlds\drone_handoff.wbt` | Webots world: 3 Mavic drones, ground target, radio channel 1 (abstract checkerboard) |
| `drone_handoff_realmap.wbt` | `webots_drone_handoff\worlds\drone_handoff_realmap.wbt` | Same mission on real terrain: Washington Monument DC, Esri satellite texture, SRTM relief, OSM 3D buildings |
| `.drone_handoff.wbproj` | `webots_drone_handoff\worlds\.drone_handoff.wbproj` | Webots world project metadata |
| `realmap_esri.png` | `webots_drone_handoff\worlds\textures\realmap_esri.png` | Stitched Esri satellite texture (1024x1024, 400 m box; OSM fallback writes here too) |
| `realmap_meta.json` | `webots_drone_handoff\worlds\textures\realmap_meta.json` | Anchor lat/lon, box size, relief, imagery source, building counts, attribution |
| `elevation_cache.npy` | `webots_drone_handoff\worlds\textures\elevation_cache.npy` | Cached SRTM elevation grid (avoids re-download) |

## Controllers

| File | Location | Purpose |
|---|---|---|
| `scout_controller.py` | `webots_drone_handoff\controllers\scout_controller\scout_controller.py` | Scout: search, red-target detection, laser raycast marking, packet broadcast |
| `handoff_protocol.py` | `webots_drone_handoff\controllers\scout_controller\handoff_protocol.py` | `HandoffPacket` serialize/deserialize (JSON + base64 JPEG chip) |
| `follower_controller.py` | `webots_drone_handoff\controllers\follower_controller\follower_controller.py` | Followers: receive packet, GPS navigation, ORB visual ID, terminal hover |
| `handoff_protocol.py` | `webots_drone_handoff\controllers\follower_controller\handoff_protocol.py` | Same protocol module (copy per controller dir, as Webots requires) |
| `metrics_supervisor.py` | `webots_drone_handoff\controllers\metrics_supervisor\metrics_supervisor.py` | Ground truth: writes `tracks.csv`, ends run when both followers lock |

## Verification results (latest run)

| File | Location | Content |
|---|---|---|
| `mark.json` | `webots_drone_handoff\results\mark.json` | Scout's marked target GPS, raycast vector, packet size |
| `handoff_Follower_Drone_01.json` | `webots_drone_handoff\results\handoff_Follower_Drone_01.json` | Follower 1 packet receipt telemetry |
| `handoff_Follower_Drone_02.json` | `webots_drone_handoff\results\handoff_Follower_Drone_02.json` | Follower 2 packet receipt telemetry |
| `visual_id_Follower_Drone_01.json` | `webots_drone_handoff\results\visual_id_Follower_Drone_01.json` | Follower 1 ORB/RANSAC target confirmation |
| `visual_id_Follower_Drone_02.json` | `webots_drone_handoff\results\visual_id_Follower_Drone_02.json` | Follower 2 ORB/RANSAC target confirmation |
| `lock_Follower_Drone_01.json` | `webots_drone_handoff\results\lock_Follower_Drone_01.json` | Follower 1 terminal arrival (error, time) |
| `lock_Follower_Drone_02.json` | `webots_drone_handoff\results\lock_Follower_Drone_02.json` | Follower 2 terminal arrival (error, time) |
| `ground_truth.json` | `webots_drone_handoff\results\ground_truth.json` | Supervisor outcome + final true positions |
| `report.json` | `webots_drone_handoff\results\report.json` | Consolidated verification report |
| `world_meta.json` | `webots_drone_handoff\results\world_meta.json` | Real-map run metadata (location/attribution), copied by the runner |
| `tracks.csv` | `webots_drone_handoff\results\tracks.csv` | Full ground-truth trajectories (t, drone, x, y, z) |
| `scout_chip.png` | `webots_drone_handoff\results\scout_chip.png` | 240x240 reference image chip broadcast in the packet |
| `scout_front_full.png` | `webots_drone_handoff\results\scout_front_full.png` | Scout's full forward-camera frame at mark time |
| `received_chip_Follower_Drone_01.png` | `webots_drone_handoff\results\received_chip_Follower_Drone_01.png` | Chip as decoded by follower 1 |
| `received_chip_Follower_Drone_02.png` | `webots_drone_handoff\results\received_chip_Follower_Drone_02.png` | Chip as decoded by follower 2 |
| `id_frame_Follower_Drone_01.png` | `webots_drone_handoff\results\id_frame_Follower_Drone_01.png` | Follower 1 visual-ID frame (target in view) |
| `id_frame_Follower_Drone_02.png` | `webots_drone_handoff\results\id_frame_Follower_Drone_02.png` | Follower 2 visual-ID frame (target in view) |
| `lock_frame_Follower_Drone_01.png` | `webots_drone_handoff\results\lock_frame_Follower_Drone_01.png` | Follower 1 camera frame at terminal hover |
| `lock_frame_Follower_Drone_02.png` | `webots_drone_handoff\results\lock_frame_Follower_Drone_02.png` | Follower 2 camera frame at terminal hover |
| `scene_dump.txt` | `webots_drone_handoff\results\scene_dump.txt` | One-time scene state diagnostic |
| `webots_console.log` | `webots_drone_handoff\results\webots_console.log` | Full Webots console output of the run |

## Archived previous run

| Path | Purpose |
|---|---|
| `webots_drone_handoff\results_prev\` | Copy of `results\` from the run before the latest verification (kept by `run_verification.py`) |

```

## requirements.txt
```text
numpy>=1.24.0
opencv-python>=4.8.0
```

## HANDS_ON_GUIDE.md is 61 KB — present verbatim in the project root; request it if needed.