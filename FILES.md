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
