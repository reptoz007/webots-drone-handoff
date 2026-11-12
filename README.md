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
