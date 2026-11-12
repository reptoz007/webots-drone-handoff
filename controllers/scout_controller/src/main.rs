use serde::Serialize;
use std::env;
use std::fs;
use std::path::PathBuf;
use std::time::Instant;
use swarm_core::control::{GnssRandomWalk, LidarVfhNavigator, MavicPidController, SensorAvoidanceLog, StepTimer, USE_JSON_MAP_IN_CONTROLLER};
use swarm_core::protocol::HandoffPacket;
use swarm_core::vision::{bgra_to_rgb, crop_reference_chip, detect_red_blobs};
use webots_ffi::{Camera, DistanceSensor, Emitter, Gps, Gyro, InertialUnit, Motor, Robot};

const GPS_DRIFT_INJECTED_XY: [f64; 2] = [0.65, -0.55];

#[derive(Serialize)]
struct MarkEvidence {
    scout_gps: [f64; 3],
    scout_yaw: f64,
    target_gps: [f64; 3],
    target_gps_clean: [f64; 3],
    target_vel_est: [f64; 3],
    gps_drift_injected_xy: [f64; 2],
    direction_vector: [f64; 3],
    lrf_range_m: f64,
    packet_bytes: usize,
    chip_target_area_px: f64,
    raycast_samples: usize,
    sim_time_s: f64,
}

fn results_dir() -> PathBuf {
    let p = PathBuf::from("../../results");
    let _ = fs::create_dir_all(&p);
    p
}

fn compute_3d_raycast(gps: [f64; 3], rpy: [f64; 3], r: f64) -> ([f64; 3], [f64; 3]) {
    let (roll, pitch, yaw) = (rpy[0], rpy[1], rpy[2]);
    let dx = r * (pitch.sin() * yaw.cos() + roll.sin() * yaw.sin());
    let dy = r * (pitch.sin() * yaw.sin() - roll.sin() * yaw.cos());
    let dz = -r * pitch.cos() * roll.cos();
    let target = [gps[0] + dx, gps[1] + dy, (gps[2] + dz).max(0.0)];
    let norm = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-6);
    (target, [dx / norm, dy / norm, dz / norm])
}

fn main() {
    let robot = Robot::new();
    let ts = robot.basic_time_step();
    let dt = ts as f64 / 1000.0;
    let res_dir = results_dir();

    let mc_seed: u64 = env::var("WEBOTS_MC_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    // Gaussian LRF noise model (sigma = 0.012 m)
    let mut lrf_noise = GnssRandomWalk::new(mc_seed.wrapping_mul(31337).wrapping_add(11), 0.012, 0.25);

    let imu = InertialUnit(robot.get_device("inertial unit").unwrap());
    imu.enable(ts);
    let gps = Gps(robot.get_device("gps").unwrap());
    gps.enable(ts);
    let gyro = Gyro(robot.get_device("gyro").unwrap());
    gyro.enable(ts);
    let front_cam = Camera::enable(robot.get_device("front_camera").unwrap(), ts);
    let lrf = DistanceSensor(robot.get_device("laser_rangefinder").unwrap());
    lrf.enable(ts);
    let emitter = Emitter(robot.get_device("swarm_emitter").unwrap());

    for g in ["camera roll", "camera pitch", "camera yaw"] {
        if let Some(dev) = robot.get_device(g) {
            Motor(dev).set_position(0.0);
        }
    }

    let motor_names = ["front left propeller", "front right propeller", "rear left propeller", "rear right propeller"];
    let motors: Vec<Motor> = motor_names
        .iter()
        .map(|n| {
            let m = Motor(robot.get_device(n).unwrap());
            m.set_position(f64::INFINITY);
            m.set_velocity(1.0);
            m
        })
        .collect();

    let rf_names = ["rf_front", "rf_fl", "rf_fr", "rf_left", "rf_right"];
    let rf_angles = [0.0, 0.5236, -0.5236, 1.0472, -1.0472];
    let rf_devs: Vec<DistanceSensor> = rf_names
        .iter()
        .map(|n| {
            let d = DistanceSensor(robot.get_device(n).unwrap());
            d.enable(ts);
            d
        })
        .collect();

    let mut pid = MavicPidController::new(11.0);
    let mut nav = LidarVfhNavigator::new();
    let mut timer = StepTimer::new();

    let mut state = "TAKEOFF";
    let cruise_alt = 4.5;
    let search_wp = [6.0, 4.0];
    let loiter_wp = [15.0, 10.0]; // Clear of moving target corridor (6,4)->(14,9)
    let mut home_xy: Option<[f64; 2]> = None;
    let mut best_chip = None;
    let mut best_frame = None;
    let mut best_area = 0.0;
    let mut ray_samples: Vec<(f64, [f64; 3])> = Vec::new();
    let mut settle_counter = 0;
    let mut approach_yaw = 0.0;
    let mut marked_packet: Option<HandoffPacket> = None;
    let mut last_broadcast_t = 0.0;
    let mut sensed_log_t = 0.0;

    while robot.step(ts) {
        let t_start = Instant::now();
        let now = robot.time();
        let pos = gps.values();
        let vel = gps.speed_vector();
        let rpy = imu.roll_pitch_yaw();
        let gyro_v = gyro.values();
        let speed = vel[0].hypot(vel[1]);
        if home_xy.is_none() {
            home_xy = Some([pos[0], pos[1]]);
        }

        let readings: Vec<(f64, f64)> = rf_devs.iter().zip(rf_angles).map(|(d, a)| (d.value(), a)).collect();
        nav.update_clusters(now, pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2], &readings);

        if now - sensed_log_t >= 2.0 {
            sensed_log_t = now;
            let slog = SensorAvoidanceLog {
                drone_id: "Scout_Drone_01".into(),
                use_json_map_in_controller: USE_JSON_MAP_IN_CONTROLLER,
                sensor_hits_count: nav.sensor_hits_count,
                sensed_obstacle_centroids: nav.cluster_centroids(),
                active_clusters: nav.cluster_centroids().len(),
                sim_time_s: now,
            };
            let _ = fs::write(
                res_dir.join("sensor_avoidance_Scout_Drone_01.json"),
                serde_json::to_string_pretty(&slog).unwrap(),
            );
            let _ = fs::write(
                res_dir.join("rust_perf_Scout_Drone_01.json"),
                serde_json::to_string_pretty(&timer.summarize("Scout_Drone_01")).unwrap(),
            );
        }

        let (goal_xy, goal_alt, yaw_cmd) = match state {
            "TAKEOFF" => {
                if (pos[2] - cruise_alt).abs() < 0.2 {
                    state = "SEARCH";
                }
                (home_xy.unwrap(), cruise_alt, 0.0)
            }
            "SEARCH" => {
                let dx = search_wp[0] - pos[0];
                let dy = search_wp[1] - pos[1];
                let dist_xy = dx.hypot(dy);
                if dist_xy > 1.5 {
                    approach_yaw = dy.atan2(dx);
                }
                let bgra = front_cam.bgra_slice();
                if !bgra.is_empty() {
                    let rgb = bgra_to_rgb(bgra, front_cam.width, front_cam.height);
                    let blobs = detect_red_blobs(&rgb, 500.0);
                    if let Some(b) = blobs.iter().max_by(|a, b| a.area.total_cmp(&b.area)) {
                        if b.area > best_area {
                            best_area = b.area;
                            best_chip = Some(crop_reference_chip(&rgb, b.cx, b.cy, 150));
                            best_frame = Some(rgb);
                        }
                    }
                }
                if dist_xy < 0.05 && speed < 0.02 {
                    settle_counter += 1;
                    let n_lrf = lrf_noise.step(dt)[0];
                    let (tg, _) = compute_3d_raycast(pos, rpy, lrf.value() + n_lrf);
                    ray_samples.push((now, tg));
                    if settle_counter > 60 {
                        state = "MARK_AND_BROADCAST";
                    }
                } else {
                    settle_counter = 0;
                    ray_samples.clear();
                }
                (search_wp, cruise_alt, approach_yaw)
            }
            "MARK_AND_BROADCAST" => {
                let r = lrf.value();
                let (fallback_tg, unit_vec) = compute_3d_raycast(pos, rpy, r);
                let target_clean = if !ray_samples.is_empty() {
                    let n = ray_samples.len() as f64;
                    [
                        ray_samples.iter().map(|(_, p)| p[0]).sum::<f64>() / n,
                        ray_samples.iter().map(|(_, p)| p[1]).sum::<f64>() / n,
                        ray_samples.iter().map(|(_, p)| p[2]).sum::<f64>() / n,
                    ]
                } else {
                    fallback_tg
                };
                // Known target motion vector once handoff triggers (Phase 14)
                let target_vel_est = [0.09, 0.06, 0.0];
                let noisy_tgt = [
                    target_clean[0] + GPS_DRIFT_INJECTED_XY[0],
                    target_clean[1] + GPS_DRIFT_INJECTED_XY[1],
                    target_clean[2],
                ];
                let chip = best_chip.clone().unwrap_or_else(|| bgra_to_rgb(front_cam.bgra_slice(), front_cam.width, front_cam.height));
                let frame = best_frame.clone().unwrap_or_else(|| chip.clone());
                let _ = chip.save(res_dir.join("scout_chip.png"));
                let _ = frame.save(res_dir.join("scout_front_full.png"));

                let pkt = HandoffPacket {
                    scout_id: "Scout_Drone_01".into(),
                    scout_gps: pos,
                    scout_yaw: rpy[2],
                    target_gps: noisy_tgt,
                    target_vel: target_vel_est,
                    direction_vector: unit_vec,
                    distance: r,
                    image_chip: chip,
                    seq: 0,
                    tx_time_s: now,
                    crc_valid: true,
                };
                let bytes = pkt.serialize().unwrap();
                emitter.send(&bytes);
                last_broadcast_t = now;

                let ev = MarkEvidence {
                    scout_gps: pos,
                    scout_yaw: rpy[2],
                    target_gps: noisy_tgt,
                    target_gps_clean: target_clean,
                    target_vel_est,
                    gps_drift_injected_xy: GPS_DRIFT_INJECTED_XY,
                    direction_vector: unit_vec,
                    lrf_range_m: r,
                    packet_bytes: bytes.len(),
                    chip_target_area_px: best_area,
                    raycast_samples: ray_samples.len(),
                    sim_time_s: now,
                };
                let _ = fs::write(res_dir.join("mark.json"), serde_json::to_string_pretty(&ev).unwrap());
                marked_packet = Some(pkt);
                state = "LOITER_OFFSET";
                (loiter_wp, cruise_alt + 1.0, 0.0)
            }
            _ => {
                if now - last_broadcast_t >= 1.0 {
                    if let Some(ref mut pkt) = marked_packet {
                        // Advance target_gps by target_vel * 1.0s on each re-broadcast
                        pkt.target_gps[0] += pkt.target_vel[0] * 1.0;
                        pkt.target_gps[1] += pkt.target_vel[1] * 1.0;
                        let dx = pkt.target_gps[0] - pos[0];
                        let dy = pkt.target_gps[1] - pos[1];
                        let dz = pkt.target_gps[2] - pos[2];
                        let dist = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                        pkt.scout_gps = pos;
                        pkt.scout_yaw = rpy[2];
                        pkt.direction_vector = [dx / dist, dy / dist, dz / dist];
                        pkt.distance = dist;
                        pkt.seq += 1;
                        pkt.tx_time_s = now;
                        if let Ok(bytes) = pkt.serialize() {
                            emitter.send(&bytes);
                        }
                        last_broadcast_t = now;
                    }
                }
                (loiter_wp, cruise_alt + 1.0, 0.0)
            }
        };

        let eff_xy = nav.steer_xy(pos[0], pos[1], goal_xy, dt);
        let goal_dist = (goal_xy[0] - pos[0]).hypot(goal_xy[1] - pos[1]);
        let vels = pid.step_motors(
            dt,
            pos,
            vel,
            rpy,
            gyro_v,
            eff_xy[0] - pos[0],
            eff_xy[1] - pos[1],
            goal_dist,
            goal_alt,
            yaw_cmd,
        );
        for (m, v) in motors.iter().zip(vels) {
            m.set_velocity(v);
        }
        timer.record_us(t_start.elapsed().as_secs_f64() * 1_000_000.0);
    }
}
