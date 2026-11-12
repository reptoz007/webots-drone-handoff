use serde::{Deserialize, Serialize};
use std::env;
use std::fs;
use std::path::PathBuf;
use std::time::Instant;
use swarm_core::control::{GnssRandomWalk, LidarVfhNavigator, MavicPidController, SensorAvoidanceLog, StepTimer, USE_JSON_MAP_IN_CONTROLLER};
use swarm_core::protocol::HandoffPacket;
use swarm_core::vision::{bgra_to_rgb, detect_red_blobs, extract_orb_features, match_visual_id, verify_fiducial, Keypoint};
use webots_ffi::{Camera, DistanceSensor, Emitter, Gps, Gyro, InertialUnit, Motor, Receiver, Robot};

const TERMINAL_CAM_PITCH: f64 = 1.35; // Eliminates bottom-edge plate clipping at 4.5 m hover
const FOV: f64 = 1.047;
const HOMING_ACCEPT_RADIUS_M: f64 = 3.0;
const HOLD_RING_M: f64 = 7.0;
const TEAMMATE_FALLBACK_S: f64 = 115.0;
const VACATE_OFFSET_XY: [f64; 2] = [2.2, 1.5]; // Relative to current moving target so it never occludes

#[derive(Deserialize)]
struct CustomConfig {
    id: Option<String>,
    home: Option<[f64; 2]>,
    alt: Option<f64>,
}

#[derive(Serialize, Clone)]
struct DropRecord {
    seq: i32,
    tx_time_s: f64,
    rx_time_s: f64,
    reason: String,
}

#[derive(Serialize)]
struct HandoffEvidence {
    drone: String,
    scout_id: String,
    scout_gps: [f64; 3],
    target_gps: [f64; 3],
    target_vel: [f64; 3],
    approach_bearing_deg: f64,
    distance_at_receipt_m: f64,
    ref_orb_features: usize,
    packet_bytes: usize,
    packet_seq: i32,
    packet_tx_time_s: f64,
    packet_crc_valid: bool,
    dropped_packets: Vec<DropRecord>,
    decode_latency_ms: f64,
    receipt_lag_from_mark_ms: f64,
    sim_time_s: f64,
}

#[derive(Serialize)]
struct VisualIdEvidence {
    drone: String,
    gps: [f64; 3],
    orb_good_matches: usize,
    ransac_inliers: usize,
    red_target_area_px: f64,
    sim_time_s: f64,
}

#[derive(Serialize)]
struct LockEvidence {
    drone: String,
    gps: [f64; 3],
    perceived_gps_at_lock: [f64; 3],
    noisy_target_gps: [f64; 3],
    refined_target_xy: [f64; 2],
    rel_target_xy: [f64; 2],
    stage2_triggered: bool,
    homing_update_count: usize,
    last_red_seen_sim_time: f64,
    gnss_model: String,
    max_gps_wander_m: f64,
    gps_wander_at_lock_xy: [f64; 2],
    handoff_packet_seq: i32,
    handoff_crc_valid: bool,
    fiducial_verified: bool,
    fiducial_target_yaw_deg: Option<f64>,
    fiducial_marker_separation_m: Option<f64>,
    terminal_error_m: f64,
    visual_id_confirmed: bool,
    sim_time_s: f64,
    time_handoff_to_lock_s: f64,
}

fn results_dir() -> PathBuf {
    let p = PathBuf::from("../../results");
    let _ = fs::create_dir_all(&p);
    p
}

fn rot_matrix(roll: f64, pitch: f64, yaw: f64) -> [[f64; 3]; 3] {
    let (cr, sr) = (roll.cos(), roll.sin());
    let (cp, sp) = (pitch.cos(), pitch.sin());
    let (cy, sy) = (yaw.cos(), yaw.sin());
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

fn mat_vec(r: &[[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2],
        r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
        r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2],
    ]
}

fn main() {
    let robot = Robot::new();
    let ts = robot.basic_time_step();
    let dt = ts as f64 / 1000.0;
    let res_dir = results_dir();

    let cd = robot.custom_data();
    let cfg: CustomConfig = serde_json::from_str(&cd).unwrap_or(CustomConfig { id: None, home: None, alt: None });
    let drone_id = cfg.id.unwrap_or_else(|| "Follower_Drone_01".into());
    let home_xy = cfg.home.unwrap_or([-4.0, -3.0]);
    let cruise_alt = cfg.alt.unwrap_or(4.5);
    let is_primary = drone_id.ends_with("01");

    let mc_seed: u64 = env::var("WEBOTS_MC_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let id_hash: u64 = drone_id.bytes().fold(0u64, |acc, b| acc.wrapping_mul(31).wrapping_add(b as u64));
    // Phase 13: Ornstein-Uhlenbeck stochastic GNSS random walk (sigma = 0.35 m, tau = 12.0 s)
    let mut gnss_walk = GnssRandomWalk::new(mc_seed.wrapping_mul(104729).wrapping_add(id_hash), 0.35, 12.0);

    let imu = InertialUnit(robot.get_device("inertial unit").unwrap());
    imu.enable(ts);
    let gps = Gps(robot.get_device("gps").unwrap());
    gps.enable(ts);
    let gyro = Gyro(robot.get_device("gyro").unwrap());
    gyro.enable(ts);
    let front_cam = Camera::enable(robot.get_device("front_camera").unwrap(), ts);
    let term_cam = Camera::enable(robot.get_device("terminal_camera").unwrap(), ts);
    let receiver = Receiver(robot.get_device("swarm_receiver").unwrap());
    receiver.enable(ts);
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

    // Capped at 6.5x boost (7.5x total) so terminal gusts/fiducial updates never saturate attitude
    let mut pid = MavicPidController::new(6.5);
    let mut nav = LidarVfhNavigator::new();
    let mut timer = StepTimer::new();

    let mut state = "IDLE_HOVER";
    let mut handoff_data: Option<HandoffPacket> = None;
    let mut ref_kp: Vec<Keypoint> = Vec::new();
    let mut handoff_recv_time: Option<f64> = None;
    let mut stage1_engage_time: Option<f64> = None;
    let mut first_tx_seen: Option<f64> = None;
    let mut dropped_log: Vec<DropRecord> = Vec::new();

    let mut visual_id_logged = false;
    let mut lock_achieved_logged = false;
    let mut lock_streak = 0usize;
    let mut approach_yaw = 0.0;
    let mut rel_target_xy: Option<[f64; 2]> = None;
    let mut homing_update_count = 0usize;
    let mut last_red_seen = -1.0;
    let mut stall_reject_streak = 0usize;
    let mut fiducial_streak = 0usize;
    // Bug #50: at the lock instant a single fiducial probe can miss (F2's markers
    // are ~150 px^2 at 6.5 m altitude, so one noisy frame early-returns None).
    // Keep the best geometry observed during the terminal hover and report that
    // at lock — the hover is verified over ~180 steps, so should the evidence be.
    let mut fid_best: Option<swarm_core::vision::FiducialResult> = None;

    let mut primary_cleared = is_primary;
    let mut hold_xy: Option<[f64; 2]> = None;
    let mut vacate_xy: Option<[f64; 2]> = None;
    let mut last_lock_bcast = -1.0;
    let mut max_gps_wander_m: f64 = 0.0;
    let mut sensed_log_t = 0.0;
    let mut step_count = 0usize;

    let decode_latency_s = if is_primary { 0.0 } else { 0.120 };

    while robot.step(ts) {
        let t_start = Instant::now();
        step_count += 1;
        let now = robot.time();
        let true_pos = gps.values();
        let vel = gps.speed_vector();
        let rpy = imu.roll_pitch_yaw();
        let gyro_v = gyro.values();
        let speed = vel[0].hypot(vel[1]);

        let (cur_x, cur_y, cur_z, cur_wander) = if state == "STAGE_1_MACRO_NAV" || state == "STAGE_2_VISUAL_HOMING" {
            let w = gnss_walk.step(dt);
            max_gps_wander_m = max_gps_wander_m.max(w[0].hypot(w[1]));
            (true_pos[0] + w[0], true_pos[1] + w[1], true_pos[2], w)
        } else {
            (true_pos[0], true_pos[1], true_pos[2], [0.0, 0.0])
        };

        // Propagate camera-relative target vector using relative velocity (v_drone - v_target)
        if state == "STAGE_2_VISUAL_HOMING" {
            let tvel = handoff_data.as_ref().map(|p| p.target_vel).unwrap_or([0.0, 0.0, 0.0]);
            if let Some(ref mut rel) = rel_target_xy {
                rel[0] -= (vel[0] - tvel[0]) * dt;
                rel[1] -= (vel[1] - tvel[1]) * dt;
            }
        }

        // Poll receiver queue
        while let Some(raw_bytes) = receiver.pop_bytes() {
            if let Ok(v) = serde_json::from_slice::<serde_json::Value>(&raw_bytes) {
                if v.get("msg").and_then(|m| m.as_str()) == Some("follower_lock") {
                    if v.get("drone_id").and_then(|d| d.as_str()) != Some(&drone_id) && !primary_cleared {
                        primary_cleared = true;
                    }
                    continue;
                }
            }
            if let Ok(pkt) = HandoffPacket::deserialize(&raw_bytes) {
                if first_tx_seen.map_or(true, |t0| pkt.tx_time_s < t0) {
                    first_tx_seen = Some(pkt.tx_time_s);
                }
                if !pkt.crc_valid {
                    dropped_log.push(DropRecord { seq: pkt.seq, tx_time_s: pkt.tx_time_s, rx_time_s: now, reason: "crc_invalid".into() });
                    continue;
                }
                let drop = if is_primary { pkt.seq == 0 } else { pkt.seq == 0 || pkt.seq == 1 };
                if drop {
                    dropped_log.push(DropRecord { seq: pkt.seq, tx_time_s: pkt.tx_time_s, rx_time_s: now, reason: "link_drop_emulated".into() });
                    continue;
                }
                if handoff_data.is_none() {
                    let _ = pkt.image_chip.save(res_dir.join(format!("received_chip_{drone_id}.png")));
                    ref_kp = extract_orb_features(&pkt.image_chip, 600);
                    let bearing_deg = (pkt.target_gps[1] - true_pos[1]).atan2(pkt.target_gps[0] - true_pos[0]).to_degrees();
                    let ev = HandoffEvidence {
                        drone: drone_id.clone(),
                        scout_id: pkt.scout_id.clone(),
                        scout_gps: pkt.scout_gps,
                        target_gps: pkt.target_gps,
                        target_vel: pkt.target_vel,
                        approach_bearing_deg: bearing_deg,
                        distance_at_receipt_m: (pkt.target_gps[0] - true_pos[0]).hypot(pkt.target_gps[1] - true_pos[1]),
                        ref_orb_features: ref_kp.len(),
                        packet_bytes: raw_bytes.len(),
                        packet_seq: pkt.seq,
                        packet_tx_time_s: pkt.tx_time_s,
                        packet_crc_valid: pkt.crc_valid,
                        dropped_packets: dropped_log.clone(),
                        decode_latency_ms: decode_latency_s * 1000.0,
                        receipt_lag_from_mark_ms: (now - first_tx_seen.unwrap_or(now)) * 1000.0,
                        sim_time_s: now,
                    };
                    let _ = fs::write(res_dir.join(format!("handoff_{drone_id}.json")), serde_json::to_string_pretty(&ev).unwrap());
                    handoff_recv_time = Some(now);
                    stage1_engage_time = Some(now + decode_latency_s);
                    handoff_data = Some(pkt);
                }
            }
        }

        if state == "STAGE_1_MACRO_NAV" && !primary_cleared {
            if let Some(t_rx) = handoff_recv_time {
                if now - t_rx > TEAMMATE_FALLBACK_S {
                    primary_cleared = true;
                }
            }
        }

        if state == "IDLE_HOVER" {
            if let Some(t_eng) = stage1_engage_time {
                if now >= t_eng {
                    state = "STAGE_1_MACRO_NAV";
                }
            }
        }

        let readings: Vec<(f64, f64)> = rf_devs.iter().zip(rf_angles).map(|(d, a)| (d.value(), a)).collect();
        nav.update_clusters(now, cur_x, cur_y, cur_z, rpy[0], rpy[1], rpy[2], &readings);

        if now - sensed_log_t >= 2.0 {
            sensed_log_t = now;
            let slog = SensorAvoidanceLog {
                drone_id: drone_id.clone(),
                use_json_map_in_controller: USE_JSON_MAP_IN_CONTROLLER,
                sensor_hits_count: nav.sensor_hits_count,
                sensed_obstacle_centroids: nav.cluster_centroids(),
                active_clusters: nav.cluster_centroids().len(),
                sim_time_s: now,
            };
            let _ = fs::write(
                res_dir.join(format!("sensor_avoidance_{drone_id}.json")),
                serde_json::to_string_pretty(&slog).unwrap(),
            );
            let _ = fs::write(
                res_dir.join(format!("rust_perf_{drone_id}.json")),
                serde_json::to_string_pretty(&timer.summarize(&drone_id)).unwrap(),
            );
        }

        let (dx_world, dy_world, goal_dist, target_alt, yaw_cmd) = match state {
            "IDLE_HOVER" => {
                let eff = nav.steer_xy(cur_x, cur_y, home_xy, dt);
                (eff[0] - cur_x, eff[1] - cur_y, (home_xy[0] - cur_x).hypot(home_xy[1] - cur_y), cruise_alt, 0.0)
            }
            "STAGE_1_MACRO_NAV" => {
                let pkt = handoff_data.as_ref().unwrap();
                let elapsed_tx = (now - pkt.tx_time_s).max(0.0);
                // Predict moving target position using handed-off velocity vector
                let tgt_xy = [
                    pkt.target_gps[0] + pkt.target_vel[0] * elapsed_tx,
                    pkt.target_gps[1] + pkt.target_vel[1] * elapsed_tx,
                ];
                let dx = tgt_xy[0] - cur_x;
                let dy = tgt_xy[1] - cur_y;
                let dist = dx.hypot(dy);
                if dist > 1.5 {
                    approach_yaw = dy.atan2(dx);
                }

                if step_count % 4 == 0 && dist > 4.0 && !visual_id_logged {
                    let yaw_err = (approach_yaw - rpy[2]).sin().atan2((approach_yaw - rpy[2]).cos());
                    if yaw_err.abs() < 0.25 {
                        let bgra = front_cam.bgra_slice();
                        if !bgra.is_empty() {
                            let rgb = bgra_to_rgb(bgra, front_cam.width, front_cam.height);
                            let res = match_visual_id(&ref_kp, &rgb);
                            if res.confirmed {
                                visual_id_logged = true;
                                let _ = rgb.save(res_dir.join(format!("id_frame_{drone_id}.png")));
                                let ev = VisualIdEvidence {
                                    drone: drone_id.clone(),
                                    gps: true_pos,
                                    orb_good_matches: res.good_matches,
                                    ransac_inliers: res.inliers,
                                    red_target_area_px: res.red_area_px,
                                    sim_time_s: now,
                                };
                                let _ = fs::write(res_dir.join(format!("visual_id_{drone_id}.json")), serde_json::to_string_pretty(&ev).unwrap());
                            }
                        }
                    }
                }

                let nav_goal = if !primary_cleared {
                    // Bug #45 follow-up: with the Phase-14 moving target, the old
                    // ring-distance gate released the hold whenever the target
                    // drifted past 7 m, producing a hold/chase limit cycle around
                    // the ring boundary. The secondary now holds its frozen spot
                    // until the primary announces its lock (or the 115 s fallback).
                    if hold_xy.is_none() {
                        let candidate = if nav.park_spot_field_free([true_pos[0], true_pos[1]]) {
                            [true_pos[0], true_pos[1]]
                        } else {
                            let dxh = true_pos[0] - tgt_xy[0];
                            let dyh = true_pos[1] - tgt_xy[1];
                            let dn = dxh.hypot(dyh).max(1.0);
                            [tgt_xy[0] + dxh / dn * HOLD_RING_M, tgt_xy[1] + dyh / dn * HOLD_RING_M]
                        };
                        hold_xy = Some(nav.push_spot_out_of_clusters(candidate));
                    }
                    hold_xy.unwrap()
                } else {
                    hold_xy = None;
                    if dist < 4.5 {
                        state = "STAGE_2_VISUAL_HOMING";
                        rel_target_xy = Some([tgt_xy[0] - cur_x, tgt_xy[1] - cur_y]);
                    }
                    tgt_xy
                };

                let eff = nav.steer_xy(cur_x, cur_y, nav_goal, dt);
                (eff[0] - cur_x, eff[1] - cur_y, (nav_goal[0] - cur_x).hypot(nav_goal[1] - cur_y), cruise_alt, approach_yaw)
            }
            "STAGE_2_VISUAL_HOMING" => {
                let pkt = handoff_data.as_ref().unwrap();
                let plate_z = pkt.target_gps[2];
                let rx0 = rel_target_xy.unwrap()[0];
                let ry0 = rel_target_xy.unwrap()[1];
                let vdist0 = rx0.hypot(ry0);
                if vdist0 > 1.5 {
                    approach_yaw = ry0.atan2(rx0);
                }

                let term_bgra = term_cam.bgra_slice();
                let mut seen = false;
                if !term_bgra.is_empty() {
                    let term_rgb = bgra_to_rgb(term_bgra, term_cam.width, term_cam.height);
                    if !visual_id_logged && vdist0 > 2.5 {
                        let id_res = match_visual_id(&ref_kp, &term_rgb);
                        if id_res.confirmed {
                            visual_id_logged = true;
                            let _ = term_rgb.save(res_dir.join(format!("id_frame_{drone_id}.png")));
                            let ev = VisualIdEvidence {
                                drone: drone_id.clone(),
                                gps: true_pos,
                                orb_good_matches: id_res.good_matches,
                                ransac_inliers: id_res.inliers,
                                red_target_area_px: id_res.red_area_px,
                                sim_time_s: now,
                            };
                            let _ = fs::write(res_dir.join(format!("visual_id_{drone_id}.json")), serde_json::to_string_pretty(&ev).unwrap());
                        }
                    }

                    let f_px = (term_cam.width as f64 / 2.0) / (FOV / 2.0).tan();
                    let (ct, st) = (TERMINAL_CAM_PITCH.cos(), TERMINAL_CAM_PITCH.sin());
                    let r_mat = rot_matrix(rpy[0], rpy[1], rpy[2]);
                    let cam_off_w = mat_vec(&r_mat, [0.12, 0.0, 0.02]);
                    let pcam_z = cur_z + cam_off_w[2];

                    let invert_rel = |px: f64, py: f64| -> Option<[f64; 2]> {
                        if (cur_z + 0.02) - plate_z <= 0.5 {
                            return None;
                        }
                        let uu = (px - term_cam.width as f64 / 2.0) / f_px;
                        let vv = (py - term_cam.height as f64 / 2.0) / f_px;
                        let db = [ct - vv * st, -uu, -st - vv * ct];
                        let dw = mat_vec(&r_mat, db);
                        if dw[2] >= -0.05 {
                            return None;
                        }
                        let s = (plate_z - pcam_z) / dw[2];
                        if s <= 0.0 || s > 60.0 {
                            return None;
                        }
                        let rel = [cam_off_w[0] + s * dw[0], cam_off_w[1] + s * dw[1]];
                        if rel[0].hypot(rel[1]) > 25.0 {
                            return None;
                        }
                        Some(rel)
                    };

                    // Consensus-gated fiducial precision aid (3-frame streak + 0.10 m outlier check)
                    if vdist0 < 1.2 && step_count % 5 == 0 {
                        let f_res = verify_fiducial(&term_rgb, invert_rel);
                        // Bug #50: remember the best geometry seen during the hover —
                        // prefer a fully-verified read, else any read with geometry.
                        if f_res.verified
                            || (f_res.target_yaw_deg.is_some()
                                && fid_best.as_ref().map_or(true, |b| !b.verified))
                        {
                            fid_best = Some(f_res.clone());
                        }
                        if f_res.verified {
                            if let Some(f_mid) = f_res.midpoint_rel {
                                let cur_rel = rel_target_xy.unwrap();
                                if (f_mid[0] - cur_rel[0]).hypot(f_mid[1] - cur_rel[1]) < 0.10 {
                                    fiducial_streak += 1;
                                    if fiducial_streak >= 3 {
                                        let a = 0.15;
                                        rel_target_xy = Some([
                                            a * f_mid[0] + (1.0 - a) * cur_rel[0],
                                            a * f_mid[1] + (1.0 - a) * cur_rel[1],
                                        ]);
                                        homing_update_count += 1;
                                    }
                                } else {
                                    fiducial_streak = 0;
                                }
                            }
                        } else {
                            fiducial_streak = 0;
                        }
                    }

                    let blobs = detect_red_blobs(&term_rgb, 800.0);
                    seen = !blobs.is_empty();
                    if seen {
                        last_red_seen = now;
                        let cur_rel = rel_target_xy.unwrap();
                        let mut best_cand: Option<(f64, [f64; 2])> = None;

                        for b in &blobs {
                            let (bx0, by0, bw, bh) = b.bbox;
                            if bx0 <= 3 || by0 <= 3 || bx0 + bw >= term_cam.width - 4 || by0 + bh >= term_cam.height - 4 {
                                continue;
                            }
                            let mut pts = [[0.0; 2]; 4];
                            let mut ok_pts = 0;
                            for (k, &(cx, cy)) in b.corners.iter().enumerate() {
                                if let Some(p) = invert_rel(cx, cy) {
                                    pts[k] = p;
                                    ok_pts += 1;
                                }
                            }
                            if ok_pts < 4 {
                                continue;
                            }
                            let est = [
                                (pts[0][0] + pts[1][0] + pts[2][0] + pts[3][0]) * 0.25,
                                (pts[0][1] + pts[1][1] + pts[2][1] + pts[3][1]) * 0.25,
                            ];
                            let mean_side: f64 = (0..4)
                                .map(|i| (pts[i][0] - pts[(i + 1) % 4][0]).hypot(pts[i][1] - pts[(i + 1) % 4][1]))
                                .sum::<f64>()
                                * 0.25;
                            if !(0.8..=2.2).contains(&mean_side) {
                                continue;
                            }
                            let slant = est[0].hypot(est[1]).hypot((cur_z + 0.02) - plate_z);
                            let expected_area = (1.4 * f_px / slant).powi(2);
                            if b.area < 0.5 * expected_area {
                                continue;
                            }
                            let d_rel = (est[0] - cur_rel[0]).hypot(est[1] - cur_rel[1]);
                            if best_cand.map_or(true, |(bd, _)| d_rel < bd) {
                                best_cand = Some((d_rel, est));
                            }
                        }

                        let accept_r = if homing_update_count < 150 { HOMING_ACCEPT_RADIUS_M } else { 0.12 };
                        if let Some((d_rel, est)) = best_cand {
                            if d_rel < accept_r {
                                let a = 0.45;
                                rel_target_xy = Some([
                                    a * est[0] + (1.0 - a) * cur_rel[0],
                                    a * est[1] + (1.0 - a) * cur_rel[1],
                                ]);
                                homing_update_count += 1;
                                stall_reject_streak = 0;
                            } else if speed < 0.18 {
                                stall_reject_streak += 1;
                                if stall_reject_streak >= 200 {
                                    let a = 0.45;
                                    rel_target_xy = Some([
                                        a * est[0] + (1.0 - a) * cur_rel[0],
                                        a * est[1] + (1.0 - a) * cur_rel[1],
                                    ]);
                                    homing_update_count += 1;
                                    stall_reject_streak = 0;
                                }
                            } else {
                                stall_reject_streak = 0;
                            }
                        }
                    }

                    let [rx, ry] = rel_target_xy.unwrap();
                    let vdist = rx.hypot(ry);
                    let red_fresh = seen || (now - last_red_seen) < 0.3;
                    // Relative ground speed error (drone velocity minus target velocity)
                    let rel_speed = (vel[0] - pkt.target_vel[0]).hypot(vel[1] - pkt.target_vel[1]);
                    if vdist < 0.045 && rel_speed < 0.03 && red_fresh {
                        lock_streak += 1;
                    } else {
                        lock_streak = 0;
                    }

                    if lock_streak >= 180 && homing_update_count > 0 && !lock_achieved_logged {
                        lock_achieved_logged = true;
                        // Compare drone's true position against the plate's true world position (true_pos + rel_camera_measured)
                        // Bug #50: report the best fiducial geometry seen during the
                        // hover; only re-probe if the whole hover saw nothing.
                        let fid = match fid_best {
                            Some(ref f) if f.verified || f.target_yaw_deg.is_some() => f.clone(),
                            _ => verify_fiducial(&term_rgb, invert_rel),
                        };
                        let plate_true_est = if let Some(mid) = fid.midpoint_rel {
                            [true_pos[0] + mid[0], true_pos[1] + mid[1]]
                        } else {
                            [true_pos[0] + rx, true_pos[1] + ry]
                        };
                        let true_err = rx.hypot(ry);
                        let _ = term_rgb.save(res_dir.join(format!("lock_frame_{drone_id}.png")));
                        let ev = LockEvidence {
                            drone: drone_id.clone(),
                            gps: true_pos,
                            perceived_gps_at_lock: [cur_x, cur_y, cur_z],
                            noisy_target_gps: pkt.target_gps,
                            refined_target_xy: plate_true_est,
                            rel_target_xy: [rx, ry],
                            stage2_triggered: true,
                            homing_update_count,
                            last_red_seen_sim_time: last_red_seen,
                            gnss_model: "ornstein_uhlenbeck".into(),
                            max_gps_wander_m,
                            gps_wander_at_lock_xy: cur_wander,
                            handoff_packet_seq: pkt.seq,
                            handoff_crc_valid: pkt.crc_valid,
                            fiducial_verified: fid.verified,
                            fiducial_target_yaw_deg: fid.target_yaw_deg,
                            fiducial_marker_separation_m: fid.marker_separation_m,
                            terminal_error_m: true_err,
                            visual_id_confirmed: visual_id_logged,
                            sim_time_s: now,
                            time_handoff_to_lock_s: now - handoff_recv_time.unwrap_or(now),
                        };
                        let _ = fs::write(res_dir.join(format!("lock_{drone_id}.json")), serde_json::to_string_pretty(&ev).unwrap());
                        let msg = format!(r#"{{"msg":"follower_lock","drone_id":"{drone_id}"}}"#);
                        emitter.send(msg.as_bytes());
                        last_lock_bcast = now;
                        state = "STAGE_3_VACATE";
                    }
                }

                let [rx, ry] = rel_target_xy.unwrap();
                let vdist = rx.hypot(ry);
                if vdist < 0.8 {
                    (rx, ry, vdist, cruise_alt, approach_yaw)
                } else {
                    let eff = nav.steer_xy(cur_x, cur_y, [cur_x + rx, cur_y + ry], dt);
                    (eff[0] - cur_x, eff[1] - cur_y, vdist, cruise_alt, approach_yaw)
                }
            }
            _ => {
                if vacate_xy.is_none() {
                    let pkt = handoff_data.as_ref().unwrap();
                    let elapsed_tx = (now - pkt.tx_time_s).max(0.0);
                    let cur_tgt = [
                        pkt.target_gps[0] + pkt.target_vel[0] * elapsed_tx + VACATE_OFFSET_XY[0],
                        pkt.target_gps[1] + pkt.target_vel[1] * elapsed_tx + VACATE_OFFSET_XY[1],
                    ];
                    vacate_xy = Some(nav.push_spot_out_of_clusters(cur_tgt));
                }
                if now - last_lock_bcast >= 1.0 {
                    let msg = format!(r#"{{"msg":"follower_lock","drone_id":"{drone_id}"}}"#);
                    emitter.send(msg.as_bytes());
                    last_lock_bcast = now;
                }
                let vx = vacate_xy.unwrap();
                let eff = nav.steer_xy(cur_x, cur_y, vx, dt);
                (eff[0] - cur_x, eff[1] - cur_y, (vx[0] - cur_x).hypot(vx[1] - cur_y), cruise_alt, 0.0)
            }
        };

        // Bug #45: the terminal boost exists for Stage-2 precision only. A sustained
        // Stage-1 hold chases the OU-wandered setpoint (up to 0.35 m) with saturated
        // commands at full boost and limit-cycled the attitude loop — the drone
        // tumbled from a stable hover at t ~ 151 s in two consecutive runs. Stage 1
        // / IDLE / VACATE fly at plain cruise gain; Stage 2 gets the full 6.5x boost
        // exactly when terminal precision is needed (its own gates only ramp the
        // boost inside the last 2 m anyway).
        pid.max_terminal_boost = if state == "STAGE_2_VISUAL_HOMING" { 6.5 } else { 0.0 };

        // Feed-forward target velocity into the relative velocity damping in Stage 2
        let rel_vel = if state == "STAGE_2_VISUAL_HOMING" {
            let tvel = handoff_data.as_ref().map(|p| p.target_vel).unwrap_or([0.0, 0.0, 0.0]);
            [vel[0] - tvel[0], vel[1] - tvel[1], vel[2]]
        } else {
            vel
        };

        let vels = pid.step_motors(
            dt,
            [cur_x, cur_y, cur_z],
            rel_vel,
            rpy,
            gyro_v,
            dx_world,
            dy_world,
            goal_dist,
            target_alt,
            yaw_cmd,
        );
        for (m, v) in motors.iter().zip(vels) {
            m.set_velocity(v);
        }
        timer.record_us(t_start.elapsed().as_secs_f64() * 1_000_000.0);
    }
}
