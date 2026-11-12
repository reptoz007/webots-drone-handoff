use rand::{rngs::StdRng, Rng, SeedableRng};
use serde::Serialize;
use std::collections::HashMap;
use std::env;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use webots_ffi::{Robot, Supervisor};

const WIND_SCALE: f64 = 0.04;
const STEADY_WIND_N: [f64; 3] = [0.35 * WIND_SCALE, -0.25 * WIND_SCALE, 0.0];
const GUST_AMP_XY_N: f64 = 0.25 * WIND_SCALE;
const GUST_AMP_Z_N: f64 = 0.10 * WIND_SCALE;
const TARGET_VEL: [f64; 3] = [0.09, 0.06, 0.0];
const TIMEOUT_S: f64 = 240.0;

#[derive(Serialize)]
struct WindModelMeta {
    steady_crosswind_n: [f64; 3],
    gust_amp_xy_n: f64,
    gust_amp_z_n: f64,
    applied_while_z_gt_m: f64,
    wind_scale_vs_spec: f64,
    mc_seed: u64,
}

#[derive(Serialize)]
struct GroundTruth {
    outcome: String,
    sim_time_s: f64,
    initial_target_xyz: [f64; 3],
    true_target_xyz: [f64; 3],
    target_displacement_m: f64,
    lock_true_targets: HashMap<String, [f64; 3]>,
    final_positions: HashMap<String, [f64; 3]>,
    wind_gust_active: bool,
    wind_model: WindModelMeta,
}

fn results_dir() -> PathBuf {
    let p = PathBuf::from("../../results");
    let _ = fs::create_dir_all(&p);
    p
}

fn main() {
    let robot = Robot::new();
    let ts = robot.basic_time_step();
    let dt = ts as f64 / 1000.0;
    let res_dir = results_dir();

    let mc_seed: u64 = env::var("WEBOTS_MC_SEED").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
    let mut rng = StdRng::seed_from_u64(mc_seed.wrapping_mul(7919).wrapping_add(13));
    let wind_phase = [
        rng.gen_range(0.0..6.283185),
        rng.gen_range(0.0..6.283185),
        rng.gen_range(0.0..6.283185),
    ];

    let defs = ["SCOUT", "FOLLOWER_1", "FOLLOWER_2"];
    let nodes: Vec<(&str, webots_ffi::WbNodeRef)> = defs
        .iter()
        .filter_map(|&d| Supervisor::from_def(d).map(|n| (d, n)))
        .collect();

    let target_node = Supervisor::from_def("TARGET");
    let target_trans_field = target_node.and_then(|n| Supervisor::node_field(n, "translation"));
    let initial_target_xyz = target_trans_field
        .map(Supervisor::field_get_vec3f)
        .unwrap_or([6.0, 4.0, 0.05]);
    let mut cur_target_xyz = initial_target_xyz;

    let mut tracks = BufWriter::new(File::create(res_dir.join("tracks.csv")).unwrap());
    let _ = writeln!(tracks, "t,drone,x,y,z");
    let mut wind_log = BufWriter::new(File::create(res_dir.join("wind_log.csv")).unwrap());
    let _ = writeln!(wind_log, "t,fx_n,fy_n,fz_n,applied_scout,applied_f1,applied_f2");

    let mut dumped = false;
    let mut done_since: Option<f64> = None;
    let mut wind_gust_active = false;
    let mut wind_log_t = 0.0;
    let mut target_moving = false;
    let mut lock_true_targets: HashMap<String, [f64; 3]> = HashMap::new();

    let finalize = |outcome: &str,
                    sim_t: f64,
                    active: bool,
                    cur_tgt: [f64; 3],
                    lock_tgts: &HashMap<String, [f64; 3]>,
                    tracks_f: &mut BufWriter<File>,
                    wind_f: &mut BufWriter<File>| {
        let _ = tracks_f.flush();
        let _ = wind_f.flush();
        let mut final_positions = HashMap::new();
        for &(k, n) in &nodes {
            final_positions.insert(k.to_string(), Supervisor::node_position(n));
        }
        let disp = (cur_tgt[0] - initial_target_xyz[0]).hypot(cur_tgt[1] - initial_target_xyz[1]);
        let gt = GroundTruth {
            outcome: outcome.to_string(),
            sim_time_s: sim_t,
            initial_target_xyz,
            true_target_xyz: cur_tgt,
            target_displacement_m: disp,
            lock_true_targets: lock_tgts.clone(),
            final_positions,
            wind_gust_active: active,
            wind_model: WindModelMeta {
                steady_crosswind_n: STEADY_WIND_N,
                gust_amp_xy_n: GUST_AMP_XY_N,
                gust_amp_z_n: GUST_AMP_Z_N,
                applied_while_z_gt_m: 1.0,
                wind_scale_vs_spec: WIND_SCALE,
                mc_seed,
            },
        };
        let _ = fs::write(res_dir.join("ground_truth.json"), serde_json::to_string_pretty(&gt).unwrap());
    };

    while robot.step(ts) {
        let t = robot.time();
        if !dumped && t > 0.5 {
            let lines = Supervisor::dump_root_scene();
            let _ = fs::write(res_dir.join("scene_dump.txt"), lines.join("\n"));
            dumped = true;
        }

        // Start moving Ground_Target smoothly once the Scout has broadcast its mark
        if !target_moving && res_dir.join("mark.json").exists() {
            target_moving = true;
        }
        if target_moving {
            if let Some(tf) = target_trans_field {
                cur_target_xyz[0] += TARGET_VEL[0] * dt;
                cur_target_xyz[1] += TARGET_VEL[1] * dt;
                Supervisor::field_set_vec3f(tf, cur_target_xyz);
            }
        }

        let fx = STEADY_WIND_N[0]
            + GUST_AMP_XY_N * (0.90 * t + wind_phase[0]).sin()
            + 0.08 * WIND_SCALE * (2.1 * t + wind_phase[1]).sin();
        let fy = STEADY_WIND_N[1]
            + GUST_AMP_XY_N * (1.30 * t + wind_phase[1]).sin()
            + 0.08 * WIND_SCALE * (1.7 * t + wind_phase[2]).cos();
        let fz = STEADY_WIND_N[2] + GUST_AMP_Z_N * (0.60 * t + wind_phase[2]).sin();

        let mut app = [0u8; 3];
        for (idx, &(k, n)) in nodes.iter().enumerate() {
            let p = Supervisor::node_position(n);
            let _ = writeln!(tracks, "{t:.3},{k},{:.4},{:.4},{:.4}", p[0], p[1], p[2]);
            if WIND_SCALE > 0.0 && p[2] > 1.0 {
                Supervisor::add_force(n, [fx, fy, fz], false);
                wind_gust_active = true;
                app[idx] = 1;
            }
            if t > 20.0 && p[2] < 0.15 {
                finalize(&format!("crash:{k}"), t, wind_gust_active, cur_target_xyz, &lock_true_targets, &mut tracks, &mut wind_log);
                Supervisor::quit(1);
                return;
            }
        }

        if t - wind_log_t >= 2.0 {
            wind_log_t = t;
            let _ = writeln!(wind_log, "{t:.2},{fx:.3},{fy:.3},{fz:.3},{},{},{}", app[0], app[1], app[2]);
        }

        for fid in ["Follower_Drone_01", "Follower_Drone_02"] {
            if !lock_true_targets.contains_key(fid) && res_dir.join(format!("lock_{fid}.json")).exists() {
                lock_true_targets.insert(fid.to_string(), cur_target_xyz);
            }
        }

        if lock_true_targets.len() == 2 {
            let start = *done_since.get_or_insert(t);
            if t - start > 2.0 {
                finalize("mission_complete", t, wind_gust_active, cur_target_xyz, &lock_true_targets, &mut tracks, &mut wind_log);
                Supervisor::quit(0);
                return;
            }
        }

        if t > TIMEOUT_S {
            finalize("timeout", t, wind_gust_active, cur_target_xyz, &lock_true_targets, &mut tracks, &mut wind_log);
            Supervisor::quit(2);
            return;
        }
    }
}
