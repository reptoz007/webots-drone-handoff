use rand::{rngs::StdRng, Rng, SeedableRng};
use serde::Serialize;
use std::collections::HashMap;
use std::f64::consts::PI;

pub const USE_JSON_MAP_IN_CONTROLLER: bool = false;
pub const RF_MAX_RANGE_M: f64 = 11.5;
pub const RF_MIN_RANGE_M: f64 = 0.8;
pub const CLUSTER_EPS_M: f64 = 2.2;
pub const CLUSTER_MIN_HITS: usize = 5;
pub const CLUSTER_TTL_S: f64 = 6.0;
pub const HULL_INFLATE_M: f64 = 0.3;
pub const MERGE_GAP_M: f64 = 3.0;
pub const PARK_CLEAR_M: f64 = 5.7;

pub const STEER_LEAD_M: f64 = 1.6;
pub const STEER_CLEAR_M: f64 = 2.5;
pub const STEER_STEP_RAD: f64 = 15.0 * PI / 180.0;
pub const STEER_TERMINAL_M: f64 = 3.5;
pub const STEER_MAX_RATE_RAD_S: f64 = 2.0;

#[inline(always)]
pub fn clamp(val: f64, lo: f64, hi: f64) -> f64 {
    val.max(lo).min(hi)
}

#[derive(Debug, Clone, Serialize)]
pub struct SensorAvoidanceLog {
    pub drone_id: String,
    pub use_json_map_in_controller: bool,
    pub sensor_hits_count: usize,
    pub sensed_obstacle_centroids: Vec<[f64; 2]>,
    pub active_clusters: usize,
    pub sim_time_s: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct StepLatencyStats {
    pub drone_id: String,
    pub total_steps: usize,
    pub mean_step_us: f64,
    pub p99_step_us: f64,
    pub max_step_us: f64,
}

pub struct StepTimer {
    samples_us: Vec<f64>,
}

impl StepTimer {
    pub fn new() -> Self {
        Self { samples_us: Vec::with_capacity(32_000) }
    }
    pub fn record_us(&mut self, us: f64) {
        self.samples_us.push(us);
    }
    pub fn summarize(&self, drone_id: &str) -> StepLatencyStats {
        if self.samples_us.is_empty() {
            return StepLatencyStats {
                drone_id: drone_id.to_string(),
                total_steps: 0,
                mean_step_us: 0.0,
                p99_step_us: 0.0,
                max_step_us: 0.0,
            };
        }
        let mut sorted = self.samples_us.clone();
        sorted.sort_by(|a, b| a.total_cmp(b));
        let sum: f64 = sorted.iter().sum();
        let p99_idx = ((sorted.len() as f64 * 0.99) as usize).min(sorted.len() - 1);
        StepLatencyStats {
            drone_id: drone_id.to_string(),
            total_steps: sorted.len(),
            mean_step_us: sum / sorted.len() as f64,
            p99_step_us: sorted[p99_idx],
            max_step_us: *sorted.last().unwrap(),
        }
    }
}

/// Phase 13: Stochastic Ornstein-Uhlenbeck (mean-reverting random walk) GNSS error model.
pub struct GnssRandomWalk {
    rng: StdRng,
    pub wx: f64,
    pub wy: f64,
    tau_s: f64,
    sigma_m: f64,
}

impl GnssRandomWalk {
    pub fn new(seed: u64, sigma_m: f64, tau_s: f64) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed),
            wx: 0.0,
            wy: 0.0,
            tau_s,
            sigma_m,
        }
    }

    fn randn(&mut self) -> f64 {
        let u1: f64 = self.rng.gen_range(1e-9..1.0);
        let u2: f64 = self.rng.gen_range(0.0..6.283185307179586);
        (-2.0 * u1.ln()).sqrt() * u2.cos()
    }

    pub fn step(&mut self, dt: f64) -> [f64; 2] {
        let decay = (-dt / self.tau_s).exp();
        let diff = self.sigma_m * (1.0 - decay * decay).sqrt();
        self.wx = decay * self.wx + diff * self.randn();
        self.wy = decay * self.wy + diff * self.randn();
        let jitter_x = 0.02 * self.randn();
        let jitter_y = 0.02 * self.randn();
        [self.wx + jitter_x, self.wy + jitter_y]
    }
}

pub struct LidarVfhNavigator {
    pub clusters: HashMap<usize, Vec<(f64, f64, f64)>>,
    pub next_cluster_id: usize,
    pub sensor_hits_count: usize,
    pub steer_ang: Option<f64>,
    pub last_rep: [f64; 2],
}

impl LidarVfhNavigator {
    pub fn new() -> Self {
        Self {
            clusters: HashMap::new(),
            next_cluster_id: 0,
            sensor_hits_count: 0,
            steer_ang: None,
            last_rep: [0.0, 0.0],
        }
    }

    pub fn update_clusters(
        &mut self,
        now: f64,
        x: f64,
        y: f64,
        z: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
        readings: &[(f64, f64)],
    ) {
        if z > 1.5 && roll.abs() < 0.25 && pitch.abs() < 0.25 {
            for &(r, rel) in readings {
                if (RF_MIN_RANGE_M..RF_MAX_RANGE_M).contains(&r) {
                    let beam_yaw = yaw + rel;
                    let hx = x + r * beam_yaw.cos();
                    let hy = y + r * beam_yaw.sin();
                    self.sensor_hits_count += 1;

                    let mut best_id = None;
                    let mut best_d = CLUSTER_EPS_M;
                    for (&cid, pts) in &self.clusters {
                        let n = pts.len() as f64;
                        let ccx = pts.iter().map(|p| p.0).sum::<f64>() / n;
                        let ccy = pts.iter().map(|p| p.1).sum::<f64>() / n;
                        let d = (hx - ccx).hypot(hy - ccy);
                        if d < best_d {
                            best_d = d;
                            best_id = Some(cid);
                        }
                    }
                    let cid = best_id.unwrap_or_else(|| {
                        let id = self.next_cluster_id;
                        self.next_cluster_id += 1;
                        self.clusters.insert(id, Vec::new());
                        id
                    });
                    self.clusters.get_mut(&cid).unwrap().push((hx, hy, now));
                }
            }
        }

        self.clusters.retain(|_, pts| {
            pts.retain(|p| now - p.2 <= CLUSTER_TTL_S);
            if pts.is_empty() {
                return false;
            }
            if pts.len() < CLUSTER_MIN_HITS && now - pts.last().unwrap().2 > 0.5 {
                return false;
            }
            true
        });

        loop {
            let mut merged = false;
            let ids: Vec<usize> = self.clusters.keys().copied().collect();
            'outer: for i in 0..ids.len() {
                for j in (i + 1)..ids.len() {
                    let (a, b) = (ids[i], ids[j]);
                    if let (Some(pa), Some(pb)) = (self.clusters.get(&a), self.clusters.get(&b)) {
                        if Self::hull_gap(pa, pb) < MERGE_GAP_M {
                            let pb_owned = self.clusters.remove(&b).unwrap();
                            self.clusters.get_mut(&a).unwrap().extend(pb_owned);
                            merged = true;
                            break 'outer;
                        }
                    }
                }
            }
            if !merged {
                break;
            }
        }
    }

    fn hull_gap(pa: &[(f64, f64, f64)], pb: &[(f64, f64, f64)]) -> f64 {
        let (mut ax0, mut ax1, mut ay0, mut ay1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &(x, y, _) in pa {
            ax0 = ax0.min(x); ax1 = ax1.max(x);
            ay0 = ay0.min(y); ay1 = ay1.max(y);
        }
        let (mut bx0, mut bx1, mut by0, mut by1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &(x, y, _) in pb {
            bx0 = bx0.min(x); bx1 = bx1.max(x);
            by0 = by0.min(y); by1 = by1.max(y);
        }
        let gx = (bx0 - (ax1 + HULL_INFLATE_M)).max(ax0 - (bx1 + HULL_INFLATE_M)).max(0.0);
        let gy = (by0 - (ay1 + HULL_INFLATE_M)).max(ay0 - (by1 + HULL_INFLATE_M)).max(0.0);
        gx.hypot(gy)
    }

    pub fn cluster_aabbs(&self) -> Vec<(f64, f64, f64, f64, f64, f64)> {
        let mut boxes = Vec::new();
        for pts in self.clusters.values() {
            if pts.len() < CLUSTER_MIN_HITS {
                continue;
            }
            let (mut x0, mut x1, mut y0, mut y1, mut sx, mut sy) =
                (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY, 0.0, 0.0);
            for &(x, y, _) in pts {
                x0 = x0.min(x); x1 = x1.max(x);
                y0 = y0.min(y); y1 = y1.max(y);
                sx += x; sy += y;
            }
            let n = pts.len() as f64;
            boxes.push((
                x0 - HULL_INFLATE_M,
                y0 - HULL_INFLATE_M,
                x1 + HULL_INFLATE_M,
                y1 + HULL_INFLATE_M,
                sx / n,
                sy / n,
            ));
        }
        boxes
    }

    pub fn cluster_centroids(&self) -> Vec<[f64; 2]> {
        self.cluster_aabbs()
            .into_iter()
            .map(|(_, _, _, _, cx, cy)| [(cx * 100.0).round() / 100.0, (cy * 100.0).round() / 100.0])
            .collect()
    }

    pub fn park_spot_field_free(&self, spot: [f64; 2]) -> bool {
        for (minx, miny, maxx, maxy, _, _) in self.cluster_aabbs() {
            let cx = clamp(spot[0], minx, maxx);
            let cy = clamp(spot[1], miny, maxy);
            if (spot[0] - cx).hypot(spot[1] - cy) < PARK_CLEAR_M {
                return false;
            }
        }
        true
    }

    pub fn push_spot_out_of_clusters(&self, spot: [f64; 2]) -> [f64; 2] {
        let mut s = spot;
        let boxes = self.cluster_aabbs();
        for _ in 0..6 {
            let mut moved = false;
            for &(minx, miny, maxx, maxy, ccx, ccy) in &boxes {
                let cx = clamp(s[0], minx, maxx);
                let cy = clamp(s[1], miny, maxy);
                let (mut dx, mut dy) = (s[0] - cx, s[1] - cy);
                let mut d = dx.hypot(dy);
                if d >= PARK_CLEAR_M {
                    continue;
                }
                if d < 1e-3 {
                    dx = s[0] - ccx;
                    dy = s[1] - ccy;
                    d = dx.hypot(dy).max(1.0);
                }
                s = [cx + dx / d * PARK_CLEAR_M, cy + dy / d * PARK_CLEAR_M];
                moved = true;
            }
            if !moved {
                break;
            }
        }
        s
    }

    fn probe_clearance(px: f64, py: f64, boxes: &[(f64, f64, f64, f64, f64, f64)]) -> f64 {
        let mut best = f64::INFINITY;
        for &(minx, miny, maxx, maxy, _, _) in boxes {
            let cx = clamp(px, minx, maxx);
            let cy = clamp(py, miny, maxy);
            best = best.min((px - cx).hypot(py - cy));
        }
        best
    }

    pub fn steer_xy(&mut self, x: f64, y: f64, goal_xy: [f64; 2], dt: f64) -> [f64; 2] {
        let (gx, gy) = (goal_xy[0], goal_xy[1]);
        let (dxg, dyg) = (gx - x, gy - y);
        let goal_dist = dxg.hypot(dyg);
        if goal_dist < 0.4 {
            return goal_xy;
        }
        let lead = STEER_LEAD_M.min(goal_dist);
        let base = dyg.atan2(dxg);
        let boxes = self.cluster_aabbs();
        let mut best_ang = base;

        if !boxes.is_empty() && goal_dist > STEER_TERMINAL_M {
            let clear_base = Self::probe_clearance(x + lead * base.cos(), y + lead * base.sin(), &boxes) >= STEER_CLEAR_M;
            if clear_base {
                best_ang = base;
            } else if let Some(sa) = self.steer_ang {
                if Self::probe_clearance(x + lead * sa.cos(), y + lead * sa.sin(), &boxes) >= STEER_CLEAR_M {
                    best_ang = sa;
                } else {
                    best_ang = Self::scan_best_angle(x, y, lead, base, &boxes);
                }
            } else {
                best_ang = Self::scan_best_angle(x, y, lead, base, &boxes);
            }
        }

        match self.steer_ang {
            None => self.steer_ang = Some(best_ang),
            Some(cur) => {
                let diff = (best_ang - cur).sin().atan2((best_ang - cur).cos());
                self.steer_ang = Some(cur + clamp(diff, -STEER_MAX_RATE_RAD_S * dt, STEER_MAX_RATE_RAD_S * dt));
            }
        }
        let sa = self.steer_ang.unwrap();
        let eff_x = x + lead * sa.cos();
        let eff_y = y + lead * sa.sin();
        self.last_rep = [eff_x - gx, eff_y - gy];
        [eff_x, eff_y]
    }

    fn scan_best_angle(x: f64, y: f64, lead: f64, base: f64, boxes: &[(f64, f64, f64, f64, f64, f64)]) -> f64 {
        let mut best_clear = -1.0;
        let mut best_ang = base;
        for k in 1..=12 {
            for sign in [1.0, -1.0] {
                let ang = base + sign * (k as f64) * STEER_STEP_RAD;
                let clear = Self::probe_clearance(x + lead * ang.cos(), y + lead * ang.sin(), boxes);
                if clear > best_clear {
                    best_clear = clear;
                    best_ang = ang;
                }
            }
        }
        best_ang
    }
}

pub struct MavicPidController {
    pub i_fwd: f64,
    pub i_left: f64,
    pub max_terminal_boost: f64,
}

impl MavicPidController {
    const K_VERTICAL_THRUST: f64 = 68.5;
    const K_VERTICAL_OFFSET: f64 = 0.6;
    const K_VERTICAL_P: f64 = 3.0;
    const K_ROLL_P: f64 = 50.0;
    const K_PITCH_P: f64 = 30.0;

    pub fn new(max_terminal_boost: f64) -> Self {
        Self { i_fwd: 0.0, i_left: 0.0, max_terminal_boost }
    }

    pub fn step_motors(
        &mut self,
        dt: f64,
        pos: [f64; 3],
        vel: [f64; 3],
        rpy: [f64; 3],
        gyro: [f64; 3],
        dx_world: f64,
        dy_world: f64,
        goal_dist: f64,
        target_alt: f64,
        target_yaw: f64,
    ) -> [f64; 4] {
        let (roll, pitch, yaw) = (rpy[0], rpy[1], rpy[2]);
        let (cos_y, sin_y) = (yaw.cos(), yaw.sin());

        let body_fwd_err = dx_world * cos_y + dy_world * sin_y;
        let body_left_err = -dx_world * sin_y + dy_world * cos_y;
        let body_fwd_vel = vel[0] * cos_y + vel[1] * sin_y;
        let body_left_vel = -vel[0] * sin_y + vel[1] * cos_y;

        let err_mag = body_fwd_err.hypot(body_left_err);
        if err_mag < 0.6 {
            self.i_fwd -= 0.4 * body_fwd_err * dt;
            self.i_left += 0.4 * body_left_err * dt;
        } else {
            self.i_fwd *= 0.995;
            self.i_left *= 0.995;
        }
        self.i_fwd = clamp(self.i_fwd, -0.6, 0.6);
        self.i_left = clamp(self.i_left, -0.6, 0.6);

        let hover_speed = vel[0].hypot(vel[1]);
        let gs = 1.0
            + self.max_terminal_boost
                * clamp((2.0 - goal_dist) / 2.0, 0.0, 1.0)
                * clamp((pos[2] - 1.5) / 1.0, 0.0, 1.0)
                * clamp(1.0 - hover_speed / 0.20, 0.0, 1.0);

        let pitch_dist = clamp(-(1.2 * gs) * body_fwd_err + (1.8 * gs) * body_fwd_vel + self.i_fwd, -1.5, 1.5);
        let roll_dist = clamp((1.2 * gs) * body_left_err - (1.8 * gs) * body_left_vel + self.i_left, -1.5, 1.5);

        let yaw_err = (target_yaw - yaw).sin().atan2((target_yaw - yaw).cos());
        let yaw_dist = clamp(1.2 * yaw_err, -1.0, 1.0);

        let clamped_alt = clamp(target_alt - pos[2] + Self::K_VERTICAL_OFFSET, -1.0, 1.0);
        let vertical_in = Self::K_VERTICAL_P * clamped_alt.powi(3);

        let roll_in = Self::K_ROLL_P * clamp(roll, -1.0, 1.0) + gyro[0] + roll_dist;
        let pitch_in = Self::K_PITCH_P * clamp(pitch, -1.0, 1.0) + gyro[1] + pitch_dist;
        let yaw_in = yaw_dist - 0.5 * gyro[2];

        let fl = Self::K_VERTICAL_THRUST + vertical_in - roll_in + pitch_in - yaw_in;
        let fr = Self::K_VERTICAL_THRUST + vertical_in + roll_in + pitch_in + yaw_in;
        let rl = Self::K_VERTICAL_THRUST + vertical_in - roll_in - pitch_in + yaw_in;
        let rr = Self::K_VERTICAL_THRUST + vertical_in + roll_in - pitch_in - yaw_in;

        [fl, -fr, -rl, rr]
    }
}
