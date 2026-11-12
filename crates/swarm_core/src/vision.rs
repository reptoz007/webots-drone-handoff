use image::RgbImage;
use rand::{rngs::StdRng, Rng, SeedableRng};
use std::collections::VecDeque;

#[derive(Debug, Clone)]
pub struct RedBlob {
    pub area: f64,
    pub cx: f64,
    pub cy: f64,
    pub corners: [(f64, f64); 4],
    pub bbox: (usize, usize, usize, usize),
}

#[derive(Debug, Clone)]
pub struct Keypoint {
    pub x: f32,
    pub y: f32,
    pub angle: f32,
    pub score: i32,
    pub desc: [u64; 4],
}

#[derive(Debug, Clone)]
pub struct VisualIdResult {
    pub confirmed: bool,
    pub good_matches: usize,
    pub inliers: usize,
    pub red_area_px: f64,
}

#[derive(Debug, Clone)]
pub struct FiducialResult {
    pub verified: bool,
    pub target_yaw_deg: Option<f64>,
    pub marker_separation_m: Option<f64>,
    pub midpoint_rel: Option<[f64; 2]>,
}

pub fn bgra_to_rgb(bgra: &[u8], w: usize, h: usize) -> RgbImage {
    let mut rgb = vec![0u8; w * h * 3];
    for i in 0..(w * h) {
        rgb[i * 3] = bgra[i * 4 + 2];
        rgb[i * 3 + 1] = bgra[i * 4 + 1];
        rgb[i * 3 + 2] = bgra[i * 4];
    }
    RgbImage::from_raw(w as u32, h as u32, rgb).unwrap()
}

#[inline(always)]
pub fn rgb_to_hsv_cv(r: u8, g: u8, b: u8) -> (u8, u8, u8) {
    let max = r.max(g).max(b);
    let min = r.min(g).min(b);
    let v = max;
    let delta = max - min;
    if delta == 0 || max == 0 {
        return (0, 0, v);
    }
    let s = ((delta as u32 * 255) / max as u32) as u8;
    let d = delta as i32;
    let mut h_deg: i32 = if max == r {
        (60 * (g as i32 - b as i32)) / d
    } else if max == g {
        120 + (60 * (b as i32 - r as i32)) / d
    } else {
        240 + (60 * (r as i32 - g as i32)) / d
    };
    if h_deg < 0 {
        h_deg += 360;
    }
    ((h_deg / 2) as u8, s, v)
}

#[inline(always)]
fn is_tight_red(r: u8, g: u8, b: u8) -> bool {
    let (h, s, v) = rgb_to_hsv_cv(r, g, b);
    (h <= 5 || h >= 175) && s >= 150 && v >= 100
}

#[inline(always)]
fn is_fiducial_black(r: u8, g: u8, b: u8) -> bool {
    let (_, s, v) = rgb_to_hsv_cv(r, g, b);
    s <= 120 && v <= 60
}

pub fn detect_red_blobs(img: &RgbImage, min_area: f64) -> Vec<RedBlob> {
    let w = img.width() as usize;
    let h = img.height() as usize;
    let raw = img.as_raw();
    let mut visited = vec![false; w * h];
    let mut queue = VecDeque::with_capacity(2048);
    let mut blobs = Vec::new();

    for y in 0..h {
        for x in 0..w {
            let idx = y * w + x;
            if visited[idx] {
                continue;
            }
            visited[idx] = true;
            if !is_tight_red(raw[idx * 3], raw[idx * 3 + 1], raw[idx * 3 + 2]) {
                continue;
            }

            let mut pts = Vec::with_capacity(1024);
            let mut sum_x = 0.0f64;
            let mut sum_y = 0.0f64;
            let (mut min_x, mut max_x, mut min_y, mut max_y) = (x, x, y, y);

            queue.push_back((x, y));
            while let Some((cx, cy)) = queue.pop_front() {
                pts.push((cx as f64, cy as f64));
                sum_x += cx as f64;
                sum_y += cy as f64;
                min_x = min_x.min(cx);
                max_x = max_x.max(cx);
                min_y = min_y.min(cy);
                max_y = max_y.max(cy);

                for dy in -1isize..=1 {
                    for dx in -1isize..=1 {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let nx = cx as isize + dx;
                        let ny = cy as isize + dy;
                        if nx >= 0 && nx < w as isize && ny >= 0 && ny < h as isize {
                            let nidx = ny as usize * w + nx as usize;
                            if !visited[nidx] {
                                visited[nidx] = true;
                                if is_tight_red(raw[nidx * 3], raw[nidx * 3 + 1], raw[nidx * 3 + 2]) {
                                    queue.push_back((nx as usize, ny as usize));
                                }
                            }
                        }
                    }
                }
            }

            let area = pts.len() as f64;
            if area < min_area {
                continue;
            }
            let corners = min_area_rect(&pts);
            blobs.push(RedBlob {
                area,
                cx: sum_x / area,
                cy: sum_y / area,
                corners,
                bbox: (min_x, min_y, max_x - min_x + 1, max_y - min_y + 1),
            });
        }
    }
    blobs
}

fn min_area_rect(pts: &[(f64, f64)]) -> [(f64, f64); 4] {
    let mut sorted = pts.to_vec();
    sorted.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
    sorted.dedup();
    if sorted.len() < 3 {
        let p = sorted[0];
        return [p, p, p, p];
    }

    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
        (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0)
    };

    let mut hull = Vec::with_capacity(sorted.len() * 2);
    for &p in &sorted {
        while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    let lower_len = hull.len() + 1;
    for &p in sorted.iter().rev() {
        while hull.len() >= lower_len && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
            hull.pop();
        }
        hull.push(p);
    }
    hull.pop();

    let mut best_area = f64::INFINITY;
    let mut best_corners = [(0.0, 0.0); 4];
    let m = hull.len();
    for i in 0..m {
        let p1 = hull[i];
        let p2 = hull[(i + 1) % m];
        let ex = p2.0 - p1.0;
        let ey = p2.1 - p1.1;
        let len = ex.hypot(ey);
        if len < 1e-9 {
            continue;
        }
        let ux = ex / len;
        let uy = ey / len;
        let vx = -uy;
        let vy = ux;

        let (mut min_u, mut max_u) = (f64::INFINITY, f64::NEG_INFINITY);
        let (mut min_v, mut max_v) = (f64::INFINITY, f64::NEG_INFINITY);
        for &q in &hull {
            let u = q.0 * ux + q.1 * uy;
            let v = q.0 * vx + q.1 * vy;
            min_u = min_u.min(u);
            max_u = max_u.max(u);
            min_v = min_v.min(v);
            max_v = max_v.max(v);
        }
        let area = (max_u - min_u) * (max_v - min_v);
        if area < best_area {
            best_area = area;
            let uv = [(min_u, min_v), (max_u, min_v), (max_u, max_v), (min_u, max_v)];
            for (k, &(u, v)) in uv.iter().enumerate() {
                best_corners[k] = (u * ux + v * vx, u * uy + v * vy);
            }
        }
    }
    best_corners
}

pub fn crop_reference_chip(img: &RgbImage, cx: f64, cy: f64, half_size: u32) -> RgbImage {
    let w = img.width();
    let h = img.height();
    let side = (half_size * 2).min(w).min(h);
    let x0 = (cx as i32 - half_size as i32).clamp(0, (w - side) as i32) as u32;
    let y0 = (cy as i32 - half_size as i32).clamp(0, (h - side) as i32) as u32;
    image::imageops::crop_imm(img, x0, y0, side, side).to_image()
}

pub fn extract_orb_features(img: &RgbImage, max_features: usize) -> Vec<Keypoint> {
    let w = img.width() as usize;
    let h = img.height() as usize;
    let raw = img.as_raw();
    let mut gray = vec![0u8; w * h];
    for i in 0..(w * h) {
        gray[i] = ((raw[i * 3] as u32 * 77 + raw[i * 3 + 1] as u32 * 150 + raw[i * 3 + 2] as u32 * 29) >> 8) as u8;
    }

    let circle: [(isize, isize); 16] = [
        (0, -3), (1, -3), (2, -2), (3, -1), (3, 0), (3, 1), (2, 2), (1, 3),
        (0, 3), (-1, 3), (-2, 2), (-3, 1), (-3, 0), (-3, -1), (-2, -2), (-1, -3),
    ];

    let mut rng = StdRng::seed_from_u64(0x5EED_04B2_56);
    let mut pairs = [((0i32, 0i32), (0i32, 0i32)); 256];
    for p in &mut pairs {
        p.0 = (rng.gen_range(-11..=11), rng.gen_range(-11..=11));
        p.1 = (rng.gen_range(-11..=11), rng.gen_range(-11..=11));
    }

    let thresh = 18i32;
    let mut kps = Vec::new();
    for y in 16..(h.saturating_sub(16)) {
        for x in 16..(w.saturating_sub(16)) {
            let c = gray[y * w + x] as i32;
            let (mut run_hi, mut run_lo, mut max_run, mut score) = (0, 0, 0, 0);
            for k in 0..32 {
                let (dx, dy) = circle[k & 15];
                let p = gray[(y as isize + dy) as usize * w + (x as isize + dx) as usize] as i32;
                let diff = p - c;
                if diff > thresh {
                    run_hi += 1;
                    run_lo = 0;
                    score += diff;
                } else if diff < -thresh {
                    run_lo += 1;
                    run_hi = 0;
                    score -= diff;
                } else {
                    run_hi = 0;
                    run_lo = 0;
                }
                max_run = max_run.max(run_hi.max(run_lo));
            }
            if max_run >= 9 {
                let (mut m10, mut m01) = (0.0f32, 0.0f32);
                for dy in -7isize..=7 {
                    for dx in -7isize..=7 {
                        if dx * dx + dy * dy <= 49 {
                            let val = gray[(y as isize + dy) as usize * w + (x as isize + dx) as usize] as f32;
                            m10 += dx as f32 * val;
                            m01 += dy as f32 * val;
                        }
                    }
                }
                let angle = m01.atan2(m10);
                let (sa, ca) = angle.sin_cos();

                let mut desc = [0u64; 4];
                for (bit, &((px1, py1), (px2, py2))) in pairs.iter().enumerate() {
                    let rx1 = (px1 as f32 * ca - py1 as f32 * sa).round() as isize;
                    let ry1 = (px1 as f32 * sa + py1 as f32 * ca).round() as isize;
                    let rx2 = (px2 as f32 * ca - py2 as f32 * sa).round() as isize;
                    let ry2 = (px2 as f32 * sa + py2 as f32 * ca).round() as isize;
                    let v1 = gray[(y as isize + ry1) as usize * w + (x as isize + rx1) as usize];
                    let v2 = gray[(y as isize + ry2) as usize * w + (x as isize + rx2) as usize];
                    if v1 < v2 {
                        desc[bit >> 6] |= 1u64 << (bit & 63);
                    }
                }
                kps.push(Keypoint {
                    x: x as f32,
                    y: y as f32,
                    angle,
                    score,
                    desc,
                });
            }
        }
    }

    kps.sort_by(|a, b| b.score.cmp(&a.score));
    kps.truncate(max_features);
    kps
}

#[inline(always)]
fn hamming_dist(a: &[u64; 4], b: &[u64; 4]) -> u32 {
    (a[0] ^ b[0]).count_ones()
        + (a[1] ^ b[1]).count_ones()
        + (a[2] ^ b[2]).count_ones()
        + (a[3] ^ b[3]).count_ones()
}

pub fn match_visual_id(ref_kp: &[Keypoint], live_img: &RgbImage) -> VisualIdResult {
    let blobs = detect_red_blobs(live_img, 800.0);
    let red_area = blobs.iter().map(|b| b.area).fold(0.0f64, f64::max);
    if red_area < 800.0 || ref_kp.len() < 8 {
        return VisualIdResult { confirmed: false, good_matches: 0, inliers: 0, red_area_px: red_area };
    }

    let live_kp = extract_orb_features(live_img, 600);
    if live_kp.len() < 8 {
        return VisualIdResult { confirmed: false, good_matches: 0, inliers: 0, red_area_px: red_area };
    }

    let mut good_pairs = Vec::new();
    for r in ref_kp {
        let (mut d1, mut d2) = (u32::MAX, u32::MAX);
        let mut best_j = 0;
        for (j, l) in live_kp.iter().enumerate() {
            let d = hamming_dist(&r.desc, &l.desc);
            if d < d1 {
                d2 = d1;
                d1 = d;
                best_j = j;
            } else if d < d2 {
                d2 = d;
            }
        }
        if (d1 as f32) < 0.80 * (d2 as f32) {
            good_pairs.push(((r.x as f64, r.y as f64), (live_kp[best_j].x as f64, live_kp[best_j].y as f64)));
        }
    }

    if good_pairs.len() < 10 {
        return VisualIdResult { confirmed: false, good_matches: good_pairs.len(), inliers: 0, red_area_px: red_area };
    }

    let inliers = ransac_homography_inliers(&good_pairs, 5.0, 200);
    VisualIdResult {
        confirmed: inliers >= 8,
        good_matches: good_pairs.len(),
        inliers,
        red_area_px: red_area,
    }
}

fn ransac_homography_inliers(pairs: &[((f64, f64), (f64, f64))], thresh_px: f64, iters: usize) -> usize {
    let n = pairs.len();
    if n < 4 {
        return 0;
    }
    let mut rng = StdRng::seed_from_u64(0x1234_5678);
    let mut best_inliers = 0;
    let t2 = thresh_px * thresh_px;

    for _ in 0..iters {
        let mut idx = [0usize; 4];
        for k in 0..4 {
            loop {
                let c = rng.gen_range(0..n);
                if !idx[..k].contains(&c) {
                    idx[k] = c;
                    break;
                }
            }
        }
        let sample = [pairs[idx[0]], pairs[idx[1]], pairs[idx[2]], pairs[idx[3]]];
        if let Some(h) = solve_homography_4pt(&sample) {
            let mut count = 0;
            for &((sx, sy), (dx, dy)) in pairs {
                let w = h[6] * sx + h[7] * sy + h[8];
                if w.abs() < 1e-8 {
                    continue;
                }
                let px = (h[0] * sx + h[1] * sy + h[2]) / w;
                let py = (h[3] * sx + h[4] * sy + h[5]) / w;
                if (px - dx).powi(2) + (py - dy).powi(2) <= t2 {
                    count += 1;
                }
            }
            best_inliers = best_inliers.max(count);
        }
    }
    best_inliers
}

fn solve_homography_4pt(pts: &[((f64, f64), (f64, f64)); 4]) -> Option<[f64; 9]> {
    let mut a = [[0.0f64; 9]; 8];
    for (i, &((x, y), (u, v))) in pts.iter().enumerate() {
        a[2 * i] = [-x, -y, -1.0, 0.0, 0.0, 0.0, u * x, u * y, u];
        a[2 * i + 1] = [0.0, 0.0, 0.0, -x, -y, -1.0, v * x, v * y, v];
    }
    for col in 0..8 {
        let mut piv = col;
        for row in (col + 1)..8 {
            if a[row][col].abs() > a[piv][col].abs() {
                piv = row;
            }
        }
        if a[piv][col].abs() < 1e-10 {
            return None;
        }
        a.swap(col, piv);
        let div = a[col][col];
        for c in col..9 {
            a[col][c] /= div;
        }
        for row in 0..8 {
            if row != col {
                let factor = a[row][col];
                for c in col..9 {
                    a[row][c] -= factor * a[col][c];
                }
            }
        }
    }
    Some([-a[0][8], -a[1][8], -a[2][8], -a[3][8], -a[4][8], -a[5][8], -a[6][8], -a[7][8], 1.0])
}

/// Phase 10 geometric fiducial verification with border-clip guard.
pub fn verify_fiducial<F>(img: &RgbImage, invert_rel: F) -> FiducialResult
where
    F: Fn(f64, f64) -> Option<[f64; 2]>,
{
    let w = img.width() as usize;
    let h = img.height() as usize;
    let red_blobs = detect_red_blobs(img, 800.0);
    let Some(plate) = red_blobs.iter().max_by(|a, b| a.area.total_cmp(&b.area)) else {
        return FiducialResult { verified: false, target_yaw_deg: None, marker_separation_m: None, midpoint_rel: None };
    };

    // Border-clip guard: never read fiducial geometry when the red plate touches the image frame edge
    let (rbx, rby, rbw, rbh) = plate.bbox;
    if rbx <= 4 || rby <= 4 || rbx + rbw >= w - 5 || rby + rbh >= h - 5 {
        return FiducialResult { verified: false, target_yaw_deg: None, marker_separation_m: None, midpoint_rel: None };
    }

    let pad = 0.05;
    let rx0 = rbx as f64 + rbw as f64 * pad;
    let rx1 = rbx as f64 + rbw as f64 * (1.0 - pad);
    let ry0 = rby as f64 + rbh as f64 * pad;
    let ry1 = rby as f64 + rbh as f64 * (1.0 - pad);

    let raw = img.as_raw();
    let mut visited = vec![false; w * h];
    let mut queue = VecDeque::with_capacity(512);
    let mut black_blobs: Vec<(f64, f64, f64)> = Vec::new();

    for y in rby..(rby + rbh).min(h) {
        for x in rbx..(rbx + rbw).min(w) {
            let idx = y * w + x;
            if visited[idx] {
                continue;
            }
            visited[idx] = true;
            if !is_fiducial_black(raw[idx * 3], raw[idx * 3 + 1], raw[idx * 3 + 2]) {
                continue;
            }
            let (mut area, mut sx, mut sy) = (0.0, 0.0, 0.0);
            queue.push_back((x, y));
            while let Some((cx, cy)) = queue.pop_front() {
                area += 1.0;
                sx += cx as f64;
                sy += cy as f64;
                for dy in -1isize..=1 {
                    for dx in -1isize..=1 {
                        let nx = cx as isize + dx;
                        let ny = cy as isize + dy;
                        if nx >= 0 && nx < w as isize && ny >= 0 && ny < h as isize {
                            let nidx = ny as usize * w + nx as usize;
                            if !visited[nidx] {
                                visited[nidx] = true;
                                if is_fiducial_black(raw[nidx * 3], raw[nidx * 3 + 1], raw[nidx * 3 + 2]) {
                                    queue.push_back((nx as usize, ny as usize));
                                }
                            }
                        }
                    }
                }
            }
            if area >= 120.0 {
                let bcx = sx / area;
                let bcy = sy / area;
                if bcx >= rx0 && bcx <= rx1 && bcy >= ry0 && bcy <= ry1 {
                    black_blobs.push((area, bcx, bcy));
                }
            }
        }
    }

    black_blobs.sort_by(|a, b| b.0.total_cmp(&a.0));
    if black_blobs.len() < 2 {
        return FiducialResult { verified: false, target_yaw_deg: None, marker_separation_m: None, midpoint_rel: None };
    }
    let (area_a, cax, cay) = black_blobs[0];
    let (area_b, cbx, cby) = black_blobs[1];
    if area_a < 1.3 * area_b {
        return FiducialResult { verified: false, target_yaw_deg: None, marker_separation_m: None, midpoint_rel: None };
    }

    let (Some(pa), Some(pb)) = (invert_rel(cax, cay), invert_rel(cbx, cby)) else {
        return FiducialResult { verified: false, target_yaw_deg: None, marker_separation_m: None, midpoint_rel: None };
    };

    let vx = pa[0] - pb[0];
    let vy = pa[1] - pb[1];
    let sep = vx.hypot(vy);
    let ang = vy.atan2(vx).to_degrees().rem_euclid(360.0);
    let ang_err = ((ang - 45.7 + 180.0).rem_euclid(360.0) - 180.0).abs();
    let verified = ang_err < 25.0 && (sep - 1.17).abs() < 0.45;
    let mid = [(pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5];

    FiducialResult {
        verified,
        target_yaw_deg: Some(ang),
        marker_separation_m: Some(sep),
        midpoint_rel: Some(mid),
    }
}
