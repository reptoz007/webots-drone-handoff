//! Offline diagnostic: does match_visual_id work on the saved evidence frames?
use swarm_core::vision::{extract_orb_features, match_visual_id};
use std::path::Path;

fn load(p: &str) -> image::RgbImage {
    let path = ["results", "../../results"]
        .iter()
        .map(|base| Path::new(base).join(p))
        .find(|c| c.exists())
        .expect("image not found in results/ or ../../results/");
    image::ImageReader::open(&path)
        .expect("open")
        .decode()
        .expect("decode")
        .to_rgb8()
}

fn main() {
    let chip = load("scout_chip.png");
    println!("chip {}x{}", chip.width(), chip.height());
    let ref_kp = extract_orb_features(&chip, 600);
    println!("ref features: {}", ref_kp.len());

    for frame in ["scout_front_full.png", "lock_frame_Follower_Drone_01.png"] {
        let live = load(frame);
        println!("-- {} ({}x{})", frame, live.width(), live.height());
        let live_kp = extract_orb_features(&live, 600);
        println!("   live features: {}", live_kp.len());
        let res = match_visual_id(&ref_kp, &live);
        println!(
            "   confirmed={} good={} inliers={} red_area={}",
            res.confirmed, res.good_matches, res.inliers, res.red_area_px
        );
        // Ratio histogram for diagnosis: best vs second-best distances
        let mut ratios = vec![];
        for r in &ref_kp {
            let (mut d1, mut d2) = (u32::MAX, u32::MAX);
            for l in &live_kp {
                let d = (r.desc[0] ^ l.desc[0]).count_ones()
                    + (r.desc[1] ^ l.desc[1]).count_ones()
                    + (r.desc[2] ^ l.desc[2]).count_ones()
                    + (r.desc[3] ^ l.desc[3]).count_ones();
                if d < d1 {
                    d2 = d1;
                    d1 = d;
                } else if d < d2 {
                    d2 = d;
                }
            }
            if d2 < u32::MAX && d1 < u32::MAX {
                ratios.push(d1 as f32 / d2 as f32);
            }
        }
        ratios.sort_by(|a, b| a.total_cmp(b));
        if !ratios.is_empty() {
            let n = ratios.len();
            println!(
                "   ratio dist: p10={:.2} p25={:.2} p50={:.2} count<0.8={}",
                ratios[n / 10],
                ratios[n / 4],
                ratios[n / 2],
                ratios.iter().filter(|&&r| r < 0.8).count()
            );
        }
    }
}
