//! Quick CRC round-trip verification for the Rust handoff protocol.
use swarm_core::protocol::HandoffPacket;

fn main() {
    // Reproduce the scout's seq-0 packet shape (initial mark broadcast)
    let chip = image::RgbImage::new(300, 300);
    let pkt = HandoffPacket {
        scout_id: "Scout_Drone_01".into(),
        scout_gps: [-6.104, -3.982, 4.501],
        scout_yaw: 0.6108,
        target_gps: [6.65, -0.55, 0.15],
        target_vel: [0.09, 0.06, 0.0],
        direction_vector: [0.936, -0.351, -0.0134],
        distance: 12.94,
        image_chip: chip,
        seq: 0,
        tx_time_s: 91.6422348022461,
        crc_valid: true,
    };
    let bytes = pkt.serialize().unwrap();
    println!("serialized {} bytes", bytes.len());
    match HandoffPacket::deserialize(&bytes) {
        Ok(rx) => println!("seq 0 round-trip crc_valid = {}", rx.crc_valid),
        Err(e) => println!("deserialize failed: {e}"),
    }

    // And a mutated re-broadcast (seq 2, accumulated target position)
    let mut pkt2 = pkt;
    pkt2.seq = 2;
    pkt2.target_gps[0] += 0.09 + 0.09;
    pkt2.target_gps[1] += 0.06 + 0.06;
    pkt2.tx_time_s = 93.71240234375;
    let bytes2 = pkt2.serialize().unwrap();
    match HandoffPacket::deserialize(&bytes2) {
        Ok(rx) => println!("seq 2 round-trip crc_valid = {}", rx.crc_valid),
        Err(e) => println!("deserialize failed: {e}"),
    }
}
