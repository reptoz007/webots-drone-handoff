use base64::{engine::general_purpose::STANDARD as B64, Engine as _};
use crc32fast::Hasher;
use image::{codecs::jpeg::JpegEncoder, ExtendedColorType, ImageReader, RgbImage};
use serde::{Deserialize, Serialize};
use std::io::Cursor;

#[derive(Debug, Clone, Serialize, Deserialize)]
struct WirePayload {
    scout_id: String,
    scout_gps: [f64; 3],
    scout_yaw: f64,
    target_gps: [f64; 3],
    #[serde(default)]
    target_vel: [f64; 3],
    direction_vector: [f64; 3],
    distance: f64,
    seq: i32,
    tx_time_s: f64,
    image_b64: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    crc: Option<u32>,
}

#[derive(Debug, Clone)]
pub struct HandoffPacket {
    pub scout_id: String,
    pub scout_gps: [f64; 3],
    pub scout_yaw: f64,
    pub target_gps: [f64; 3],
    pub target_vel: [f64; 3],
    pub direction_vector: [f64; 3],
    pub distance: f64,
    pub image_chip: RgbImage,
    pub seq: i32,
    pub tx_time_s: f64,
    pub crc_valid: bool,
}

impl HandoffPacket {
    pub fn serialize(&self) -> Result<Vec<u8>, String> {
        let mut jpeg_buf = Vec::with_capacity(16 * 1024);
        let mut encoder = JpegEncoder::new_with_quality(&mut jpeg_buf, 85);
        encoder
            .encode(
                self.image_chip.as_raw(),
                self.image_chip.width(),
                self.image_chip.height(),
                ExtendedColorType::Rgb8,
            )
            .map_err(|e| format!("JPEG encode failed: {e}"))?;

        let b64_img = B64.encode(&jpeg_buf);
        let mut payload = WirePayload {
            scout_id: self.scout_id.clone(),
            scout_gps: self.scout_gps,
            scout_yaw: self.scout_yaw,
            target_gps: self.target_gps,
            target_vel: self.target_vel,
            direction_vector: self.direction_vector,
            distance: self.distance,
            seq: self.seq,
            tx_time_s: self.tx_time_s,
            image_b64: b64_img,
            crc: None,
        };

        let canonical = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;
        let mut hasher = Hasher::new();
        hasher.update(&canonical);
        payload.crc = Some(hasher.finalize());

        serde_json::to_vec(&payload).map_err(|e| e.to_string())
    }

    pub fn deserialize(raw_bytes: &[u8]) -> Result<Self, String> {
        let mut payload: WirePayload =
            serde_json::from_slice(raw_bytes).map_err(|e| format!("JSON parse failed: {e}"))?;
        let rx_crc = payload.crc.take();
        let canonical = serde_json::to_vec(&payload).map_err(|e| e.to_string())?;
        let mut hasher = Hasher::new();
        hasher.update(&canonical);
        let crc_valid = rx_crc == Some(hasher.finalize());

        let jpg_bytes = B64
            .decode(&payload.image_b64)
            .map_err(|e| format!("Base64 decode failed: {e}"))?;
        let img = ImageReader::new(Cursor::new(jpg_bytes))
            .with_guessed_format()
            .map_err(|e| e.to_string())?
            .decode()
            .map_err(|e| format!("JPEG decode failed: {e}"))?
            .to_rgb8();

        Ok(Self {
            scout_id: payload.scout_id,
            scout_gps: payload.scout_gps,
            scout_yaw: payload.scout_yaw,
            target_gps: payload.target_gps,
            target_vel: payload.target_vel,
            direction_vector: payload.direction_vector,
            distance: payload.distance,
            image_chip: img,
            seq: payload.seq,
            tx_time_s: payload.tx_time_s,
            crc_valid,
        })
    }
}
