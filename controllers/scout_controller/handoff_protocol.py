# File: controllers/*/handoff_protocol.py
import json
import base64
import zlib
import cv2
import numpy as np

class HandoffPacket:
    """
    Encapsulates the Target Handoff Packet broadcasted by the Scout Drone:
    - scout_id: Identifier of the sending drone
    - scout_gps: [x, y, z] position of the scout drone at the moment of marking
    - scout_yaw: Heading (radians) of the scout drone
    - target_gps: [x, y, z] calculated world coordinates of the ground target
    - direction_vector: [dx, dy, dz] unit vector from scout to target
    - distance: Euclidean distance (meters) from scout to target
    - image_chip: Cropped reference BGR image (numpy array) of the target area
    - seq: Broadcast sequence number (0 = the original mark broadcast; the
      Scout's 1 Hz loiter re-broadcasts increment it — the recovery path of
      the Phase 8 degraded-link emulation)
    - tx_time_s: Simulation time at which this broadcast left the emitter
    - crc_valid: Set by deserialize(); True when the CRC32 payload checksum
      computed at the receiver matches the transmitted one
    """
    def __init__(self, scout_id: str, scout_gps: list, scout_yaw: float,
                 target_gps: list, direction_vector: list, distance: float,
                 image_chip: np.ndarray, seq: int = 0, tx_time_s: float = 0.0):
        self.scout_id = scout_id
        self.scout_gps = [float(v) for v in scout_gps]
        self.scout_yaw = float(scout_yaw)
        self.target_gps = [float(v) for v in target_gps]
        self.direction_vector = [float(v) for v in direction_vector]
        self.distance = float(distance)
        self.image_chip = image_chip
        self.seq = int(seq)
        self.tx_time_s = float(tx_time_s)
        self.crc_valid = None

    def serialize(self) -> bytes:
        """Compresses the image chip to JPEG and serializes the packet to UTF-8
        JSON bytes with a CRC32 integrity checksum over the payload."""
        success, encoded_img = cv2.imencode('.jpg', self.image_chip, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not success:
            raise RuntimeError("Failed to encode reference image chip.")
        b64_img = base64.b64encode(encoded_img.tobytes()).decode('utf-8')

        payload = {
            "scout_id": self.scout_id,
            "scout_gps": self.scout_gps,
            "scout_yaw": self.scout_yaw,
            "target_gps": self.target_gps,
            "direction_vector": self.direction_vector,
            "distance": self.distance,
            "seq": self.seq,
            "tx_time_s": self.tx_time_s,
            "image_b64": b64_img
        }
        # CRC32 over the canonical JSON of the payload WITHOUT the crc field
        payload["crc"] = zlib.crc32(json.dumps(payload).encode('utf-8')) & 0xffffffff
        return json.dumps(payload).encode('utf-8')

    @staticmethod
    def deserialize(raw_bytes: bytes) -> 'HandoffPacket':
        """Reconstructs a HandoffPacket object (including decoded OpenCV image)
        from raw bytes and verifies the CRC32 payload checksum."""
        payload = json.loads(raw_bytes.decode('utf-8'))
        crc_rx = payload.pop("crc", None)
        crc_calc = zlib.crc32(json.dumps(payload).encode('utf-8')) & 0xffffffff
        img_bytes = base64.b64decode(payload["image_b64"])
        np_arr = np.frombuffer(img_bytes, dtype=np.uint8)
        image_chip = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        packet = HandoffPacket(
            scout_id=payload["scout_id"],
            scout_gps=payload["scout_gps"],
            scout_yaw=payload["scout_yaw"],
            target_gps=payload["target_gps"],
            direction_vector=payload["direction_vector"],
            distance=payload["distance"],
            image_chip=image_chip,
            seq=payload.get("seq", 0),
            tx_time_s=payload.get("tx_time_s", 0.0)
        )
        packet.crc_valid = (crc_rx == crc_calc)
        return packet
