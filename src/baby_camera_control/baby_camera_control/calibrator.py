"""
Camera calibration for baby_camera_control.

Measures how many pixels the image shifts per ONVIF position unit for pan and tilt.
Saves the result to a JSON file that palm_tracker loads at startup.

Usage (while camera image topic is publishing, WITHOUT ptz_controller running):
  ros2 run baby_camera_control calibrator --ros-args \\
    -p host:=<ip> -p username:=<user> -p password:=<pass>
"""
import json
import threading
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from onvif import ONVIFCamera
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

DEFAULT_CALIBRATION_PATH = str(Path.home() / ".ros" / "baby_cam_calibration.json")
MOVE_DELTA = 0.15
SETTLE_SEC = 2.5


class Calibrator(Node):
    def __init__(self):
        super().__init__("baby_cam_calibrator")

        self.declare_parameter("host", "")
        self.declare_parameter("port", 2020)
        self.declare_parameter("username", "")
        self.declare_parameter("password", "")
        self.declare_parameter("profile_index", 0)
        self.declare_parameter("image_topic", "/baby_cam/image_raw")
        self.declare_parameter("calibration_file", DEFAULT_CALIBRATION_PATH)
        self.declare_parameter("move_delta", MOVE_DELTA)
        self.declare_parameter("pan_delta", -1.0)
        self.declare_parameter("tilt_delta", -1.0)
        self.declare_parameter("settle_sec", SETTLE_SEC)

        self.bridge = CvBridge()
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._new_frame_event = threading.Event()

        image_topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f"Waiting for frames on {image_topic}...")

    def _on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return
        with self._frame_lock:
            self._latest_frame = frame
        self._new_frame_event.set()

    def _capture_frame(self, timeout=8.0):
        self._new_frame_event.clear()
        if not self._new_frame_event.wait(timeout):
            return None
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def _connect(self):
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        username = self.get_parameter("username").value
        password = self.get_parameter("password").value
        profile_index = int(self.get_parameter("profile_index").value)
        camera = ONVIFCamera(host, port, username, password)
        media = camera.create_media_service()
        ptz = camera.create_ptz_service()
        token = media.GetProfiles()[profile_index].token
        return ptz, token

    def _absolute_move(self, ptz, token, pan, tilt):
        req = ptz.create_type("AbsoluteMove")
        req.ProfileToken = token
        req.Position = {"PanTilt": {"x": float(pan), "y": float(tilt)}, "Zoom": {"x": 0.0}}
        ptz.AbsoluteMove(req)

    def _relative_move(self, ptz, token, pan, tilt):
        req = ptz.create_type("RelativeMove")
        req.ProfileToken = token
        req.Translation = {"PanTilt": {"x": float(pan), "y": float(tilt)}, "Zoom": {"x": 0.0}}
        ptz.RelativeMove(req)

    @staticmethod
    def _measure_shift(ref_frame, moved_frame):
        """Returns (dx, dy): how far image content moved from ref to moved."""
        gray_ref = cv2.cvtColor(ref_frame, cv2.COLOR_BGR2GRAY).astype("float32")
        gray_mov = cv2.cvtColor(moved_frame, cv2.COLOR_BGR2GRAY).astype("float32")
        (dx, dy), _ = cv2.phaseCorrelate(gray_ref, gray_mov)
        return dx, dy

    def run_calibration(self):
        log = self.get_logger()
        log.info("Connecting to ONVIF camera...")
        try:
            ptz, token = self._connect()
        except Exception as exc:
            log.error(f"Connection failed: {exc}")
            return

        default_delta = float(self.get_parameter("move_delta").value)
        pan_delta = float(self.get_parameter("pan_delta").value)
        tilt_delta = float(self.get_parameter("tilt_delta").value)
        if pan_delta < 0:
            pan_delta = default_delta
        if tilt_delta < 0:
            tilt_delta = default_delta
        settle = float(self.get_parameter("settle_sec").value)
        out_path = self.get_parameter("calibration_file").value

        log.info("Moving to center position...")
        self._absolute_move(ptz, token, 0.0, 0.0)
        time.sleep(settle)

        ref = self._capture_frame()
        if ref is None:
            log.error("No image received. Is the camera topic publishing?")
            return

        h, w = ref.shape[:2]
        log.info(f"Reference frame: {w}x{h}")

        pan_factors = []
        tilt_factors = []

        for axis, sign in [("pan", +1), ("pan", -1), ("tilt", +1), ("tilt", -1)]:
            label = f"{axis} {'↑/→' if sign > 0 else '↓/←'}"
            log.info(f"Measuring {label}...")

            self._absolute_move(ptz, token, 0.0, 0.0)
            time.sleep(settle)
            ref_frame = self._capture_frame()

            delta = pan_delta if axis == "pan" else tilt_delta
            pan_d = delta * sign if axis == "pan" else 0.0
            tilt_d = delta * sign if axis == "tilt" else 0.0
            self._relative_move(ptz, token, pan_d, tilt_d)
            time.sleep(settle)
            moved_frame = self._capture_frame()

            if ref_frame is None or moved_frame is None:
                log.warning(f"  Missing frame, skipping {label}")
                continue

            dx, dy = self._measure_shift(ref_frame, moved_frame)
            log.info(f"  ONVIF delta={delta * sign:+.3f}  shift=({dx:+.1f}, {dy:+.1f}) px")


            if axis == "pan":
                if abs(dx) < 2.0:
                    log.warning("  Pan shift too small — is the camera actually moving?")
                    continue
                # Formula: pan_delta = error_x * pan_error_to_onvif
                # error_x + pan_delta * dx/ONVIF_delta = 0  →  pan_delta = -error_x * ONVIF_delta/dx
                pan_factors.append(-(delta * sign) / dx)
            else:
                if abs(dy) < 2.0:
                    log.warning("  Tilt shift too small — is the camera actually moving?")
                    continue
                tilt_factors.append(-(delta * sign) / dy)

        self._absolute_move(ptz, token, 0.0, 0.0)

        if not pan_factors or not tilt_factors:
            log.error("Calibration failed: too few valid measurements.")
            return

        pan_e2o = sum(pan_factors) / len(pan_factors)
        tilt_e2o = sum(tilt_factors) / len(tilt_factors)

        calibration = {
            "pan_error_to_onvif": pan_e2o,
            "tilt_error_to_onvif": tilt_e2o,
            "image_width": w,
            "image_height": h,
        }

        Path(out_path).parent.mkdir(parents=True, exist_ok=True)
        with open(out_path, "w") as f:
            json.dump(calibration, f, indent=2)

        log.info(
            f"\n=== Calibration complete ===\n"
            f"  pan_error_to_onvif  = {pan_e2o:.6f}  (px→ONVIF)\n"
            f"  tilt_error_to_onvif = {tilt_e2o:.6f}  (px→ONVIF)\n"
            f"  Saved → {out_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Calibrator()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
