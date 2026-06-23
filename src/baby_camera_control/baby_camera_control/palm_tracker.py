import json
import os
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import mediapipe as mp
except ImportError as exc:
    mp = None
    MEDIAPIPE_IMPORT_ERROR = exc
else:
    MEDIAPIPE_IMPORT_ERROR = None

DEFAULT_CALIBRATION_PATH = str(Path.home() / ".ros" / "baby_cam_calibration.json")


class PalmTracker(Node):
    def __init__(self):
        super().__init__("palm_tracker")

        self.declare_parameter("image_topic", "/baby_cam/image_raw")
        self.declare_parameter("tracking_image_topic", "/baby_cam/tracking_image")
        self.declare_parameter("command_topic", "/baby_cam/ptz_cmd")
        self.declare_parameter("calibration_file", DEFAULT_CALIBRATION_PATH)
        self.declare_parameter("center_tolerance_px", 50)
        self.declare_parameter("movement_start_tolerance_px", 90)
        self.declare_parameter("max_pan_delta", 0.3)
        self.declare_parameter("max_tilt_delta", 0.3)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.5)
        self.declare_parameter("process_every_n_frames", 1)
        self.declare_parameter("max_processing_fps", 10.0)
        self.declare_parameter("processing_scale", 0.5)
        self.declare_parameter("publish_tracking_every_n_frames", 2)
        self.declare_parameter("min_hand_size_px", 80)
        self.declare_parameter("min_landmark_visibility_ratio", 0.9)
        self.declare_parameter("stable_palm_frames", 2)
        self.declare_parameter("draw_landmarks", False)
        self.declare_parameter("post_move_pause_sec", 1.5)

        if mp is None:
            raise RuntimeError(
                "mediapipe is required for palm tracking. Install it with "
                "'python3 -m pip install mediapipe'."
            ) from MEDIAPIPE_IMPORT_ERROR

        image_topic = self.get_parameter("image_topic").value
        tracking_image_topic = self.get_parameter("tracking_image_topic").value
        command_topic = self.get_parameter("command_topic").value

        calibration_file = self.get_parameter("calibration_file").value
        self.cal = self._load_calibration(calibration_file)

        self.center_tolerance_px = int(self.get_parameter("center_tolerance_px").value)
        self.movement_start_tolerance_px = max(
            self.center_tolerance_px,
            int(self.get_parameter("movement_start_tolerance_px").value),
        )
        self.max_pan_delta = float(self.get_parameter("max_pan_delta").value)
        self.max_tilt_delta = float(self.get_parameter("max_tilt_delta").value)
        self.process_every_n_frames = max(
            1, int(self.get_parameter("process_every_n_frames").value)
        )
        self.max_processing_fps = float(self.get_parameter("max_processing_fps").value)
        self.min_processing_interval = (
            1.0 / self.max_processing_fps if self.max_processing_fps > 0.0 else 0.0
        )
        self.processing_scale = self._clamp(
            self.get_parameter("processing_scale").value, 0.1, 1.0
        )
        self.publish_tracking_every_n_frames = max(
            1, int(self.get_parameter("publish_tracking_every_n_frames").value)
        )
        self.min_hand_size_px = int(self.get_parameter("min_hand_size_px").value)
        self.min_landmark_visibility_ratio = self._clamp(
            self.get_parameter("min_landmark_visibility_ratio").value, 0.0, 1.0
        )
        self.stable_palm_frames = max(
            1, int(self.get_parameter("stable_palm_frames").value)
        )
        self.draw_landmarks = bool(self.get_parameter("draw_landmarks").value)

        self.bridge = CvBridge()
        self.command_publisher = self.create_publisher(Twist, command_topic, 10)
        self.tracking_publisher = self.create_publisher(
            Image, tracking_image_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data
        )

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            model_complexity=0,
            max_num_hands=1,
            min_detection_confidence=float(
                self.get_parameter("min_detection_confidence").value
            ),
            min_tracking_confidence=float(
                self.get_parameter("min_tracking_confidence").value
            ),
        )
        self.drawer = mp.solutions.drawing_utils
        self.hand_connections = mp.solutions.hands.HAND_CONNECTIONS

        self.pan_tracking_active = False
        self.tilt_tracking_active = False
        self.frame_count = 0
        self.processed_frame_count = 0
        self.last_processing_time = 0.0
        self.palm_frame_count = 0
        self.last_frame_stamp = None
        self.post_move_pause_sec = float(self.get_parameter("post_move_pause_sec").value)
        self.resume_processing_after = 0.0

        self.get_logger().info(
            f"Tracking open palms from {image_topic}; publishing PTZ commands on "
            f"{command_topic} and annotated images on {tracking_image_topic}"
        )

    def _load_calibration(self, path):
        path = os.path.expanduser(path)
        try:
            with open(path) as f:
                cal = json.load(f)
            self.get_logger().info(
                f"Calibration loaded from {path}: "
                f"pan={cal['pan_error_to_onvif']:.6f}, "
                f"tilt={cal['tilt_error_to_onvif']:.6f}"
            )
            return cal
        except FileNotFoundError:
            self.get_logger().fatal(
                f"Calibration file not found: {path}\n"
                "Run 'ros2 run baby_camera_control calibrator' first."
            )
            raise

    def on_image(self, msg):
        # Skip duplicate frames (same timestamp = same frame, camera still moving)
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_frame_stamp:
            return
        self.last_frame_stamp = stamp

        self.frame_count += 1
        now = time.monotonic()
        if self.frame_count % self.process_every_n_frames != 0:
            return
        if (
            self.min_processing_interval > 0.0
            and now - self.last_processing_time < self.min_processing_interval
        ):
            return
        if now < self.resume_processing_after:
            if self.should_publish_tracking_image():
                try:
                    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    self.publish_tracking_image(frame, msg)
                except Exception:
                    pass
            return
        self.last_processing_time = now
        self.processed_frame_count += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Could not convert image: {exc}")
            return

        height, width = frame.shape[:2]
        processing_frame = self.processing_frame(frame)
        rgb = cv2.cvtColor(processing_frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        if not result.multi_hand_landmarks:
            self.get_logger().info("no hand detected by mediapipe", throttle_duration_sec=1.0)
            self.stop_tracking(frame, msg, "no palm")
            return

        hand = result.multi_hand_landmarks[0]
        landmarks = hand.landmark
        center_x, center_y = self.hand_center(landmarks, width, height)
        if self.draw_landmarks:
            cv2.circle(frame, (int(center_x), int(center_y)), 12, (0, 255, 0), -1)
        confident = self.is_confident_hand(landmarks, width, height)
        is_palm = self.is_palm(landmarks)
        if not confident or not is_palm:
            fingers = self.extended_fingers(landmarks)
            hand_w, hand_h = self.hand_size(landmarks, width, height)
            vis = sum(1 for lm in landmarks if 0.0 <= lm.x <= 1.0 and 0.0 <= lm.y <= 1.0)
            self.get_logger().info(
                f"no palm: confident={confident} is_palm={is_palm} "
                f"fingers={fingers} hand={hand_w:.0f}x{hand_h:.0f}px vis={vis}/21",
                throttle_duration_sec=1.0,
            )
        palm_visible = confident and is_palm
        if not self.stable_palm(palm_visible):
            self.stop_tracking(frame, msg, "no palm", center_x, center_y)
            return

        if self.should_publish_tracking_image():
            self.draw_overlay(frame, "palm", center_x, center_y)
            self.publish_tracking_image(frame, msg)
        self.get_logger().info(
            f"PALM detected at ({center_x:.0f}, {center_y:.0f}) img={width}x{height}",
            throttle_duration_sec=1.0,
        )
        self.publish_centering_command(center_x, center_y, width, height)

    def processing_frame(self, frame):
        if self.processing_scale >= 1.0:
            return frame
        return cv2.resize(
            frame,
            None,
            fx=self.processing_scale,
            fy=self.processing_scale,
            interpolation=cv2.INTER_AREA,
        )

    def should_publish_tracking_image(self):
        return self.processed_frame_count % self.publish_tracking_every_n_frames == 0

    def is_confident_hand(self, landmarks, width, height):
        hand_width, hand_height = self.hand_size(landmarks, width, height)
        return max(hand_width, hand_height) >= self.min_hand_size_px

    def hand_size(self, landmarks, width, height):
        xs = [point.x for point in landmarks]
        ys = [point.y for point in landmarks]
        return ((max(xs) - min(xs)) * width, (max(ys) - min(ys)) * height)

    def is_palm(self, landmarks):
        return self.extended_fingers(landmarks) >= 3

    def extended_fingers(self, landmarks):
        # MCP(너클)→tip 거리 vs MCP→PIP 거리 비교
        count = 0
        for mcp, pip, tip in ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)):
            dx_tip = landmarks[tip].x - landmarks[mcp].x
            dy_tip = landmarks[tip].y - landmarks[mcp].y
            dx_pip = landmarks[pip].x - landmarks[mcp].x
            dy_pip = landmarks[pip].y - landmarks[mcp].y
            if dx_tip ** 2 + dy_tip ** 2 > dx_pip ** 2 + dy_pip ** 2:
                count += 1
        return count

    def stable_palm(self, palm_visible):
        if palm_visible:
            self.palm_frame_count += 1
        else:
            self.palm_frame_count = 0
        return self.palm_frame_count >= self.stable_palm_frames

    def publish_centering_command(self, center_x, center_y, width, height):
        error_x = center_x - (width / 2.0)
        error_y = center_y - (height / 2.0)

        self.pan_tracking_active = self.should_move_axis(
            abs(error_x), self.pan_tracking_active
        )
        self.tilt_tracking_active = self.should_move_axis(
            abs(error_y), self.tilt_tracking_active
        )

        pan_delta = 0.0
        tilt_delta = 0.0

        if self.pan_tracking_active:
            pan_delta = self._clamp(
                -error_x * self.cal["pan_error_to_onvif"],
                -self.max_pan_delta,
                self.max_pan_delta,
            )

        if self.tilt_tracking_active:
            tilt_delta = self._clamp(
                -error_y * self.cal["tilt_error_to_onvif"],
                -self.max_tilt_delta,
                self.max_tilt_delta,
            )

        command = Twist()
        command.angular.z = pan_delta
        command.angular.y = tilt_delta
        self.command_publisher.publish(command)
        if pan_delta != 0.0 or tilt_delta != 0.0:
            self.resume_processing_after = time.monotonic() + self.post_move_pause_sec

    def should_move_axis(self, abs_error, currently_active):
        if abs_error <= self.center_tolerance_px:
            return False
        if abs_error >= self.movement_start_tolerance_px:
            return True
        return currently_active

    def stop_tracking(self, frame, source_msg, label, center_x=None, center_y=None):
        self.pan_tracking_active = False
        self.tilt_tracking_active = False
        self.palm_frame_count = 0
        self.command_publisher.publish(Twist())
        if self.should_publish_tracking_image():
            if center_x is not None and center_y is not None:
                self.draw_overlay(frame, label, center_x, center_y)
            self.publish_tracking_image(frame, source_msg)

    def hand_center(self, landmarks, width, height):
        xs = [point.x for point in landmarks]
        ys = [point.y for point in landmarks]
        return (sum(xs) / len(xs) * width, sum(ys) / len(ys) * height)

    def draw_overlay(self, frame, label, center_x, center_y):
        height, width = frame.shape[:2]
        center = (width // 2, height // 2)
        target = (int(center_x), int(center_y))
        cv2.circle(frame, center, self.center_tolerance_px, (255, 180, 0), 2)
        cv2.circle(frame, target, 8, (0, 220, 0), -1)
        cv2.line(frame, center, target, (0, 220, 255), 2)
        cv2.putText(
            frame,
            label,
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )

    def publish_tracking_image(self, frame, source_msg):
        tracking_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        tracking_msg.header = source_msg.header
        self.tracking_publisher.publish(tracking_msg)

    def destroy_node(self):
        if hasattr(self, "hands"):
            self.hands.close()
        super().destroy_node()

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PalmTracker()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
