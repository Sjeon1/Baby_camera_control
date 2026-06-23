from datetime import datetime
import os
from pathlib import Path
from threading import Lock
from time import monotonic

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VideoRecorder(Node):
    def __init__(self):
        super().__init__("video_recorder")

        self.declare_parameter("image_topic", "/baby_cam/tracking_image")
        self.declare_parameter("command_topic", "/baby_cam/ptz_cmd")
        self.declare_parameter("output_dir", "~/.ros/baby_cam_recordings")
        self.declare_parameter("filename_prefix", "baby_cam")
        self.declare_parameter("output_fps", 20.0)
        self.declare_parameter("video_codec", "mp4v")
        self.declare_parameter("draw_command_overlay", True)
        self.declare_parameter("stale_command_sec", 2.0)

        image_topic = self.get_parameter("image_topic").value
        command_topic = self.get_parameter("command_topic").value
        output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        filename_prefix = self.get_parameter("filename_prefix").value
        self.output_fps = max(1.0, float(self.get_parameter("output_fps").value))
        self.video_codec = str(self.get_parameter("video_codec").value)
        self.draw_command_overlay = bool(
            self.get_parameter("draw_command_overlay").value
        )
        self.stale_command_sec = max(
            0.0, float(self.get_parameter("stale_command_sec").value)
        )

        Path(output_dir).mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = str(Path(output_dir) / f"{filename_prefix}_{timestamp}.mp4")

        self.bridge = CvBridge()
        self.writer = None
        self.frame_size = None
        self.last_command = Twist()
        self.last_command_time = None
        self.lock = Lock()

        self.create_subscription(Twist, command_topic, self.on_command, 10)
        self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Recording {image_topic} with PTZ overlay from {command_topic} to "
            f"{self.output_path}"
        )

    def on_command(self, msg):
        with self.lock:
            self.last_command = msg
            self.last_command_time = monotonic()

    def on_image(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warning(f"Could not convert image for recording: {exc}")
            return

        if self.draw_command_overlay:
            self.draw_overlay(frame, msg)

        if self.writer is None:
            self.open_writer(frame)

        height, width = frame.shape[:2]
        if (width, height) != self.frame_size:
            frame = cv2.resize(frame, self.frame_size, interpolation=cv2.INTER_AREA)

        self.writer.write(frame)

    def open_writer(self, frame):
        height, width = frame.shape[:2]
        self.frame_size = (width, height)
        fourcc_text = (self.video_codec + "    ")[:4]
        fourcc = cv2.VideoWriter_fourcc(*fourcc_text)
        self.writer = cv2.VideoWriter(
            self.output_path, fourcc, self.output_fps, self.frame_size
        )
        if not self.writer.isOpened():
            self.writer = None
            raise RuntimeError(f"Could not open video writer: {self.output_path}")
        self.get_logger().info(
            f"Video writer opened: {self.output_path} "
            f"{width}x{height}@{self.output_fps:.1f}fps"
        )

    def draw_overlay(self, frame, msg):
        with self.lock:
            command = self.last_command
            command_time = self.last_command_time

        now = monotonic()
        age = None if command_time is None else now - command_time
        active = age is not None and age <= self.stale_command_sec
        pan = float(command.angular.z)
        tilt = float(command.angular.y)
        zoom = float(command.linear.x)

        stamp = msg.header.stamp
        ros_time = f"{stamp.sec}.{stamp.nanosec:09d}"
        status = "PTZ active" if active and any((pan, tilt, zoom)) else "PTZ idle"
        lines = [
            f"time {ros_time}",
            f"{status} pan={pan:+.3f} tilt={tilt:+.3f} zoom={zoom:+.3f}",
        ]

        x, y = 12, 28
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 1
        line_height = 24
        widths = [
            cv2.getTextSize(line, font, scale, thickness)[0][0]
            for line in lines
        ]
        box_w = max(widths) + 24
        box_h = len(lines) * line_height + 16

        overlay = frame.copy()
        cv2.rectangle(overlay, (6, 6), (6 + box_w, 6 + box_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)

        color = (80, 220, 80) if active else (210, 210, 210)
        for index, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (x, y + index * line_height),
                font,
                scale,
                color,
                thickness,
                cv2.LINE_AA,
            )

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            self.get_logger().info(f"Saved recording: {self.output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VideoRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
