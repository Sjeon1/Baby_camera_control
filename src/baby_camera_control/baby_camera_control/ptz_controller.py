from threading import Lock
from time import monotonic

from geometry_msgs.msg import Twist
from onvif import ONVIFCamera
import rclpy
from rclpy.node import Node


class PtzController(Node):
    def __init__(self):
        super().__init__("ptz_controller")

        self.declare_parameter("host", "")
        self.declare_parameter("port", 2020)
        self.declare_parameter("username", "")
        self.declare_parameter("password", "")
        self.declare_parameter("command_topic", "/baby_cam/ptz_cmd")
        self.declare_parameter("profile_index", 0)
        self.declare_parameter("use_relative_move", True)
        self.declare_parameter("min_command_interval_sec", 1.5)
        # ContinuousMove-only parameters (ignored when use_relative_move=True)
        self.declare_parameter("pan_speed_scale", 0.45)
        self.declare_parameter("tilt_speed_scale", 0.45)
        self.declare_parameter("zoom_speed_scale", 0.0)
        self.declare_parameter("command_timeout_sec", 0.4)
        self.declare_parameter("command_change_epsilon", 0.03)
        self.declare_parameter("center_on_start", True)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.username = self.get_parameter("username").value
        self.password = self.get_parameter("password").value
        command_topic = self.get_parameter("command_topic").value
        self.profile_index = int(self.get_parameter("profile_index").value)
        self.use_relative_move = bool(self.get_parameter("use_relative_move").value)
        self.min_command_interval_sec = max(
            0.0, float(self.get_parameter("min_command_interval_sec").value)
        )
        self.pan_speed_scale = float(self.get_parameter("pan_speed_scale").value)
        self.tilt_speed_scale = float(self.get_parameter("tilt_speed_scale").value)
        self.zoom_speed_scale = float(self.get_parameter("zoom_speed_scale").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.command_change_epsilon = max(
            0.0, float(self.get_parameter("command_change_epsilon").value)
        )
        self.center_on_start = bool(self.get_parameter("center_on_start").value)

        self._validate_parameters()

        self.ptz = None
        self.profile_token = None
        self.move_request = None
        self.relative_move_request = None
        self.stop_request = None
        self.lock = Lock()
        self.last_move_sent_time = None
        # ContinuousMove state
        self.last_command_time = None
        self.last_pan = 0.0
        self.last_tilt = 0.0
        self.last_zoom = 0.0
        self.command_active = False

        self.connect()
        if self.center_on_start:
            self.goto_center()
        self.create_subscription(Twist, command_topic, self.on_twist, 10)

        if not self.use_relative_move:
            self.stop_timer = self.create_timer(
                max(0.05, self.command_timeout_sec / 2.0), self.stop_if_timed_out
            )

        mode = "RelativeMove (calibrated)" if self.use_relative_move else "ContinuousMove"
        self.get_logger().info(
            f"Listening on {command_topic} [{mode}]. "
            "Use Twist angular.z for pan, angular.y for tilt."
        )

    def _validate_parameters(self):
        missing = [
            name
            for name, value in {
                "host": self.host,
                "username": self.username,
                "password": self.password,
            }.items()
            if not value
        ]
        if missing:
            raise ValueError(f"Missing required ONVIF parameter(s): {', '.join(missing)}")

    def connect(self):
        camera = ONVIFCamera(self.host, self.port, self.username, self.password)
        media = camera.create_media_service()
        self.ptz = camera.create_ptz_service()

        profiles = media.GetProfiles()
        if self.profile_index >= len(profiles):
            raise IndexError(
                f"profile_index {self.profile_index} out of range; "
                f"camera has {len(profiles)} profile(s)"
            )

        self.profile_token = profiles[self.profile_index].token

        self.move_request = self.ptz.create_type("ContinuousMove")
        self.move_request.ProfileToken = self.profile_token

        self.relative_move_request = self.ptz.create_type("RelativeMove")
        self.relative_move_request.ProfileToken = self.profile_token

        self.stop_request = self.ptz.create_type("Stop")
        self.stop_request.ProfileToken = self.profile_token
        self.stop_request.PanTilt = True
        self.stop_request.Zoom = True

        self.get_logger().info(f"Connected to ONVIF camera {self.host}:{self.port}")

    def goto_center(self):
        with self.lock:
            try:
                request = self.ptz.create_type("AbsoluteMove")
                request.ProfileToken = self.profile_token
                request.Position = {
                    "PanTilt": {"x": 0.0, "y": 0.0},
                    "Zoom": {"x": 0.0},
                }
                self.ptz.AbsoluteMove(request)
                self.get_logger().info("Sent camera to absolute center position")
            except Exception as exc:
                self.get_logger().warning(
                    f"Could not send camera to absolute center position: {exc}"
                )
                self.goto_home()

    def goto_home(self):
        try:
            request = self.ptz.create_type("GotoHomePosition")
            request.ProfileToken = self.profile_token
            self.ptz.GotoHomePosition(request)
            self.get_logger().info("Sent camera to home position")
        except Exception as exc:
            self.get_logger().warning(f"Could not send camera to home position: {exc}")

    def on_twist(self, msg):
        if self.use_relative_move:
            self._on_twist_relative(msg)
        else:
            self._on_twist_continuous(msg)

    # ── RelativeMove path ────────────────────────────────────────────────────

    def _on_twist_relative(self, msg):
        pan = float(msg.angular.z)
        tilt = float(msg.angular.y)
        zoom = float(msg.linear.x)

        if pan == 0.0 and tilt == 0.0 and zoom == 0.0:
            return

        now = monotonic()
        with self.lock:
            if (
                self.last_move_sent_time is not None
                and now - self.last_move_sent_time < self.min_command_interval_sec
            ):
                return

            self.relative_move_request.Translation = {
                "PanTilt": {"x": pan, "y": tilt},
                "Zoom": {"x": zoom},
            }
            try:
                self.ptz.RelativeMove(self.relative_move_request)
                self.last_move_sent_time = now
                self.get_logger().info(f"RelativeMove pan={pan:.4f} tilt={tilt:.4f}")
            except Exception as exc:
                self.get_logger().error(f"ONVIF RelativeMove failed: {exc}")

    # ── ContinuousMove path (unchanged) ────────────────────────────────────

    def _on_twist_continuous(self, msg):
        pan = self._clamp(msg.angular.z * self.pan_speed_scale)
        tilt = self._clamp(msg.angular.y * self.tilt_speed_scale)
        zoom = self._clamp(msg.linear.x * self.zoom_speed_scale)
        now = monotonic()

        with self.lock:
            if pan == 0.0 and tilt == 0.0 and zoom == 0.0:
                if self.command_active:
                    self._stop_locked()
                return

            self.last_command_time = now
            if self._should_skip_move(pan, tilt, zoom, now):
                return

            self.command_active = True
            self.move_request.Velocity = {
                "PanTilt": {"x": pan, "y": tilt},
                "Zoom": {"x": zoom},
            }
            try:
                self.ptz.ContinuousMove(self.move_request)
                self.last_move_sent_time = now
                self.last_pan = pan
                self.last_tilt = tilt
                self.last_zoom = zoom
            except Exception as exc:
                self.get_logger().error(f"ONVIF ContinuousMove failed: {exc}")

    def _should_skip_move(self, pan, tilt, zoom, now):
        if self.last_move_sent_time is None:
            return False
        elapsed = now - self.last_move_sent_time
        change = max(
            abs(pan - self.last_pan),
            abs(tilt - self.last_tilt),
            abs(zoom - self.last_zoom),
        )
        if elapsed < self.min_command_interval_sec:
            return True
        return change < self.command_change_epsilon

    def stop(self):
        if self.ptz is None:
            return
        with self.lock:
            self._stop_locked()

    def stop_if_timed_out(self):
        if self.ptz is None:
            return
        with self.lock:
            if not self.command_active or self.last_command_time is None:
                return
            if monotonic() - self.last_command_time >= self.command_timeout_sec:
                self._stop_locked()

    def _stop_locked(self):
        try:
            self.ptz.Stop(self.stop_request)
            self.command_active = False
            self.last_command_time = None
            self.last_move_sent_time = None
            self.last_pan = 0.0
            self.last_tilt = 0.0
            self.last_zoom = 0.0
        except Exception as exc:
            self.get_logger().debug(f"ONVIF Stop failed: {exc}")

    @staticmethod
    def _clamp(value):
        return max(-1.0, min(1.0, float(value)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PtzController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
