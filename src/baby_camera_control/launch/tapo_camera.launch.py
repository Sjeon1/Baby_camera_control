from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    start_ptz_controller = LaunchConfiguration("start_ptz_controller")
    start_palm_tracker = LaunchConfiguration("start_palm_tracker")
    start_video_recorder = LaunchConfiguration("start_video_recorder")

    default_config = PathJoinSubstitution(
        [FindPackageShare("baby_camera_control"), "config", "camera.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML config with RTSP, ONVIF, and palm tracking settings.",
            ),
            DeclareLaunchArgument(
                "start_ptz_controller",
                default_value="true",
                description="Start the ONVIF PTZ controller.",
            ),
            DeclareLaunchArgument(
                "start_palm_tracker",
                default_value="true",
                description="Track open palms and publish pan/tilt commands.",
            ),
            DeclareLaunchArgument(
                "start_video_recorder",
                default_value="true",
                description="Record annotated tracking video with PTZ command overlay.",
            ),
            Node(
                package="baby_camera_control",
                executable="rtsp_image_publisher",
                name="rtsp_image_publisher",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="baby_camera_control",
                executable="ptz_controller",
                name="ptz_controller",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_ptz_controller),
                respawn=True,
                respawn_delay=3.0,
            ),
            Node(
                package="baby_camera_control",
                executable="palm_tracker",
                name="palm_tracker",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_palm_tracker),
            ),
            Node(
                package="baby_camera_control",
                executable="video_recorder",
                name="video_recorder",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(start_video_recorder),
            ),
        ]
    )
