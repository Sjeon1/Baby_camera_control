# Baby Camera Palm Tracking

ROS2 package for controlling a TP-Link Tapo-style ONVIF camera with open-palm tracking.

The package:

- publishes the camera RTSP stream as `sensor_msgs/Image`
- detects an open palm with MediaPipe
- sends pan/tilt commands through ONVIF so the camera follows the palm

## Camera Setup

In the Tapo app, create a separate camera account for RTSP/ONVIF:

```text
Camera -> Device Settings -> Advanced Settings -> Camera Account
```

Check the camera IP from the router DHCP list or from the Tapo app.

Before running ROS, confirm the RTSP stream in VLC:

```text
rtsp://username:password@CAMERA_IP/stream1
```

## Dependencies

Install ROS2 packages:

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-sensor-msgs ros-${ROS_DISTRO}-geometry-msgs
```

Install Python packages used by the nodes:

```bash
python3 -m pip install -r requirements.txt
```

## Configure

Edit:

```text
src/baby_camera_control/config/camera.yaml
```

Set your local camera values:

```yaml
host: "192.168.1.50"
username: "camera_username"
password: "camera_password"
```

For Tapo cameras, ONVIF usually uses port `2020` and RTSP uses port `554`.

## Build

From the repository root:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
source install/setup.bash
```

## Run

Start the camera stream, PTZ controller, and palm tracker:

```bash
ros2 launch baby_camera_control tapo_camera.launch.py
```

Palm behavior:

- Open palm: the camera follows the palm toward the center of the image.
- No palm or closed hand: the camera stops.

View the published topics:

```bash
ros2 topic echo /baby_cam/image_raw/header
ros2 topic echo /baby_cam/tracking_image/header
```

Command mapping:

- `angular.z`: pan left/right
- `angular.y`: tilt up/down
- `linear.x`: zoom, disabled by default with `zoom_speed_scale: 0.0`

## Security

Keep RTSP and ONVIF on the local network. Do not commit real camera credentials, expose the camera with router port forwarding, or reuse your main Tapo account password for RTSP/ONVIF.
