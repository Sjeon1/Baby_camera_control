from setuptools import find_packages, setup

package_name = "baby_camera_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/camera.yaml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/tapo_camera.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sungho",
    maintainer_email="sungho@example.com",
    description="ROS2 palm tracking controller for Tapo-style ONVIF cameras.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ptz_controller = baby_camera_control.ptz_controller:main",
            "palm_tracker = baby_camera_control.palm_tracker:main",
            "calibrator = baby_camera_control.calibrator:main",
            "video_recorder = baby_camera_control.video_recorder:main",
        ],
    },
)
