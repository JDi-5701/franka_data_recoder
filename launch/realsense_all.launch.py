"""Auto-launch EVERY connected Intel RealSense camera (no hardcoded serials).

Enumerates connected devices with pyrealsense2 and includes realsense2_camera's rs_launch.py
once per device, with:
  * a stable per-camera name derived from the model (d405/d435/d455; serial suffix on clash),
  * align_depth enabled (depth pixel-aligned to color -> required by PhysTwin),
  * a common resolution/fps for all cameras (default 848x480x30, PhysTwin's default).

    ros2 launch franka_data_recorder realsense_all.launch.py
    ros2 launch franka_data_recorder realsense_all.launch.py width:=1280 height:=720 fps:=30

Topics follow realsense2_camera's own convention for the chosen camera_name (verify with
`ros2 topic list`). The phystwin recorder AUTO-DISCOVERS them, so exact naming doesn't matter.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _enumerate_and_launch(context, *args, **kwargs):
    w = context.perform_substitution(LaunchConfiguration("width"))
    h = context.perform_substitution(LaunchConfiguration("height"))
    fps = context.perform_substitution(LaunchConfiguration("fps"))
    profile = f"{w}x{h}x{fps}"

    try:
        import pyrealsense2 as rs
    except ImportError:
        raise RuntimeError(
            "pyrealsense2 not importable in the launch env; install it "
            "(pip install pyrealsense2) or launch cameras manually by serial.")

    rs_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")

    actions, used = [], {}
    devices = list(rs.context().query_devices())
    if not devices:
        raise RuntimeError("no RealSense devices found. Check USB / `rs-enumerate-devices`.")

    for dev in devices:
        serial = dev.get_info(rs.camera_info.serial_number)
        model = dev.get_info(rs.camera_info.name)          # e.g. "Intel RealSense D405"
        short = model.split()[-1].lower()                   # -> "d405"
        name = short if short not in used else f"{short}_{serial[-4:]}"
        used[name] = serial
        print(f"[realsense_all] {model} serial={serial} -> camera_name={name} @ {profile}")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                "serial_no": serial,
                "camera_name": name,
                "align_depth.enable": "true",
                "rgb_camera.color_profile": profile,
                "depth_module.depth_profile": profile,
            }.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="848"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        OpaqueFunction(function=_enumerate_and_launch),
    ])
