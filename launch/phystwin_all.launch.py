"""One-shot: auto-launch ALL connected RealSense cameras + the PhysTwin recorder.

    ros2 launch franka_data_recorder phystwin_all.launch.py \
        dataset_name:=microwave_door_ep01 \
        calibrate_pkl:=/abs/calibrate.pkl gravity_json:=/abs/gravity.json

Starts every connected camera (via realsense_all.launch.py) and, after a short delay so the
camera topics are up, the phystwin recorder in AUTO-DISCOVER mode (records whatever cameras
are present). Control recording with the usual Trigger services / GUI:
    ros2 service call /franka_data_recorder/start_recording std_srvs/srv/Trigger
    ros2 service call /franka_data_recorder/stop_recording  std_srvs/srv/Trigger
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("franka_data_recorder")
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "realsense_all.launch.py")),
        launch_arguments={
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fps": LaunchConfiguration("fps"),
        }.items())

    recorder = Node(
        package="franka_data_recorder", executable="phystwin_recorder",
        name="franka_data_recorder", prefix="/usr/bin/python3", output="screen",
        parameters=[{
            "config_file": os.path.join(share, "config", "recorder_phystwin_auto.yaml"),
            "auto_discover": True,
            "dataset_name": LaunchConfiguration("dataset_name"),
            "task": LaunchConfiguration("task"),
            "calibrate_pkl": LaunchConfiguration("calibrate_pkl"),
            "gravity_json": LaunchConfiguration("gravity_json"),
        }])

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        DeclareLaunchArgument("width", default_value="848"),
        DeclareLaunchArgument("height", default_value="480"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("dataset_name", default_value=""),
        DeclareLaunchArgument("task", default_value="articulated object free motion"),
        DeclareLaunchArgument("calibrate_pkl", default_value=""),
        DeclareLaunchArgument("gravity_json", default_value=""),
        cameras,
        # give the camera drivers a few seconds to come up before the recorder discovers them
        TimerAction(period=6.0, actions=[recorder]),
    ])
