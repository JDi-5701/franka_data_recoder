"""Launch the PhysTwin recorder with the phystwin config by default.

    ros2 launch franka_data_recorder phystwin_recorder.launch.py \
        dataset_name:=microwave_door_ep01 \
        calibrate_pkl:=/abs/path/calibrate.pkl \
        gravity_json:=/abs/path/gravity.json

Override config_file:= to point at your edited recorder_phystwin.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("franka_data_recorder")
    default_cfg = os.path.join(share, "config", "recorder_phystwin_auto.yaml")

    args = [
        # match recorder.launch.py: pin RMW + run through system python so ROS libs come from
        # /opt/ros, not the ros_ml Conda env (avoids undefined-symbol / typesupport errors).
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        DeclareLaunchArgument("config_file", default_value=default_cfg),
        DeclareLaunchArgument("dataset_name", default_value=""),
        DeclareLaunchArgument("task", default_value="articulated object free motion"),
        DeclareLaunchArgument("calibrate_pkl", default_value=""),
        DeclareLaunchArgument("gravity_json", default_value=""),
    ]
    node = Node(
        package="franka_data_recorder",
        executable="phystwin_recorder",
        name="franka_data_recorder",
        prefix="/usr/bin/python3",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "dataset_name": LaunchConfiguration("dataset_name"),
            "task": LaunchConfiguration("task"),
            "calibrate_pkl": LaunchConfiguration("calibrate_pkl"),
            "gravity_json": LaunchConfiguration("gravity_json"),
        }],
    )
    return LaunchDescription(args + [node])
