"""Run the one-time PhysTwin Charuco calibration under system python (ros_ml-safe).

Mirrors recorder.launch.py's workaround: pins RMW + runs through /usr/bin/python3 so ROS libs
come from /opt/ros, not the ros_ml Conda env. Calibration auto-discovers the cameras (same
index order as the recorder), so you only pass output dir + gravity mode.

    ros2 launch franka_data_recorder phystwin_calibrate.launch.py \
        out_dir:=$HOME/phystwin_calib gravity:=board
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        DeclareLaunchArgument("out_dir", default_value=os.path.expanduser("~/phystwin_calib")),
        DeclareLaunchArgument("gravity", default_value="board",
                              description="board | base | 'gx,gy,gz'"),
        DeclareLaunchArgument("discover_sec", default_value="4.0"),
        Node(
            package="franka_data_recorder",
            executable="phystwin_calibrate",
            name="phystwin_calibrate",
            prefix="/usr/bin/python3",
            output="screen",
            arguments=[
                "--out-dir", LaunchConfiguration("out_dir"),
                "--gravity", LaunchConfiguration("gravity"),
                "--discover-sec", LaunchConfiguration("discover_sec"),
            ],
        ),
    ])
