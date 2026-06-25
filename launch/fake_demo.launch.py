"""Bring up the whole GUI demo WITHOUT the robot:
  fake_publisher  (continuous fake data)  +  recorder  +  gui  -- all on recorder_fake.yaml.

Run (conda ros_ml):  ros2 launch franka_data_recorder fake_demo.launch.py
Then open http://localhost:8088 (tunnel it to your laptop if remote).
"""
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cfg = PathJoinSubstitution(
        [FindPackageShare('franka_data_recorder'), 'config', 'recorder_fake.yaml'])
    return LaunchDescription([
        Node(package='franka_data_recorder', executable='fake',
             name='fake_data_publisher', output='screen'),
        Node(package='franka_data_recorder', executable='recorder',
             name='franka_data_recorder', output='screen',
             parameters=[{'config_file': cfg}]),
        Node(package='franka_data_recorder', executable='gui',
             name='franka_recorder_gui', output='screen',
             parameters=[{'config_file': cfg}]),
    ])
