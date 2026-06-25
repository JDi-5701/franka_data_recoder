from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    task = LaunchConfiguration('task')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('franka_data_recorder'), 'config', 'recorder.yaml']),
            description='path to the recorder YAML config'),
        DeclareLaunchArgument(
            'task', default_value='', description='language instruction for the episode'),
        Node(
            package='franka_data_recorder',
            executable='recorder',
            name='franka_data_recorder',
            output='screen',
            parameters=[{'config_file': config, 'task': task}],
        ),
    ])
