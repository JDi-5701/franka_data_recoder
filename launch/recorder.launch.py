from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration('config')
    task = LaunchConfiguration('task')
    dataset_name = LaunchConfiguration('dataset_name')
    gui = LaunchConfiguration('gui')
    port = LaunchConfiguration('port')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('franka_data_recorder'), 'config', 'recorder.yaml']),
            description='path to the recorder YAML config'),
        DeclareLaunchArgument(
            'task', default_value='', description='language instruction for the episode'),
        DeclareLaunchArgument(
            'dataset_name', default_value='',
            description='folder/repo_id base for this run (e.g. a task name); '
                        'recordings go to data/<dataset_name>_<timestamp>. '
                        'empty -> use the config root.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='also start the web GUI (same config); set false to run recorder only'),
        DeclareLaunchArgument(
            'port', default_value='8088', description='web GUI port (when gui:=true)'),
        Node(
            package='franka_data_recorder',
            executable='recorder',
            name='franka_data_recorder',
            output='screen',
            parameters=[{'config_file': config, 'task': task, 'dataset_name': dataset_name}],
        ),
        # Web GUI on the SAME config -> http://localhost:<port>. Disable with gui:=false.
        Node(
            package='franka_data_recorder',
            executable='gui',
            name='franka_recorder_gui',
            output='screen',
            parameters=[{'config_file': config, 'port': port}],
            condition=IfCondition(gui),
        ),
    ])
