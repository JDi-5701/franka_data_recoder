"""USB camera bring-up for recording: usb_cam (mjpeg2rgb -> /image_raw, rgb8 1280x720@30)
plus an optional image_proc rectify_node (-> /image_rect). One launch instead of two
`ros2 run` terminals.

    ros2 launch franka_data_recorder camera.launch.py
    ros2 launch franka_data_recorder camera.launch.py rectify:=false
    ros2 launch franka_data_recorder camera.launch.py video_device:=/dev/video2 \
        camera_info_url:=file:///abs/path/to/calib.yaml

The camera calibration (camera_info_url) is used ONLY for rectification/undistortion; it is
NOT fed to pi0.5 (the model takes raw pixels, no intrinsics/extrinsics). Default points at the
current calibration on `prs`; move the yaml into this package's config/ for portability.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Resolution / fps / pixel_format are kept as Python literals (correct param types); the values
# that actually change between machines are exposed as launch arguments.
_DEFAULT_CAMERA_INFO = 'file:///home/prs/ros_ml_ws/src/pi05/config/isy_webcam_camera_info.yaml'


def generate_launch_description():
    video_device = LaunchConfiguration('video_device')
    camera_info_url = LaunchConfiguration('camera_info_url')
    rectify = LaunchConfiguration('rectify')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_info_url', default_value=_DEFAULT_CAMERA_INFO),
        DeclareLaunchArgument('rectify', default_value='true',
                              description='also start image_proc rectify_node (-> /image_rect)'),

        Node(
            package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'image_width': 1280,
                'image_height': 720,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
                'camera_name': 'isy_usb_cam',
                'camera_frame_id': 'usb_cam',
                'camera_info_url': camera_info_url,
            }],
        ),

        Node(
            package='image_proc', executable='rectify_node', name='rectify',
            output='screen',
            condition=IfCondition(rectify),
            remappings=[('image', '/image_raw'), ('camera_info', '/camera_info')],
        ),
    ])
