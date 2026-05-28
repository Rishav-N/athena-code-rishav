from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_camera_relay',
            executable='camera_relay',
            name='usb_camera_relay',
            output='screen',
            parameters=[{
                # Leave empty to auto-discover, or set explicitly:
                # 'devices': ['/dev/video0', '/dev/video2'],
                'devices': [''],
                'fps': 10.0,
                'width': 640,
                'height': 480,
                'topic_prefix': '/cameras',
            }],
        ),
    ])
