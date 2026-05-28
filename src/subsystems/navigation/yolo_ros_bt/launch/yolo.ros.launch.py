from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('yolo_ros_bt'), 'config', 'yolo_params.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the YOLO params YAML',
        ),
        Node(
            package='yolo_ros_bt',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
