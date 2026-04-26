from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='nav2_config',
            executable='gui',  
            name='nav2_config_gui',
            output='screen'
        )
    ])